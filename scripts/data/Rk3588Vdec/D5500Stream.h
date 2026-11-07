// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// What the AppleD5500 service (Rk3588VdecD5500.cpp) does with a stream's bytes, without the kernel in it so that tests/test-d5500.cpp can run it on the host:
//
//   parseExtradata   the record the plug-in leaves at offset 0 of the ring buffer (avcC / hvcC, or parameter sets in Annex B, or behind length fields) into the
//                    parameter sets as the core's parser wants them (Annex B), and what the plug-in asks of the "hardware" (the stream's size, bit depth, ...)
//   detectCodec      H.264 or H.265, from the same bytes
//   frameAccessUnit  one access unit as VideoToolbox's plug-in hands it over (length prefixed, or Annex B) into what the parser wants: Annex B, ended by an
//                    access unit delimiter so that the parser gives the picture back in this call
//
// H.264 and H.265 differ in the NAL header (one byte: type in the low 5 bits; two bytes: type in bits 1..6 of the first), the parameter sets (SPS, PPS; VPS, SPS,
// PPS), the records (avcC; hvcC) and the delimiter. Nothing else in here depends on the codec.
#ifndef RK3588_D5500_STREAM_H
#define RK3588_D5500_STREAM_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "core/VdecCore.h"   // kVdecH264, kVdecH265

namespace d5500 {

// What the plug-in wants to know about the stream (the hardware of the original reads it from the parameter sets): from the SPS.
struct StreamInfo {
  uint32_t profile, level;
  uint32_t chromaFormat, lumaMinus8, chromaMinus8;   // chroma_format_idc, bit_depth_*_minus8
  uint32_t widthMbs, heightMbs, numRefFrames;        // the coded size in 16x16 blocks; the pictures the stream keeps as references (H.265: sps_max_dec_pic_buffering_minus1)
  bool frameMbsOnly, valid;
};

struct BitReader {
  const uint8_t *data;
  size_t bits, at;
  bool bad;
  uint32_t u1() {
    if (at >= bits) {
      bad = true;
      return 0;
    }
    const uint32_t v = (data[at >> 3] >> (7 - (at & 7))) & 1;
    ++at;
    return v;
  }
  uint32_t u(unsigned n) {
    uint32_t v = 0;
    while (n--) v = v << 1 | u1();
    return v;
  }
  uint32_t ue() {
    unsigned zeros = 0;
    while (!u1()) {
      if (bad || ++zeros > 31) {
        bad = true;
        return 0;
      }
    }
    return zeros ? ((1u << zeros) - 1 + u(zeros)) : 0;
  }
  int32_t se() {
    const uint32_t k = ue();
    return (k & 1) ? static_cast<int32_t>((k + 1) / 2) : -static_cast<int32_t>(k / 2);
  }
  void skip(unsigned n) {
    while (n--) (void)u1();
  }
};

inline unsigned nalType(int codec, uint8_t header) { return codec == kVdecH265 ? (header >> 1) & 0x3f : header & 0x1f; }

// One sequence parameter set (the NAL unit without start code or length, header byte first).
inline bool parseSps(const uint8_t *nal, size_t size, StreamInfo *info) {
  uint8_t rbsp[512];
  if (size < 5 || size > sizeof rbsp + 1 || (nal[0] & 0x1f) != 7) return false;
  size_t n = 0;
  for (size_t i = 1; i < size; ++i) {   // the emulation prevention bytes out
    if (n >= 2 && rbsp[n - 1] == 0 && rbsp[n - 2] == 0 && nal[i] == 3) continue;
    rbsp[n++] = nal[i];
  }
  BitReader r = {rbsp, n * 8, 0, false};
  memset(info, 0, sizeof *info);
  info->profile = r.u(8);
  (void)r.u(8);
  info->level = r.u(8);
  (void)r.ue();   // sps id
  info->chromaFormat = 1;
  switch (info->profile) {
    case 100: case 110: case 122: case 244: case 44: case 83: case 86: case 118: case 128: case 138: case 139: case 134: case 135: {
      info->chromaFormat = r.ue();
      if (info->chromaFormat == 3) (void)r.u1();
      info->lumaMinus8 = r.ue();
      info->chromaMinus8 = r.ue();
      (void)r.u1();   // qpprime_y_zero_transform_bypass
      if (r.u1()) {   // scaling matrices: only their size matters here
        const unsigned lists = info->chromaFormat != 3 ? 8 : 12;
        for (unsigned i = 0; i < lists && !r.bad; ++i) {
          if (!r.u1()) continue;
          int last = 8, next = 8;
          for (unsigned j = 0, count = i < 6 ? 16 : 64; j < count; ++j) {
            if (next) {
              next = (last + r.se() + 256) % 256;
            }
            last = next ? next : last;
          }
        }
      }
      break;
    }
    default: break;
  }
  (void)r.ue();   // log2_max_frame_num_minus4
  const uint32_t pocType = r.ue();
  if (pocType == 0) {
    (void)r.ue();
  } else if (pocType == 1) {
    (void)r.u1();
    (void)r.se();
    (void)r.se();
    const uint32_t cycle = r.ue();
    for (uint32_t i = 0; i < cycle && i < 256 && !r.bad; ++i) (void)r.se();
  }
  info->numRefFrames = r.ue();
  (void)r.u1();   // gaps_in_frame_num_value_allowed_flag
  info->widthMbs = r.ue() + 1;
  const uint32_t mapUnits = r.ue() + 1;
  info->frameMbsOnly = r.u1() != 0;
  info->heightMbs = info->frameMbsOnly ? mapUnits : mapUnits * 2;
  info->valid = !r.bad && info->widthMbs && info->heightMbs && info->widthMbs <= 1024 && info->heightMbs <= 1024;
  return info->valid;
}

// One H.265 sequence parameter set (the NAL unit without start code or length, the two header bytes first): as far as the decoded picture buffer size.
// The sizes are in luma samples in the SPS; `widthMbs` and `heightMbs` are those in 16x16 blocks, rounded up, which is how the plug-in is told.
inline bool parseSpsHevc(const uint8_t *nal, size_t size, StreamInfo *info) {
  uint8_t rbsp[512];
  if (size < 16 || (((nal[0] >> 1) & 0x3f) != 33)) return false;
  size_t n = 0;
  for (size_t i = 2; i < size && n < sizeof rbsp; ++i) {   // the emulation prevention bytes out (what follows the part read here does not matter)
    if (n >= 2 && rbsp[n - 1] == 0 && rbsp[n - 2] == 0 && nal[i] == 3) continue;
    rbsp[n++] = nal[i];
  }
  BitReader r = {rbsp, n * 8, 0, false};
  memset(info, 0, sizeof *info);
  r.skip(4);   // sps_video_parameter_set_id
  const uint32_t subLayers = r.u(3);   // sps_max_sub_layers_minus1
  r.skip(1);   // sps_temporal_id_nesting_flag
  // profile_tier_level(1, subLayers): general_profile_space 2, tier 1, profile_idc 5, 32 compatibility flags, 4 flags, 43 + 1 bits, general_level_idc 8
  r.skip(3);
  info->profile = r.u(5);
  r.skip(32 + 4 + 43 + 1);
  info->level = r.u(8);
  uint32_t profilePresent = 0, levelPresent = 0;
  for (uint32_t i = 0; i < subLayers; ++i) {
    profilePresent |= r.u1() << i;
    levelPresent |= r.u1() << i;
  }
  if (subLayers > 0)
    for (uint32_t i = subLayers; i < 8; ++i) r.skip(2);   // reserved_zero_2bits
  for (uint32_t i = 0; i < subLayers && !r.bad; ++i) {
    if (profilePresent >> i & 1) r.skip(88);
    if (levelPresent >> i & 1) r.skip(8);
  }
  (void)r.ue();   // sps_seq_parameter_set_id
  info->chromaFormat = r.ue();
  if (info->chromaFormat == 3) r.skip(1);   // separate_colour_plane_flag
  const uint32_t width = r.ue(), height = r.ue();
  if (r.u1())   // conformance_window_flag: four offsets
    for (unsigned i = 0; i < 4; ++i) (void)r.ue();
  info->lumaMinus8 = r.ue();
  info->chromaMinus8 = r.ue();
  (void)r.ue();   // log2_max_pic_order_cnt_lsb_minus4
  const bool ordering = r.u1() != 0;   // sps_sub_layer_ordering_info_present_flag
  uint32_t dpbMinus1 = 0;
  for (uint32_t i = ordering ? 0 : subLayers; i <= subLayers && !r.bad; ++i) {
    dpbMinus1 = r.ue();   // sps_max_dec_pic_buffering_minus1 (the last one read is the highest sub-layer's)
    (void)r.ue();         // sps_max_num_reorder_pics
    (void)r.ue();         // sps_max_latency_increase_plus1
  }
  info->numRefFrames = dpbMinus1;
  info->widthMbs = (width + 15) / 16;
  info->heightMbs = (height + 15) / 16;
  info->frameMbsOnly = true;
  info->valid = !r.bad && width && height && width <= 16384 && height <= 16384 && info->chromaFormat <= 3 && info->lumaMinus8 <= 8 && info->chromaMinus8 <= 8 && dpbMinus1 <= 16;
  return info->valid;
}

// Do the length fields (of `field` bytes) tile the data exactly? What looks like an Annex B start code at the front (00 00 01 ..) can just be the length of a
// NAL unit of 256..511 bytes behind a four byte field (vt17: a 433 byte slice, 00 00 01 ad, was taken for Annex B), so the stream's own framing, which
// CreateDecoder announced, is tried first.
inline bool tilesAsLengthPrefixed(const uint8_t *data, size_t size, size_t field) {
  if (field != 1 && field != 2 && field != 4) return false;
  size_t at = 0;
  while (at + field <= size) {
    size_t length = 0;
    for (size_t k = 0; k < field; ++k) length = length << 8 | data[at + k];
    if (!length || at + field + length > size) return false;
    at += field + length;
  }
  return at == size && size != 0;
}

inline bool startsAnnexB(const uint8_t *data, size_t size, size_t nalSize) {
  return size >= 4 && data[0] == 0 && data[1] == 0 && (data[2] == 1 || (data[2] == 0 && data[3] == 1)) && !tilesAsLengthPrefixed(data, size, nalSize);
}

// An avcC record (configurationVersion 1, the reserved bits of the length and parameter set count bytes set).
inline bool looksLikeAvcC(const uint8_t *data, size_t size) { return size >= 7 && data[0] == 1 && (data[4] & 0xfc) == 0xfc && (data[5] & 0xe0) == 0xe0; }

// An hvcC record (ISO/IEC 14496-15: 23 bytes of header, configurationVersion 1 and the reserved bits of bytes 13, 15, 16, 17, 18 set, then the arrays).
inline bool looksLikeHvcC(const uint8_t *data, size_t size) {
  return size >= 23 && data[0] == 1 && (data[13] & 0xf0) == 0xf0 && (data[15] & 0xfc) == 0xfc && (data[16] & 0xfc) == 0xfc && (data[17] & 0xf8) == 0xf8 && (data[18] & 0xf8) == 0xf8;
}

// H.264 or H.265: the record's own shape, or the first NAL unit of what is there (a parameter set: an SPS or PPS for H.264, a VPS, SPS or PPS for H.265, whose
// second header byte has a non-zero temporal id plus one and, in the first byte, a type of 32..34 where an H.264 header has nothing that looks like it).
inline int detectCodec(const uint8_t *data, size_t size, size_t nalSize) {
  if (looksLikeHvcC(data, size)) return kVdecH265;
  if (looksLikeAvcC(data, size)) return kVdecH264;
  const uint8_t *nal = nullptr;
  size_t left = 0;
  if (startsAnnexB(data, size, nalSize)) {
    size_t at = data[2] == 1 ? 3 : 4;
    nal = data + at;
    left = size - at;
  } else {
    const size_t field = (nalSize == 1 || nalSize == 2 || nalSize == 4) ? nalSize : 4;
    if (size > field) {
      nal = data + field;
      left = size - field;
    }
  }
  if (left >= 2 && (nal[0] & 0x80) == 0) {
    const unsigned hevcType = (nal[0] >> 1) & 0x3f;
    if (hevcType >= 32 && hevcType <= 34 && (nal[1] & 7) != 0) return kVdecH265;
  }
  return kVdecH264;
}

// The extradata of the stream (the avcC or hvcC record, or parameter sets in Annex B or behind length fields) into the parameter sets as the parser wants them
// (Annex B, four byte start codes). Returns the number of parameter set NAL units.
inline unsigned parseExtradata(int codec, const uint8_t *data, size_t size, uint32_t nalSize, uint8_t *out, size_t outSize, size_t *outLength, StreamInfo *info) {
  size_t n = 0;
  unsigned units = 0;
  bool haveInfo = false;
  const bool hevc = codec == kVdecH265;
  auto add = [&](const uint8_t *nal, size_t length) {
    if (!length || n + 4 + length > outSize) return;
    // the parser needs the parameter sets; an hvcC record's arrays can carry SEI too, which it does not (H.264 keeps what the record or the list has, as before)
    const unsigned type = nalType(codec, nal[0]);
    if (hevc && !(type >= 32 && type <= 34)) return;
    out[n] = 0, out[n + 1] = 0, out[n + 2] = 0, out[n + 3] = 1;
    memcpy(out + n + 4, nal, length);
    n += 4 + length;
    ++units;
    if (!haveInfo && type == (hevc ? 33u : 7u)) haveInfo = hevc ? parseSpsHevc(nal, length, info) : parseSps(nal, length, info);
  };
  if (!hevc && looksLikeAvcC(data, size)) {   // avcC
    size_t at = 5;
    const unsigned spsCount = data[at++] & 0x1f;
    for (unsigned i = 0; i < spsCount && at + 2 <= size; ++i) {
      const size_t length = static_cast<size_t>(data[at]) << 8 | data[at + 1];
      at += 2;
      if (at + length > size) break;
      add(data + at, length);
      at += length;
    }
    if (at < size) {
      const unsigned ppsCount = data[at++];
      for (unsigned i = 0; i < ppsCount && at + 2 <= size; ++i) {
        const size_t length = static_cast<size_t>(data[at]) << 8 | data[at + 1];
        at += 2;
        if (at + length > size) break;
        add(data + at, length);
        at += length;
      }
    }
  } else if (hevc && looksLikeHvcC(data, size)) {   // hvcC: arrays of (type, count, count x (length, NAL unit))
    size_t at = 22;
    const unsigned arrays = data[at++];
    for (unsigned a = 0; a < arrays && at + 3 <= size; ++a) {
      const unsigned count = static_cast<unsigned>(data[at + 1]) << 8 | data[at + 2];
      at += 3;
      for (unsigned i = 0; i < count && at + 2 <= size; ++i) {
        const size_t length = static_cast<size_t>(data[at]) << 8 | data[at + 1];
        at += 2;
        if (at + length > size) break;
        add(data + at, length);
        at += length;
      }
    }
  } else if (startsAnnexB(data, size, nalSize)) {   // Annex B
    size_t start = 0;
    while (start + 3 < size) {
      size_t body = start + (data[start + 2] == 1 ? 3 : 4);
      size_t end = body;
      while (end + 2 < size && !(data[end] == 0 && data[end + 1] == 0 && (data[end + 2] == 1 || (end + 3 < size && data[end + 2] == 0 && data[end + 3] == 1)))) ++end;
      if (end + 2 >= size) end = size;
      add(data + body, end - body);
      start = end;
    }
  } else {   // what the plug-in really leaves there: the parameter sets one after the other, each behind a length field of the stream's NAL length size
    const size_t field = (nalSize == 1 || nalSize == 2 || nalSize == 4) ? nalSize : 4;
    size_t at = 0;
    while (at + field <= size) {
      size_t length = 0;
      for (size_t k = 0; k < field; ++k) length = length << 8 | data[at + k];
      at += field;
      if (!length || at + length > size) break;
      add(data + at, length);
      at += length;
    }
  }
  *outLength = n;
  return units;
}

// The access unit as the parser wants it: Annex B, and ended by an access unit delimiter. The parser (in the mode the core uses for streams of unknown
// framing) only knows a picture is complete when the next NAL unit of another picture starts; the delimiter at the end makes it hand the picture over in
// this call, which is what the plug-in's one-completion-per-call protocol needs. `nals` gets the number of NAL units, `types` the first few types.
// `params` (when `paramsLength` is not 0) go in front: the parameter sets, which the plug-in gave at CreateDecoder, never again.
// Returns the number of bytes in `out`, 0 when the data cannot be read as the codec's stream.
inline size_t frameAccessUnit(int codec, const uint8_t *data, size_t size, uint32_t nalSize, const uint8_t *params, size_t paramsLength, uint8_t *out, size_t outSize,
                              unsigned *nals, uint8_t types[8], bool *annexB) {
  *nals = 0;
  if (size + 16 + paramsLength > outSize) return 0;
  size_t n = 0;
  if (paramsLength) {
    memcpy(out, params, paramsLength);
    n = paramsLength;
  }
  const bool annex = startsAnnexB(data, size, nalSize);
  *annexB = annex;
  if (annex) {
    memcpy(out + n, data, size);
    n += size;
    for (size_t i = 0; i + 3 < size; ++i)
      if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 && i + 3 < size) {
        if (*nals < 8) types[*nals] = static_cast<uint8_t>(nalType(codec, data[i + 3]));
        ++*nals;
      }
  } else {
    // length-prefixed (AVCC / HVCC): the length fields are as long as CreateDecoder said
    const size_t field = (nalSize == 1 || nalSize == 2 || nalSize == 4) ? nalSize : 4;
    size_t at = 0;
    while (at + field <= size) {
      uint32_t length = 0;
      for (size_t k = 0; k < field; ++k) length = length << 8 | data[at + k];
      if (!length || at + field + length > size || n + 4 + length + 8 > outSize) return 0;
      out[n] = 0, out[n + 1] = 0, out[n + 2] = 0, out[n + 3] = 1;
      memcpy(out + n + 4, data + at + field, length);
      if (*nals < 8) types[*nals] = static_cast<uint8_t>(nalType(codec, data[at + field]));
      ++*nals;
      n += 4 + length;
      at += field + length;
    }
    if (at != size) return 0;
  }
  static const uint8_t audH264[] = {0, 0, 0, 1, 0x09, 0xf0};
  static const uint8_t audH265[] = {0, 0, 0, 1, 0x46, 0x01, 0x50};   // nal_unit_type 35, temporal id plus one 1; pic_type 2 (I, P and B slices), the stop bit
  if (codec == kVdecH265) {
    memcpy(out + n, audH265, sizeof audH265);
    return n + sizeof audH265;
  }
  memcpy(out + n, audH264, sizeof audH264);
  return n + sizeof audH264;
}

}  // namespace d5500

#endif
