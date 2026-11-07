// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// test-d5500.cpp -- host test of D5500Stream.h (what the AppleD5500 service does with the plug-in's bytes) and of the way the service drives the decoder core:
//
//   test-d5500 STREAM.h264|STREAM.h265 ...        (the extension says the codec)
//
// For each stream, as VideoToolbox's H264H8 plug-in would hand it over:
//   - the extradata, as an avcC / hvcC record (the shape CoreMedia makes of the parameter sets), as Annex B parameter sets and as parameter sets behind length
//     fields: detectCodec says the right codec, parseExtradata finds every parameter set (an SEI array of an hvcC is left out) and the stream's size, which
//     must be that of the pictures the core decodes;
//   - each access unit (the slices of one picture) as length prefixed NAL units (4 byte fields; 2 byte fields where every NAL unit is short) and as Annex B,
//     framed by frameAccessUnit (with the parameter sets in front of the first one, as the service does) and handed to the core ONE ACCESS UNIT PER CALL, the
//     last without the end-of-stream flag: every call must make exactly one decode job and give exactly one picture to the sink's `decoded` callback (the
//     service's protocol is one completion per call), and in the end every access unit has made a picture.
// There is no block here: it accepts every job and says "done" (tests/test-core.cpp is the test that compares the jobs with what MPP did on a board).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

#include "../D5500Stream.h"
#include "../VdecTrace.h"
extern "C" {
#include "../core/VdecCore.h"
#include "../core/VdecPlatform.h"
}

static int g_checks = 0, g_fail = 0;
#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_fail;                                                              \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                 \
    }                                                                        \
  } while (0)
#define CHECKF(cond, ...)                                                    \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_fail;                                                              \
      printf("FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond);                 \
      printf(__VA_ARGS__);                                                   \
      printf("\n");                                                          \
    }                                                                        \
  } while (0)

static long g_live = 0;
extern "C" void *vdc_calloc(size_t size) {
  ++g_live;
  return calloc(1, size);
}
extern "C" void vdc_free(void *p, size_t) {
  if (p) --g_live;
  free(p);
}
extern "C" void vdc_log(int level, const char *line) {
  if (level <= 2) printf("  [mpp %d] %s\n", level, line);
}

static std::vector<uint8_t> readFile(const std::string &path) {
  std::vector<uint8_t> v;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return v;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) v.insert(v.end(), buf, buf + n);
  fclose(f);
  return v;
}

struct Nal {
  const uint8_t *data;
  size_t size;
};

// The NAL units of an Annex B stream (trailing zero bytes belong to the next start code).
static std::vector<Nal> splitAnnexB(const std::vector<uint8_t> &d) {
  std::vector<Nal> nals;
  ssize_t start = -1;
  for (size_t i = 0; i + 3 <= d.size(); ++i) {
    if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1) {
      if (start >= 0) {
        size_t end = i;
        while (end > static_cast<size_t>(start) && d[end - 1] == 0) --end;
        nals.push_back({d.data() + start, end - start});
      }
      start = static_cast<ssize_t>(i + 3);
      i += 2;
    }
  }
  if (start >= 0 && static_cast<size_t>(start) < d.size()) nals.push_back({d.data() + start, d.size() - start});
  return nals;
}

static bool isSlice(int codec, const Nal &n) {
  const unsigned t = d5500::nalType(codec, n.data[0]);
  return codec == kVdecH265 ? (t <= 9 || (t >= 16 && t <= 21)) : (t == 1 || t == 5);
}

static bool startsPicture(int codec, const Nal &n) {
  return codec == kVdecH265 ? (n.size > 2 && (n.data[2] & 0x80)) : (n.size > 1 && (n.data[1] & 0x80));
}

static void be16(std::vector<uint8_t> &v, size_t x) {
  v.push_back(static_cast<uint8_t>(x >> 8));
  v.push_back(static_cast<uint8_t>(x));
}

// The records CoreMedia makes: avcC (SPS then PPS) and hvcC (VPS, SPS, PPS arrays; `withSei` adds a prefix SEI array, which real hvcC records have now and then).
static std::vector<uint8_t> makeRecord(int codec, const std::vector<Nal> &sets, bool withSei) {
  std::vector<uint8_t> r;
  if (codec == kVdecH264) {
    const Nal *sps = nullptr, *pps = nullptr;
    for (const Nal &n : sets) {
      if ((n.data[0] & 0x1f) == 7 && !sps) sps = &n;
      if ((n.data[0] & 0x1f) == 8 && !pps) pps = &n;
    }
    r = {1, sps->data[1], sps->data[2], sps->data[3], 0xff, 0xe1};
    be16(r, sps->size);
    r.insert(r.end(), sps->data, sps->data + sps->size);
    r.push_back(1);
    be16(r, pps->size);
    r.insert(r.end(), pps->data, pps->data + pps->size);
    return r;
  }
  const Nal *byType[3] = {};
  for (const Nal &n : sets) {
    const unsigned t = (n.data[0] >> 1) & 0x3f;
    if (t >= 32 && t <= 34 && !byType[t - 32]) byType[t - 32] = &n;
  }
  // header: version, profile byte, 4 compatibility bytes, 6 constraint bytes, level, then the reserved bits as the spec has them
  r = {1, 0x01, 0x60, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0xf0, 0x00, 0xfc, 0xfd, 0xf8, 0xf8, 0x00, 0x00, 0x0f};
  r.push_back(static_cast<uint8_t>(3 + (withSei ? 1 : 0)));
  for (int k = 0; k < 3; ++k) {
    r.push_back(static_cast<uint8_t>(0x80 | (32 + k)));
    be16(r, 1);
    be16(r, byType[k]->size);
    r.insert(r.end(), byType[k]->data, byType[k]->data + byType[k]->size);
  }
  if (withSei) {
    static const uint8_t sei[] = {0x4e, 0x01, 0x05, 0x02, 0xaa, 0xbb, 0x80};   // a prefix SEI (type 39) with a made-up payload
    r.push_back(0x80 | 39);
    be16(r, 1);
    be16(r, sizeof sei);
    r.insert(r.end(), sei, sei + sizeof sei);
  }
  return r;
}

static std::vector<uint8_t> annexBSets(const std::vector<Nal> &sets, int codec, bool longStart) {
  std::vector<uint8_t> v;
  for (const Nal &n : sets) {
    const unsigned t = d5500::nalType(codec, n.data[0]);
    if (codec == kVdecH265 ? !(t >= 32 && t <= 34) : !(t == 7 || t == 8)) continue;
    if (longStart) v.push_back(0);
    v.push_back(0), v.push_back(0), v.push_back(1);
    v.insert(v.end(), n.data, n.data + n.size);
  }
  return v;
}

static std::vector<uint8_t> prefixedSets(const std::vector<Nal> &sets, int codec, unsigned field) {
  std::vector<uint8_t> v;
  for (const Nal &n : sets) {
    const unsigned t = d5500::nalType(codec, n.data[0]);
    if (codec == kVdecH265 ? !(t >= 32 && t <= 34) : !(t == 7 || t == 8)) continue;
    for (int k = static_cast<int>(field) - 1; k >= 0; --k) v.push_back(static_cast<uint8_t>(n.size >> (8 * k)));
    v.insert(v.end(), n.data, n.data + n.size);
  }
  return v;
}

struct Block {
  static int run(void *, const uint32_t *, const VdecRun *, unsigned, uint32_t *status) {
    *status = vdec::kStaIrqRaw | vdec::kStaReady;
    return 0;
  }
};

struct Counter {
  unsigned decoded = 0, shown = 0;
  uint32_t width = 0, height = 0, cropLeft = 0, cropTop = 0;
  static void onDecoded(void *ctx, const VdecPicture *p) {
    Counter *c = static_cast<Counter *>(ctx);
    ++c->decoded;
    c->decodedPts.push_back(p->pts);
    c->width = p->width, c->height = p->height, c->cropLeft = p->cropLeft, c->cropTop = p->cropTop;
  }
  std::vector<long long> shownPts, decodedPts;
  static void onPicture(void *ctx, const VdecPicture *p) {
    ++static_cast<Counter *>(ctx)->shown;
    static_cast<Counter *>(ctx)->shownPts.push_back(p->pts);
  }
};

// An independent reading of a VPS (H.265 7.3.2.1) for the number it has next to the SPS's: vps_max_dec_pic_buffering_minus1 of the first sub-layer. Only for streams with
// one temporal layer, where profile_tier_level() is the fixed 12 bytes (the encoder writes the same value into the VPS and the SPS).
static bool vpsDpbMinus1(const Nal &vps, unsigned *out) {
  std::vector<uint8_t> rbsp;
  int zeros = 0;
  for (size_t i = 2; i < vps.size; ++i) {
    if (zeros >= 2 && vps.data[i] == 3) {
      zeros = 0;
      continue;
    }
    zeros = vps.data[i] == 0 ? zeros + 1 : 0;
    rbsp.push_back(vps.data[i]);
  }
  if (rbsp.size() < 17 || ((rbsp[1] >> 1) & 7) != 0) return false;
  size_t bit = 16 * 8;   // 4 + 1 + 1 + 6 + 3 + 1 + 16 bits, then profile_tier_level of 96 bits
  bool bad = false;
  auto get = [&](void) {
    if ((bit >> 3) >= rbsp.size()) {
      bad = true;
      return 1u;
    }
    const unsigned b = (rbsp[bit >> 3] >> (7 - (bit & 7))) & 1;
    ++bit;
    return b;
  };
  get();   // vps_sub_layer_ordering_info_present_flag
  unsigned zerosInCode = 0;
  while (!get() && !bad) ++zerosInCode;
  unsigned v = 1;
  for (unsigned i = 0; i < zerosInCode && !bad; ++i) v = (v << 1) | get();
  if (bad || zerosInCode > 16) return false;
  *out = v - 1;
  return true;
}

static void testStream(const std::string &path) {
  const int codec = path.size() > 5 && path.compare(path.size() - 5, 5, ".h265") == 0 ? kVdecH265 : kVdecH264;
  printf("%s (%s)\n", path.c_str(), codec == kVdecH265 ? "H.265" : "H.264");
  const std::vector<uint8_t> stream = readFile(path);
  CHECK(!stream.empty());
  if (stream.empty()) return;
  const std::vector<Nal> nals = splitAnnexB(stream);
  std::vector<Nal> sets;
  std::vector<std::vector<Nal>> units;   // the slices of each picture
  for (const Nal &n : nals) {
    const unsigned t = d5500::nalType(codec, n.data[0]);
    if (codec == kVdecH265 ? (t >= 32 && t <= 34) : (t == 7 || t == 8)) {
      // streams that repeat their parameter sets at every IDR picture give the extradata one copy of each
      bool seen = false;
      for (const Nal &o : sets) seen = seen || (o.size == n.size && !memcmp(o.data, n.data, n.size));
      if (!seen) sets.push_back(n);
    } else if (isSlice(codec, n)) {
      if (units.empty() || startsPicture(codec, n)) units.emplace_back();
      units.back().push_back(n);
    }
  }
  CHECK(!units.empty() && sets.size() >= (codec == kVdecH265 ? 3u : 2u));
  if (units.empty()) return;

  // ---- the core, to know the stream's real size and to take the access units
  std::vector<uint8_t> window(64u << 20, 0);
  VdecHw hw = {nullptr, window.data(), 0x65000000, static_cast<uint32_t>(window.size()), Block::run};

  struct Variant {
    const char *name;
    int kind;   // 0: record, 1: Annex B 4-byte start codes, 2: Annex B 3-byte, 3: 4-byte length fields, 4: 2-byte length fields
  };
  d5500::StreamInfo expected = {};
  bool haveExpected = false;
  for (const Variant &v : {Variant{"record", 0}, Variant{"Annex B, 4-byte start codes", 1}, Variant{"Annex B, 3-byte start codes", 2}, Variant{"4-byte length fields", 3},
                           Variant{"2-byte length fields", 4}}) {
    uint32_t nalSize = v.kind == 4 ? 2 : 4;
    std::vector<uint8_t> extra;
    switch (v.kind) {
      case 0: extra = makeRecord(codec, sets, codec == kVdecH265); break;
      case 1: extra = annexBSets(sets, codec, true); break;
      case 2: extra = annexBSets(sets, codec, false); break;
      case 3: extra = prefixedSets(sets, codec, 4); break;
      default: extra = prefixedSets(sets, codec, 2); break;
    }
    // the plug-in writes the extradata at offset 0 of its ring buffer; what follows it is whatever was there
    std::vector<uint8_t> ring(extra);
    ring.resize(extra.size() + 64, 0xa5);
    CHECKF(d5500::detectCodec(ring.data(), extra.size(), nalSize) == codec, "%s: codec detected as %d", v.name, d5500::detectCodec(ring.data(), extra.size(), nalSize));
    uint8_t params[1024];
    size_t paramsLength = 0;
    d5500::StreamInfo info = {};
    const unsigned units3 = d5500::parseExtradata(codec, ring.data(), extra.size(), nalSize, params, sizeof params, &paramsLength, &info);
    CHECKF(units3 == (codec == kVdecH265 ? 3u : sets.size()), "%s: %u parameter sets found", v.name, units3);
    CHECKF(info.valid, "%s: the SPS did not parse", v.name);
    size_t want = 0;
    for (const Nal &n : sets) {
      const unsigned t = d5500::nalType(codec, n.data[0]);
      if (codec == kVdecH265 ? (t >= 32 && t <= 34) : (t == 7 || t == 8)) want += 4 + n.size;
    }
    CHECKF(paramsLength == want, "%s: %zu bytes of parameter sets, %zu wanted", v.name, paramsLength, want);
    CHECK(paramsLength >= 8 && params[0] == 0 && params[1] == 0 && params[2] == 0 && params[3] == 1);
    if (getenv("D5500_DEBUG") && v.kind == 0)
      printf("  SPS: profile %u level %u %ux%u blocks, chroma %u, depth %u/%u, refs %u\n", info.profile, info.level, info.widthMbs, info.heightMbs, info.chromaFormat, 8 + info.lumaMinus8,
             8 + info.chromaMinus8, info.numRefFrames);
    if (codec == kVdecH265 && v.kind == 0) {
      unsigned vpsValue = 0;
      bool read = false;
      for (const Nal &n : sets)
        if (d5500::nalType(codec, n.data[0]) == 32 && vpsDpbMinus1(n, &vpsValue)) {
          read = true;
          CHECKF(info.numRefFrames == vpsValue, "SPS says %u pictures, the VPS %u", info.numRefFrames, vpsValue);
        }
      // (the stream with two temporal layers has a longer profile_tier_level: not read)
      CHECKF(read || path.find("_hevc_b.") != std::string::npos, "%s: the VPS could not be read", path.c_str());
    }
    if (codec == kVdecH264 && v.kind == 0) {
      // profile_idc and level_idc are bytes 1 and 3 of the SPS NAL unit
      for (const Nal &n : sets)
        if ((n.data[0] & 0x1f) == 7) CHECKF(info.profile == n.data[1] && info.level == n.data[3], "SPS profile %u level %u, bytes say %u and %u", info.profile, info.level, n.data[1], n.data[3]);
    }
    if (!haveExpected) {
      expected = info;
      haveExpected = true;
    } else {
      CHECKF(expected.widthMbs == info.widthMbs && expected.heightMbs == info.heightMbs && expected.numRefFrames == info.numRefFrames &&
                 expected.chromaFormat == info.chromaFormat && expected.lumaMinus8 == info.lumaMinus8 && expected.profile == info.profile && expected.level == info.level,
             "%s: another reading of the same SPS", v.name);
    }

    // ---- each access unit as its own call
    for (int framing = 0; framing < (v.kind == 0 ? 3 : 1); ++framing) {
      // framing 0: length prefixed with `nalSize`; 1: Annex B with 4-byte start codes; 2: Annex B with 3-byte start codes (the plug-in's own choice is the first)
      // (the Annex B variants of the extradata are about the extradata: their pictures go in length prefixed)
      std::vector<std::vector<uint8_t>> aus;
      bool tooLong = false;
      const uint32_t useSize = framing == 0 ? (v.kind == 4 ? 2 : 4) : 4;
      for (const auto &u : units) {
        std::vector<uint8_t> au;
        for (const Nal &n : u) {
          if (framing == 0) {
            if (useSize == 2 && n.size > 0xffff) tooLong = true;
            for (int k = static_cast<int>(useSize) - 1; k >= 0; --k) au.push_back(static_cast<uint8_t>(n.size >> (8 * k)));
          } else {
            if (framing == 1) au.push_back(0);
            au.push_back(0), au.push_back(0), au.push_back(1);
          }
          au.insert(au.end(), n.data, n.data + n.size);
        }
        aus.push_back(au);
      }
      if (tooLong) continue;
      VdecCore *core = nullptr;
      CHECK(vdec_core_create(&core, &hw, codec) == 0);
      if (!core) return;
      Counter counter;
      VdecSink sink = {&counter, Counter::onPicture, Counter::onDecoded};
      std::vector<uint8_t> scratch(4u << 20);
      unsigned wrongCalls = 0;
      bool needParams = true;
      for (size_t u = 0; u < aus.size(); ++u) {
        unsigned nalCount = 0;
        uint8_t types[8] = {};
        bool annexB = false;
        const size_t length = d5500::frameAccessUnit(codec, aus[u].data(), aus[u].size(), framing == 0 ? useSize : nalSize, needParams ? params : nullptr, needParams ? paramsLength : 0,
                                                     scratch.data(), scratch.size(), &nalCount, types, &annexB);
        CHECKF(length > 0, "%s: access unit %zu cannot be framed", v.name, u);
        if (!length) break;
        CHECKF(annexB == (framing != 0), "%s: access unit %zu is %s", v.name, u, annexB ? "Annex B" : "length prefixed");
        CHECKF(nalCount == units[u].size(), "%s: %u NAL units counted in access unit %zu, %zu in it", v.name, nalCount, u, units[u].size());
        const unsigned decodedBefore = counter.decoded;
        const int r = vdec_core_decode(core, scratch.data(), length, static_cast<int64_t>(u), 0, &sink);
        needParams = r < 0;
        if (r < 0 || counter.decoded != decodedBefore + 1) {
          ++wrongCalls;
          if (wrongCalls <= 3)
            printf("  %s: call %zu returned %d with %u decoded pictures (wanted exactly 1; types %u %u %u %u)\n", v.name, u, r, counter.decoded - decodedBefore, types[0], types[1],
                   types[2], types[3]);
        }
      }
      CHECKF(wrongCalls == 0, "%s (%s): %u of %zu calls did not give exactly one decoded picture", v.name, framing == 0 ? "length prefixed" : framing == 1 ? "Annex B 4" : "Annex B 3", wrongCalls,
             aus.size());
      CHECKF(counter.decoded == aus.size(), "%s: %u pictures decoded for %zu access units", v.name, counter.decoded, aus.size());
      // each picture carries the pts of the call that held its slices (an access unit delimiter at the end of the call before must not move it)
      for (size_t u = 0; u < counter.decodedPts.size(); ++u)
        if (counter.decodedPts[u] != static_cast<long long>(u)) {
          CHECKF(false, "%s: the picture of call %zu carries pts %lld", v.name, u, counter.decodedPts[u]);
          break;
        }
      // the end of the stream makes the held-back pictures come out in display order (the plug-in never does this; VideoToolbox puts frames in order itself)
      const int r = vdec_core_decode(core, nullptr, 0, 0, 1, &sink);
      CHECK(r >= 0);
      // H.265 gives back every picture. With H.264 and one access unit per call (each ended by an access unit delimiter) the last picture of a stream with B frames stays
      // in the parser's DPB (the display order list of D5500_DEBUG=1 shows it): the plug-in never ends a stream and takes the pictures of `decoded`, so it is only reported.
      if (codec == kVdecH265) CHECKF(counter.shown == aus.size(), "%s: %u pictures shown after the flush, %zu access units", v.name, counter.shown, aus.size());
      else if (counter.shown != aus.size() && v.kind == 0) printf("  note: %u of %zu pictures shown after the flush (display order is not used by the service)\n", counter.shown, aus.size());
      if (getenv("D5500_DEBUG")) {
        printf("  decoded pts:");
        for (long long x : counter.decodedPts) printf(" %lld", x);
        printf("\n  shown pts:  ");
        for (long long x : counter.shownPts) printf(" %lld", x);
        printf("\n");
      }
      CHECKF(counter.decoded == aus.size(), "%s: the flush decoded %u more pictures", v.name, counter.decoded - static_cast<unsigned>(aus.size()));
      // the size the answer to CreateDecoder is built from is the size of the pictures
      CHECKF((counter.width + counter.cropLeft + 15) / 16 == info.widthMbs && (counter.height + counter.cropTop + 15) / 16 == info.heightMbs,
             "%s: pictures %ux%u (crop %u,%u), the SPS says %ux%u blocks of 16", v.name, counter.width, counter.height, counter.cropLeft, counter.cropTop, info.widthMbs, info.heightMbs);
      vdec_core_destroy(core);
    }
  }
}

// A few parameter sets that must be refused or read in a particular way.
static void testEdges() {
  d5500::StreamInfo info = {};
  size_t length = 0;
  uint8_t out[256];
  // nothing in it
  const uint8_t zeros[64] = {};
  CHECK(d5500::parseExtradata(kVdecH265, zeros, sizeof zeros, 4, out, sizeof out, &length, &info) == 0 && length == 0);
  CHECK(d5500::parseExtradata(kVdecH264, zeros, sizeof zeros, 4, out, sizeof out, &length, &info) == 0 && length == 0);
  // a truncated hvcC record: the arrays are cut off in the middle of a NAL unit
  std::vector<uint8_t> hv = {1, 1, 0x60, 0, 0, 0, 0x90, 0, 0, 0, 0, 0, 90, 0xf0, 0, 0xfc, 0xfd, 0xf8, 0xf8, 0, 0, 3, 1, 0xa1, 0, 1, 0, 40, 0x42, 1, 1, 1};
  CHECK(d5500::looksLikeHvcC(hv.data(), hv.size()));
  CHECK(d5500::parseExtradata(kVdecH265, hv.data(), hv.size(), 4, out, sizeof out, &length, &info) == 0 && length == 0);
  // an SPS that ends early is not valid, and does not read past its end
  const uint8_t shortSps[] = {0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x5a};
  CHECK(!d5500::parseSpsHevc(shortSps, sizeof shortSps, &info));
  // an avcC is not taken for an hvcC and the other way round
  const uint8_t avcc[] = {1, 0x64, 0, 0x28, 0xff, 0xe1, 0, 4, 0x67, 0x64, 0, 0x28, 1, 0, 2, 0x68, 0xee};
  CHECK(d5500::looksLikeAvcC(avcc, sizeof avcc) && !d5500::looksLikeHvcC(avcc, sizeof avcc));
  CHECK(d5500::detectCodec(avcc, sizeof avcc, 4) == kVdecH264);
  CHECK(d5500::looksLikeHvcC(hv.data(), hv.size()) && !d5500::looksLikeAvcC(hv.data(), hv.size()));
  // a 433 byte slice behind a four byte field starts like an Annex B start code (00 00 01 ad): the length fields win
  std::vector<uint8_t> slice(4 + 433, 0x11);
  slice[0] = 0, slice[1] = 0, slice[2] = 1, slice[3] = 0xb1;
  slice[4] = 0x65;
  CHECK(d5500::tilesAsLengthPrefixed(slice.data(), slice.size(), 4) && !d5500::startsAnnexB(slice.data(), slice.size(), 4));
  uint8_t framed[1024];
  unsigned nals = 0;
  uint8_t types[8] = {};
  bool annexB = true;
  const size_t n264 = d5500::frameAccessUnit(kVdecH264, slice.data(), slice.size(), 4, nullptr, 0, framed, sizeof framed, &nals, types, &annexB);
  CHECK(n264 == 4 + 433 + 6 && !annexB && nals == 1 && types[0] == 5);
  CHECK(framed[0] == 0 && framed[1] == 0 && framed[2] == 0 && framed[3] == 1 && framed[4] == 0x65 && framed[n264 - 2] == 0x09 && framed[n264 - 1] == 0xf0);
  // H.265: the delimiter is type 35, temporal id plus one 1
  std::vector<uint8_t> h265slice = {0, 0, 0, 5, 0x26, 0x01, 0xaf, 0x08, 0x40};
  const size_t n265 = d5500::frameAccessUnit(kVdecH265, h265slice.data(), h265slice.size(), 4, nullptr, 0, framed, sizeof framed, &nals, types, &annexB);
  CHECK(n265 == 4 + 5 + 7 && nals == 1 && types[0] == 19);
  CHECK(framed[n265 - 7] == 0 && framed[n265 - 4] == 1 && framed[n265 - 3] == 0x46 && framed[n265 - 2] == 0x01 && framed[n265 - 1] == 0x50);
  // a length that runs past the data, a zero length and a field that does not tile are refused
  const uint8_t bad1[] = {0, 0, 0, 9, 0x65, 1, 2};
  CHECK(d5500::frameAccessUnit(kVdecH264, bad1, sizeof bad1, 4, nullptr, 0, framed, sizeof framed, &nals, types, &annexB) == 0);
  const uint8_t bad2[] = {0, 0, 0, 0, 0x65};
  CHECK(d5500::frameAccessUnit(kVdecH264, bad2, sizeof bad2, 4, nullptr, 0, framed, sizeof framed, &nals, types, &annexB) == 0);
  // the parameter sets go in front, once
  std::vector<uint8_t> sets = {0, 0, 0, 1, 0x40, 1, 0xaa, 0, 0, 0, 1, 0x42, 1, 0xbb};
  const size_t withSets = d5500::frameAccessUnit(kVdecH265, h265slice.data(), h265slice.size(), 4, sets.data(), sets.size(), framed, sizeof framed, &nals, types, &annexB);
  CHECK(withSets == sets.size() + n265 && memcmp(framed, sets.data(), sets.size()) == 0);
  // no room
  CHECK(d5500::frameAccessUnit(kVdecH265, h265slice.data(), h265slice.size(), 4, nullptr, 0, framed, 20, &nals, types, &annexB) == 0);
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  if (argc < 2) {
    printf("usage: test-d5500 STREAM ...\n");
    return 2;
  }
  testEdges();
  for (int i = 1; i < argc; ++i) testStream(argv[i]);
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
