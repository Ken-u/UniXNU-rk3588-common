// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// HidReport.h -- where the fields of a USB mouse's input report are, read from the device's HID report descriptor, and how to take them out of a
// report. A pure function of the descriptor, no kernel dependency: tests/test-xhci.cpp runs it.
// (A copy of the Orange Pi 5 repository's package/Opi5Usb/MouseReport.h, which the OHCI driver still uses; the two are the same code.)
//
// The boot protocol (SET_PROTOCOL 0) has buttons, X and Y in three bytes and nothing else; the wheel exists only in the report protocol, whose layout
// the device describes. The Orange Pi 5's NativeUsb.cpp used to run every mouse in boot protocol, and a wheel mouse then never sent a wheel count (board,
// 2026-11-06: Logitech 046d:c077, whose report is [buttons][X][Y][wheel]).
#pragma once

#include <stdint.h>

namespace usbhid {

// Where the fields of a mouse's report are, read from its HID report descriptor (parseMouseLayout). The boot protocol has buttons, X and Y in three bytes and
// nothing else: a wheel exists only in the report protocol, whose layout the device describes.
struct MouseField {
  int16_t bit = -1;                                  // first bit within the report's data (after the report ID byte); -1: not there
  uint8_t size = 0;                                  // bits (the buttons: how many, one bit each)
  bool sign = false;
};
struct MouseLayout {
  bool valid = false;
  bool hasId = false;
  uint8_t reportId = 0;
  uint8_t length = 0;                                // bytes of a whole report, the ID byte included
  MouseField buttons, x, y, wheel;
};

inline int32_t fieldValue(const uint8_t *data, unsigned length, const MouseField &f) {
  if (f.bit < 0) return 0;
  uint32_t v = 0;
  for (unsigned i = 0; i < f.size; ++i) {
    const unsigned b = static_cast<unsigned>(f.bit) + i;
    if ((b >> 3) < length && ((data[b >> 3] >> (b & 7)) & 1)) v |= 1u << i;
  }
  if (f.sign && f.size < 32 && (v & (1u << (f.size - 1)))) v |= ~0u << f.size;
  return static_cast<int32_t>(v);
}

inline int8_t clamp8(int32_t v) { return static_cast<int8_t>(v > 127 ? 127 : (v < -127 ? -127 : v)); }

// Walk a HID report descriptor and find where a relative pointer's fields are: Button 1..n (page 9, one bit each), X, Y and Wheel (Generic Desktop 0x30, 0x31,
// 0x38; Input, Variable, Relative) of the first report ID that has both X and Y. Only short items matter; Push/Pop are not followed (no mouse uses them).
inline bool parseMouseLayout(const uint8_t *d, unsigned len, MouseLayout *out) {
  MouseLayout l;
  uint32_t page = 0, size = 0, count = 0, id = 0, bits = 0, usages[16], umin = 0, umax = 0;
  unsigned nusage = 0;
  bool range = false, hasId = false;
  int32_t lmin = 0;
  unsigned i = 0;
  while (i < len) {
    const uint8_t prefix = d[i];
    if (prefix == 0xfe) {   // a long item (never used by a mouse): skip it
      if (i + 1 >= len) break;
      i += 3 + d[i + 1];
      continue;
    }
    unsigned bytes = prefix & 3;
    if (bytes == 3) bytes = 4;
    const unsigned type = (prefix >> 2) & 3, tag = prefix >> 4;
    if (i + 1 + bytes > len) break;
    uint32_t u = 0;
    for (unsigned k = 0; k < bytes; ++k) u |= static_cast<uint32_t>(d[i + 1 + k]) << (8 * k);
    const int32_t sv = bytes == 0 ? 0 : bytes == 1 ? static_cast<int8_t>(u) : bytes == 2 ? static_cast<int16_t>(u) : static_cast<int32_t>(u);
    i += 1 + bytes;
    if (type == 1) {   // global
      if (tag == 0) page = u;
      else if (tag == 1) lmin = sv;
      else if (tag == 7) size = u;
      else if (tag == 9) count = u;
      else if (tag == 8) {
        if (l.x.bit >= 0 && l.y.bit >= 0) break;   // the next report: the mouse report is complete
        l = MouseLayout();
        bits = 0;
        id = u;
        hasId = true;
      }
    } else if (type == 2) {   // local
      const uint32_t full = bytes == 4 ? u : ((page << 16) | u);
      if (tag == 0) {
        if (nusage < 16) usages[nusage++] = full;
      } else if (tag == 1) {
        umin = full;
        range = true;
      } else if (tag == 2) {
        umax = full;
      }
    } else if (type == 0) {   // main
      if (tag == 8 && size && size <= 32) {   // Input
        const bool constant = (u & 1) != 0, variable = (u & 2) != 0, relative = (u & 4) != 0;
        for (uint32_t k = 0; k < count && !constant; ++k) {
          uint32_t usage = 0;
          if (range) usage = umin + k > umax ? umax : umin + k;
          else if (nusage) usage = usages[k < nusage ? k : nusage - 1];
          const int16_t bit = static_cast<int16_t>(bits + k * size);
          MouseField *f = nullptr;
          if (variable && relative && usage == 0x00010030) f = &l.x;
          else if (variable && relative && usage == 0x00010031) f = &l.y;
          else if (variable && relative && usage == 0x00010038) f = &l.wheel;
          if (f && f->bit < 0 && size <= 16) {
            f->bit = bit;
            f->size = static_cast<uint8_t>(size);
            f->sign = lmin < 0;
          } else if (variable && size == 1 && usage >= 0x00090001 && usage <= 0x00090003) {   // the three buttons a mouse report carries
            if (l.buttons.bit < 0) l.buttons.bit = bit - static_cast<int16_t>(usage - 0x00090001);
            l.buttons.size = static_cast<uint8_t>(usage - 0x00090001 + 1);
          }
        }
        bits += count * size;
      }
      if (tag == 8 || tag == 9 || tag == 11 || tag == 10 || tag == 12) {   // every main item ends the local items
        nusage = 0;
        range = false;
      }
    }
  }
  if (l.x.bit < 0 || l.y.bit < 0) return false;
  l.hasId = hasId;
  l.reportId = static_cast<uint8_t>(id);
  l.length = static_cast<uint8_t>((bits + 7) / 8 + (hasId ? 1 : 0));
  l.valid = true;
  *out = l;
  return true;
}

}  // namespace usbhid
