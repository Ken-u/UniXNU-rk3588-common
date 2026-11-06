// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// UsbDesc.h -- USB descriptor constants and the walk over a configuration descriptor (USB 2.0 chapter 9, USB 3.x chapter 9, HID 1.11, hub class).
// Pure functions, no kernel include: UsbHost.h uses them, tests/test-xhci.cpp runs them.
#pragma once
#include <stdint.h>

#include "XhciCore.h"

namespace usb {

constexpr uint8_t kReqGetStatus = 0, kReqClearFeature = 1, kReqSetFeature = 3, kReqSetAddress = 5, kReqGetDescriptor = 6, kReqSetConfiguration = 9,
                  kReqSetInterface = 11, kReqSetHubDepth = 12;
constexpr uint8_t kHidReqSetIdle = 0x0a, kHidReqSetProtocol = 0x0b;
constexpr uint8_t kDescDevice = 1, kDescConfig = 2, kDescInterface = 4, kDescEndpoint = 5, kDescHid = 0x21, kDescHidReport = 0x22, kDescHub = 0x29,
                  kDescSsHub = 0x2a, kDescSsCompanion = 0x30;
// hub class features
constexpr uint8_t kHubPortConnection = 0, kHubPortEnable = 1, kHubPortReset = 4, kHubPortPower = 8, kHubCPortConnection = 16, kHubCPortEnable = 17,
                  kHubCPortSuspend = 18, kHubCPortOverCurrent = 19, kHubCPortReset = 20, kHubCPortLinkState = 25, kHubCPortConfigError = 26,
                  kHubCBhPortReset = 29;

constexpr unsigned kMaxInterfaces = 8, kMaxEndpoints = 16;

struct EndpointInfo {
  uint8_t iface;                // index into ConfigInfo::interfaces
  uint8_t address;              // bit 7 direction (1 = IN), 3:0 number
  uint8_t attributes;           // 1:0 transfer type (0 control, 1 isochronous, 2 bulk, 3 interrupt)
  uint16_t maxPacketRaw;        // wMaxPacketSize: 10:0 packet size, 12:11 additional transactions per microframe (high speed)
  uint8_t interval;
  bool companion;               // a SuperSpeed endpoint companion descriptor followed
  uint8_t maxBurst;
  uint16_t bytesPerInterval;
};

struct InterfaceInfo {
  uint8_t number, alt, cls, sub, proto;
  uint16_t hidReportLength;     // from the HID class descriptor (0: none)
  uint8_t firstEndpoint, endpoints;
};

struct ConfigInfo {
  uint8_t value;
  unsigned interfaceCount, endpointCount;
  InterfaceInfo interfaces[kMaxInterfaces];
  EndpointInfo endpoints[kMaxEndpoints];
};

// Walk the descriptors of a configuration. Interfaces beyond kMaxInterfaces and endpoints beyond kMaxEndpoints are ignored; a length of 0 or one that runs
// past the end stops the walk (what was read until then is kept). False when the data is not a configuration descriptor.
inline bool parseConfig(const uint8_t *d, unsigned length, ConfigInfo *out) {
  *out = ConfigInfo();
  if (length < 9 || d[0] < 9 || d[1] != kDescConfig) return false;
  out->value = d[5];
  int current = -1;
  unsigned off = d[0];
  while (off + 2 <= length) {
    const unsigned len = d[off], type = d[off + 1];
    if (len < 2 || off + len > length) break;
    if (type == kDescInterface && len >= 9) {
      if (out->interfaceCount < kMaxInterfaces) {
        InterfaceInfo &i = out->interfaces[out->interfaceCount];
        i.number = d[off + 2];
        i.alt = d[off + 3];
        i.cls = d[off + 5];
        i.sub = d[off + 6];
        i.proto = d[off + 7];
        i.hidReportLength = 0;
        i.firstEndpoint = static_cast<uint8_t>(out->endpointCount);
        i.endpoints = 0;
        current = static_cast<int>(out->interfaceCount++);
      } else {
        current = -1;
      }
    } else if (type == kDescHid && len >= 9 && current >= 0) {
      if (d[off + 6] == kDescHidReport) out->interfaces[current].hidReportLength = static_cast<uint16_t>(d[off + 7] | d[off + 8] << 8);
    } else if (type == kDescEndpoint && len >= 7 && current >= 0 && out->endpointCount < kMaxEndpoints) {
      EndpointInfo &e = out->endpoints[out->endpointCount++];
      e.iface = static_cast<uint8_t>(current);
      e.address = d[off + 2];
      e.attributes = d[off + 3];
      e.maxPacketRaw = static_cast<uint16_t>(d[off + 4] | d[off + 5] << 8);
      e.interval = d[off + 6];
      e.companion = false;
      e.maxBurst = 0;
      e.bytesPerInterval = 0;
      ++out->interfaces[current].endpoints;
    } else if (type == kDescSsCompanion && len >= 6 && out->endpointCount) {
      EndpointInfo &e = out->endpoints[out->endpointCount - 1];
      e.companion = true;
      e.maxBurst = d[off + 2];
      e.bytesPerInterval = static_cast<uint16_t>(d[off + 4] | d[off + 5] << 8);
    }
    off += len;
  }
  return true;
}

// xHCI endpoint context interval: an exponent of 125 microseconds. Low and full speed interrupt endpoints give bInterval in milliseconds (1..255): the
// largest power of two microframes (8 per millisecond) that does not exceed it, within 2^3..2^10 (the rule of the Linux driver). High and super speed
// give 2^(bInterval - 1) microframes already.
inline uint8_t endpointInterval(unsigned speed, uint8_t bInterval) {
  if (speed == xhci::kSpeedLow || speed == xhci::kSpeedFull) {
    unsigned v = bInterval ? bInterval : 1u, microframes = v * 8, exp = 0;
    while (microframes > 1) {
      microframes >>= 1;
      ++exp;
    }
    return static_cast<uint8_t>(exp < 3 ? 3 : exp > 10 ? 10 : exp);
  }
  const unsigned v = bInterval ? bInterval - 1 : 0;
  return static_cast<uint8_t>(v > 15 ? 15 : v);
}

// The endpoint context of a configured endpoint.
inline xhci::EpConfig makeEpConfig(unsigned speed, const EndpointInfo &e) {
  xhci::EpConfig c = {};
  const bool in = (e.address & 0x80) != 0;
  const unsigned type = e.attributes & 3;
  c.dci = static_cast<uint8_t>((e.address & 0xf) * 2 + (in ? 1 : 0));
  c.type = static_cast<uint8_t>(type == 3 ? (in ? xhci::kEpInterruptIn : xhci::kEpInterruptOut) : type == 2 ? (in ? xhci::kEpBulkIn : xhci::kEpBulkOut)
                                                              : type == 1 ? (in ? xhci::kEpIsochIn : xhci::kEpIsochOut) : xhci::kEpControl);
  c.maxPacket = static_cast<uint16_t>(e.maxPacketRaw & 0x7ff);
  const bool periodic = type == 1 || type == 3;
  if (speed >= xhci::kSpeedSuper) {
    c.maxBurst = e.companion ? e.maxBurst : 0;
  } else if (speed == xhci::kSpeedHigh && periodic) {
    c.maxBurst = static_cast<uint8_t>(e.maxPacketRaw >> 11 & 3);
  }
  if (periodic) {
    c.interval = endpointInterval(speed, e.interval);
    c.maxEsitPayload = speed >= xhci::kSpeedSuper && e.companion && e.bytesPerInterval ? e.bytesPerInterval : static_cast<uint32_t>(c.maxPacket) * (c.maxBurst + 1u);
    c.avgTrbLength = static_cast<uint16_t>(c.maxEsitPayload);
  } else {
    c.avgTrbLength = c.maxPacket;
  }
  return c;
}

}  // namespace usb
