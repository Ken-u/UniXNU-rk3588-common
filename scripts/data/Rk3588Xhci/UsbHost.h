// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// UsbHost.h -- the USB layer above XhciCore.h: root port handling, device enumeration, hubs and the HID class driver.
//
// Host::poll() is the whole life of it, called from one thread at about 1 kHz: it lets the controller handle its events, notices connects and
// disconnects on the root ports (PORTSC) and on hub ports (the hubs' status change endpoints), enumerates what arrived and removes what left.
// Enumeration is synchronous and sequential: reset the port, Enable Slot, Address Device, read the device and configuration descriptors, then bind a
// class driver: a hub (SET_CONFIGURATION, hub descriptor, Configure Endpoint with the hub fields of the slot context, power the ports, watch the
// status change endpoint) or HID boot keyboards and mice (the reports go to the HidSink, which is the board's input path).
//
// The mode is the boot argument rk3588-xhci: 3 enumerate and read descriptors only; 4 also hubs; 5 also HID. Each level does what the one below does.
#pragma once
#include "HidReport.h"
#include "UsbDesc.h"
#include "XhciCore.h"

#ifndef XHCI_LOG
#define XHCI_LOG(...) ((void)0)
#endif
#define UH_LOG(fmt, ...) XHCI_LOG("rk3588-xhci: %s: " fmt, name_, ##__VA_ARGS__)

namespace usb {

constexpr unsigned kMaxDevices = 12;
constexpr unsigned kMaxHubPorts = 15;
constexpr unsigned kHidBuffers = 3;      // transfer TRBs a HID endpoint keeps queued
constexpr unsigned kHidBufferBytes = 64;

enum Mode { kModeEnumerate = 3, kModeHubs = 4, kModeHid = 5 };

// Where the decoded input goes. Either pointer may be null (the reports of that kind are then dropped).
struct HidSink {
  void (*keyboard)(const uint8_t *report, unsigned length);                  // a boot keyboard report: modifiers, reserved, six keys (8 bytes)
  void (*mouse)(int32_t buttons, int32_t dx, int32_t dy, int32_t wheel);     // one mouse report
  void (*mouseProtocol)(bool report);                                        // the protocol the mouse was put in (report: its descriptor's layout)
};

struct Device;
class Host;

struct HidIface {
  bool active, isMouse, reportProtocol, needRecover;
  uint8_t iface, dci;
  uint16_t maxPacket;
  usbhid::MouseLayout layout;
  uint8_t *buffer;
  uint64_t bufferBus;
  unsigned done;                                // the buffer whose report arrives next
  Host *host;
  Device *dev;
};

struct Device {
  bool used;
  uint8_t slot, speed, rootPort, depth;         // depth: hubs between the device and the root hub
  uint32_t route;
  Device *parent;
  uint8_t parentPort;
  uint8_t ttSlot, ttPort;                       // the transaction translator of a low / full speed device (the slot of the high speed hub, its port)
  bool ttMulti;
  uint16_t vid, pid;
  uint8_t cls, sub, proto, mps0, address;
  Host *host;
  bool isHub, hubSuper, hubNeedRecover;
  uint8_t hubPorts, hubDci;
  uint8_t *hubBuffer;
  uint64_t hubBufferBus;
  uint32_t hubPending;                          // bit n: the hub reported a change on port n (bit 0: the hub itself)
  Device *child[kMaxHubPorts + 1];
  HidIface hid[2];                              // 0 keyboard, 1 mouse
};

class Host {
 public:
  void init(xhci::Controller *hc, const char *name, const HidSink *sink, unsigned mode);
  void poll();
  unsigned deviceCount() const;
  uint32_t statReports() const { return statReports_; }
  uint32_t statErrors() const { return statErrors_; }
  uint32_t statEnumerated() const { return statEnumerated_; }
  bool hidActive() const;                         // a HID endpoint is being polled (the caller then looks about every millisecond, else less often)
  const Device *rootDevice(unsigned port) const { return port < 32 ? rootDev_[port] : nullptr; }
  // called by the transfer event handlers (xhci::TransferHandler trampolines)
  void onHid(HidIface *h, const xhci::TransferEvent &e);
  void onHub(Device *hub, const xhci::TransferEvent &e);

 private:
  bool ctl(Device *d, uint8_t type, uint8_t request, uint16_t value, uint16_t index, void *data, uint16_t length, uint16_t *got = nullptr) {
    xhci::SetupPacket s = {type, request, value, index, length};
    return hc_->control(d->slot, s, data, got);
  }
  Device *allocDevice();
  void freeDevice(Device *d);
  void removeDevice(Device *d);
  void serviceRootPort(unsigned port);
  Device *enumerate(Device *parent, unsigned parentPort, unsigned rootPort, unsigned speed);
  bool setupHub(Device *d, const ConfigInfo &ci);
  bool setupHid(Device *d, const ConfigInfo &ci);
  void serviceHub(Device *hub);
  void serviceHubPort(Device *hub, unsigned port);
  bool hubPortStatus(Device *hub, unsigned port, uint16_t *status, uint16_t *change);
  bool hubResetPort(Device *hub, unsigned port, unsigned *speed);
  void queueHid(HidIface *h);

  xhci::Controller *hc_;
  const char *name_;
  const HidSink *sink_;
  unsigned mode_;
  Device devices_[kMaxDevices];
  Device *rootDev_[32];
  uint64_t nextTryUs_[32];
  uint64_t lastScanUs_;
  uint8_t cfgBuf_[1024];                        // the configuration descriptor being read (enumeration is sequential)
  uint32_t statReports_, statErrors_, statEnumerated_;
};


// ---------------------------------------------------------------------------------------------------------------------------------------------
// Implementation

inline void hidTrampoline(void *arg, const xhci::TransferEvent &e) {
  HidIface *h = static_cast<HidIface *>(arg);
  h->host->onHid(h, e);
}
inline void hubTrampoline(void *arg, const xhci::TransferEvent &e) {
  Device *d = static_cast<Device *>(arg);
  d->host->onHub(d, e);
}

inline const char *speedName(unsigned speed) {
  return speed == xhci::kSpeedLow ? "low" : speed == xhci::kSpeedFull ? "full" : speed == xhci::kSpeedHigh ? "high" : speed == xhci::kSpeedSuper ? "super" :
         speed == xhci::kSpeedSuperPlus ? "super+" : "unknown";
}

inline void Host::init(xhci::Controller *hc, const char *name, const HidSink *sink, unsigned mode) {
  hc_ = hc;
  name_ = name;
  sink_ = sink;
  mode_ = mode;
  for (unsigned i = 0; i < kMaxDevices; ++i) devices_[i] = Device();
  for (unsigned p = 0; p < 32; ++p) {
    rootDev_[p] = nullptr;
    nextTryUs_[p] = 0;
  }
  lastScanUs_ = 0;
  statReports_ = statErrors_ = statEnumerated_ = 0;
  for (unsigned p = 1; p <= hc_->portCount(); ++p) hc_->powerPort(p);
}

inline unsigned Host::deviceCount() const {
  unsigned n = 0;
  for (unsigned i = 0; i < kMaxDevices; ++i) n += devices_[i].used ? 1 : 0;
  return n;
}

inline bool Host::hidActive() const {
  for (unsigned i = 0; i < kMaxDevices; ++i)
    if (devices_[i].used && (devices_[i].hid[0].active || devices_[i].hid[1].active)) return true;
  return false;
}

inline Device *Host::allocDevice() {
  for (unsigned i = 0; i < kMaxDevices; ++i) {
    if (!devices_[i].used) {
      devices_[i] = Device();
      devices_[i].used = true;
      devices_[i].host = this;
      return &devices_[i];
    }
  }
  return nullptr;
}

inline void Host::freeDevice(Device *d) { *d = Device(); }

inline void Host::removeDevice(Device *d) {
  for (unsigned p = 0; p <= kMaxHubPorts; ++p) {
    if (d->child[p]) {
      Device *c = d->child[p];
      d->child[p] = nullptr;
      removeDevice(c);
    }
  }
  UH_LOG("slot %u (%04x:%04x) removed\n", d->slot, d->vid, d->pid);
  for (unsigned k = 0; k < 2; ++k) d->hid[k].active = false;
  d->isHub = false;
  if (d->slot) hc_->disableSlot(d->slot);
  if (d->parent && d->parentPort <= kMaxHubPorts && d->parent->child[d->parentPort] == d) d->parent->child[d->parentPort] = nullptr;
  freeDevice(d);
}

inline void Host::poll() {
  hc_->poll();
  const uint64_t now = hc_->nowUs();
  const uint32_t events = hc_->takePortEvents();
  const bool scan = now - lastScanUs_ >= 250000;
  if (scan) lastScanUs_ = now;
  for (unsigned p = 1; p <= hc_->portCount(); ++p)
    if (scan || (events >> p & 1)) serviceRootPort(p);
  for (unsigned i = 0; i < kMaxDevices; ++i) {
    Device *d = &devices_[i];
    if (!d->used) continue;
    if (d->isHub) serviceHub(d);
    if (!d->used) continue;
    for (unsigned k = 0; k < 2; ++k) {
      HidIface *h = &d->hid[k];
      if (h->active && h->needRecover) {
        h->needRecover = false;
        if (hc_->recoverEndpoint(d->slot, h->dci)) queueHid(h);
        else h->active = false;
      }
    }
  }
}

inline void Host::serviceRootPort(unsigned port) {
  uint32_t v = hc_->portStatus(port);
  if (v & xhci::kPortChange) hc_->clearPortChange(port, v);
  Device *d = rootDev_[port];
  const bool connected = (v & xhci::kPortCcs) != 0;
  if (d && (!connected || (v & xhci::kPortCsc))) {
    UH_LOG("port %u: %s\n", port, connected ? "reconnected" : "disconnected");
    removeDevice(d);
    rootDev_[port] = nullptr;
    d = nullptr;
    nextTryUs_[port] = 0;
  }
  if (d || !connected) return;
  const uint64_t now = hc_->nowUs();
  if (now < nextTryUs_[port]) return;
  hc_->delayUs(100000);   // the device has to stay connected for 100 ms before it is reset (USB 2.0 7.1.7.3)
  v = hc_->portStatus(port);
  if (!(v & xhci::kPortCcs)) return;
  uint32_t speed = 0;
  if (!hc_->resetPort(port, &speed)) {
    nextTryUs_[port] = hc_->nowUs() + 2000000;
    return;
  }
  UH_LOG("port %u (%s): %s speed device, portsc %#x\n", port, hc_->portIsUsb3(port) ? "usb3" : "usb2", speedName(speed), hc_->portStatus(port));
  d = enumerate(nullptr, 0, port, speed);
  if (d) rootDev_[port] = d;
  else nextTryUs_[port] = hc_->nowUs() + 2000000;
  hc_->clearPortChange(port, 0xffffffffu);   // what the reset and the enumeration stirred up is not a new connection
  hc_->takePortEvents();
}

inline Device *Host::enumerate(Device *parent, unsigned parentPort, unsigned rootPort, unsigned speed) {
  Device *d = allocDevice();
  if (!d) {
    UH_LOG("no free device entry\n");
    return nullptr;
  }
  d->parent = parent;
  d->parentPort = static_cast<uint8_t>(parentPort);
  d->rootPort = static_cast<uint8_t>(rootPort);
  d->speed = static_cast<uint8_t>(speed);
  d->depth = parent ? static_cast<uint8_t>(parent->depth + 1) : 0;
  d->route = parent ? (parent->route | static_cast<uint32_t>(parentPort) << (4 * parent->depth)) : 0;
  if (parent) {
    if (parent->ttSlot) {
      d->ttSlot = parent->ttSlot;
      d->ttPort = parent->ttPort;
      d->ttMulti = parent->ttMulti;
    } else if (speed != xhci::kSpeedHigh && parent->speed == xhci::kSpeedHigh) {
      d->ttSlot = parent->slot;
      d->ttPort = static_cast<uint8_t>(parentPort);
      d->ttMulti = parent->ttMulti;
    }
  }
  const int slot = hc_->enableSlot();
  if (slot < 0) {
    UH_LOG("Enable Slot failed\n");
    freeDevice(d);
    return nullptr;
  }
  d->slot = static_cast<uint8_t>(slot);
  d->mps0 = speed >= xhci::kSpeedSuper ? 9 : speed == xhci::kSpeedHigh ? 64 : 8;
  xhci::SlotParams params = {};
  params.speed = d->speed;
  params.route = d->route;
  params.rootPort = d->rootPort;
  params.parentSlot = d->ttSlot;
  params.parentPort = d->ttPort;
  params.mtt = d->ttMulti;
  params.mps0 = speed >= xhci::kSpeedSuper ? 512 : d->mps0;
  if (!hc_->addressDevice(d->slot, params)) {
    UH_LOG("Address Device failed (slot %d)\n", slot);
    hc_->disableSlot(d->slot);
    freeDevice(d);
    return nullptr;
  }
  d->address = static_cast<uint8_t>(hc_->deviceContext(d->slot)[3] & 0xff);
  uint8_t desc[18] = {};
  uint16_t got = 0;
  if (speed == xhci::kSpeedFull) {
    // the first 8 bytes tell the real maximum packet size of the default control endpoint (full speed devices may have 8, 16, 32 or 64)
    if (!ctl(d, 0x80, kReqGetDescriptor, kDescDevice << 8, 0, desc, 8, &got) || got < 8) goto fail;
    d->mps0 = desc[7];
    if (d->mps0 != 8 && (d->mps0 == 16 || d->mps0 == 32 || d->mps0 == 64) && !hc_->setEp0MaxPacket(d->slot, d->mps0)) goto fail;
  }
  if (!ctl(d, 0x80, kReqGetDescriptor, kDescDevice << 8, 0, desc, 18, &got) || got < 18) goto fail;
  d->vid = static_cast<uint16_t>(desc[8] | desc[9] << 8);
  d->pid = static_cast<uint16_t>(desc[10] | desc[11] << 8);
  d->cls = desc[4];
  d->sub = desc[5];
  d->proto = desc[6];
  ++statEnumerated_;
  UH_LOG("slot %u address %u: %04x:%04x class %u/%u/%u, USB %x.%02x, %u configuration(s)%s\n", d->slot, d->address, d->vid, d->pid, d->cls, d->sub, d->proto,
         desc[3], desc[2], desc[17], parent ? " (behind a hub)" : "");
  {
    if (!ctl(d, 0x80, kReqGetDescriptor, kDescConfig << 8, 0, cfgBuf_, 9, &got) || got < 9) goto fail;
    unsigned total = static_cast<unsigned>(cfgBuf_[2] | cfgBuf_[3] << 8);
    if (total > sizeof cfgBuf_) total = sizeof cfgBuf_;
    if (total > 9 && (!ctl(d, 0x80, kReqGetDescriptor, kDescConfig << 8, 0, cfgBuf_, static_cast<uint16_t>(total), &got) || got < 9)) goto fail;
    ConfigInfo ci;
    if (!parseConfig(cfgBuf_, got < total ? got : total, &ci)) goto fail;
    UH_LOG("slot %u: configuration %u: %u interface(s), %u endpoint(s)\n", d->slot, ci.value, ci.interfaceCount, ci.endpointCount);
    for (unsigned i = 0; i < ci.interfaceCount; ++i)
      UH_LOG("slot %u:   interface %u alt %u class %u/%u/%u, %u endpoint(s)\n", d->slot, ci.interfaces[i].number, ci.interfaces[i].alt, ci.interfaces[i].cls,
             ci.interfaces[i].sub, ci.interfaces[i].proto, ci.interfaces[i].endpoints);
    bool hubClass = d->cls == 9;
    for (unsigned i = 0; i < ci.interfaceCount && d->cls == 0; ++i) hubClass = hubClass || ci.interfaces[i].cls == 9;
    if (hubClass) {
      if (mode_ >= kModeHubs && !setupHub(d, ci)) UH_LOG("slot %u: hub setup failed\n", d->slot);
      if (mode_ < kModeHubs) UH_LOG("slot %u: a hub (not driven at this level)\n", d->slot);
    } else if (mode_ >= kModeHid) {
      if (!setupHid(d, ci)) UH_LOG("slot %u: no HID boot keyboard or mouse bound\n", d->slot);
    }
  }
  return d;
fail:
  UH_LOG("slot %u: enumeration failed\n", d->slot);
  hc_->disableSlot(d->slot);
  freeDevice(d);
  return nullptr;
}

// ---- hubs

inline bool Host::hubPortStatus(Device *hub, unsigned port, uint16_t *status, uint16_t *change) {
  uint8_t b[4] = {};
  uint16_t got = 0;
  if (!ctl(hub, 0xa3, kReqGetStatus, 0, static_cast<uint16_t>(port), b, 4, &got) || got < 4) return false;
  *status = static_cast<uint16_t>(b[0] | b[1] << 8);
  *change = static_cast<uint16_t>(b[2] | b[3] << 8);
  return true;
}

inline bool Host::setupHub(Device *d, const ConfigInfo &ci) {
  if (!ctl(d, 0x00, kReqSetConfiguration, ci.value, 0, nullptr, 0)) return false;
  uint8_t h[16] = {};
  uint16_t got = 0;
  const bool super = d->speed >= xhci::kSpeedSuper;
  if (!ctl(d, 0xa0, kReqGetDescriptor, static_cast<uint16_t>((super ? kDescSsHub : kDescHub) << 8), 0, h, super ? 12 : 9, &got) || got < 7) return false;
  unsigned ports = h[2];
  const unsigned characteristics = static_cast<unsigned>(h[3] | h[4] << 8);
  const unsigned powerGoodMs = h[5] * 2u;
  if (!ports) return false;
  if (ports > kMaxHubPorts) {
    UH_LOG("slot %u: hub with %u ports, only %u are used\n", d->slot, ports, kMaxHubPorts);
    ports = kMaxHubPorts;
  }
  const unsigned ttt = super ? 0 : (characteristics >> 5 & 3);
  bool mtt = false;
  if (d->speed == xhci::kSpeedHigh && d->proto == 2) mtt = ctl(d, 0x01, kReqSetInterface, 1, 0, nullptr, 0);   // alternate setting 1: one TT per port
  if (super && !ctl(d, 0x20, kReqSetHubDepth, d->depth, 0, nullptr, 0)) return false;
  const EndpointInfo *ep = nullptr;
  for (unsigned i = 0; i < ci.endpointCount; ++i)
    if ((ci.endpoints[i].attributes & 3) == 3 && (ci.endpoints[i].address & 0x80)) {
      ep = &ci.endpoints[i];
      break;
    }
  if (!ep) return false;
  const xhci::EpConfig c = makeEpConfig(d->speed, *ep);
  xhci::HubUpdate update = {true, static_cast<uint8_t>(ports), mtt, static_cast<uint8_t>(ttt)};
  if (!hc_->configureEndpoints(d->slot, &c, 1, &update)) return false;
  d->hubBuffer = hc_->allocBuffer(d->slot, 64, &d->hubBufferBus);
  if (!d->hubBuffer) return false;
  d->hubDci = c.dci;
  d->hubPorts = static_cast<uint8_t>(ports);
  d->hubSuper = super;
  d->ttMulti = d->ttMulti || mtt;
  if (!hc_->registerEndpoint(d->slot, d->hubDci, hubTrampoline, d)) return false;
  for (unsigned p = 1; p <= ports; ++p) ctl(d, 0x23, kReqSetFeature, kHubPortPower, static_cast<uint16_t>(p), nullptr, 0);
  hc_->delayUs((powerGoodMs + 100) * 1000);
  d->isHub = true;
  d->hubPending = ((1u << (ports + 1)) - 1) & ~1u;   // look at every port once: a device may be there already
  UH_LOG("slot %u: hub with %u ports%s%s, power good %u ms\n", d->slot, ports, super ? " (super speed)" : "", mtt ? " (one TT per port)" : "", powerGoodMs);
  const unsigned len = c.maxPacket < 8 ? 8 : c.maxPacket > 64 ? 64 : c.maxPacket;
  if (!hc_->queueTransfer(d->slot, d->hubDci, d->hubBufferBus, len)) return false;
  return true;
}

inline void Host::onHub(Device *hub, const xhci::TransferEvent &e) {
  if (!hub->isHub) return;
  if (e.code == xhci::kCcSuccess || e.code == xhci::kCcShortPacket) {
    uint32_t bits = hub->hubBuffer[0] | static_cast<uint32_t>(hub->hubBuffer[1]) << 8;
    hub->hubPending |= bits & ((1u << (hub->hubPorts + 1)) - 1);
    hc_->queueTransfer(hub->slot, hub->hubDci, hub->hubBufferBus, 8);
  } else {
    hub->hubNeedRecover = true;
  }
}

inline bool Host::hubResetPort(Device *hub, unsigned port, unsigned *speed) {
  if (!ctl(hub, 0x23, kReqSetFeature, kHubPortReset, static_cast<uint16_t>(port), nullptr, 0)) return false;
  uint16_t status = 0, change = 0;
  bool done = false;
  for (unsigned i = 0; i < 80; ++i) {
    hc_->delayUs(10000);
    if (!hubPortStatus(hub, port, &status, &change)) return false;
    if (change & 1u << 4) {
      done = true;
      break;
    }
  }
  ctl(hub, 0x23, kReqClearFeature, kHubCPortReset, static_cast<uint16_t>(port), nullptr, 0);
  hc_->delayUs(10000);
  if (!done || !hubPortStatus(hub, port, &status, &change) || !(status & 3)) return false;   // connected and enabled
  *speed = hub->hubSuper ? xhci::kSpeedSuper : status & 0x0200 ? xhci::kSpeedLow : status & 0x0400 ? xhci::kSpeedHigh : xhci::kSpeedFull;
  return true;
}

inline void Host::serviceHub(Device *hub) {
  if (hub->hubNeedRecover) {
    hub->hubNeedRecover = false;
    if (hc_->recoverEndpoint(hub->slot, hub->hubDci)) hc_->queueTransfer(hub->slot, hub->hubDci, hub->hubBufferBus, 8);
  }
  if (!hub->hubPending) return;
  const uint32_t pending = hub->hubPending;
  hub->hubPending = 0;
  if (pending & 1) {   // the hub itself: clear its local power / over current change bits
    uint8_t b[4] = {};
    uint16_t got = 0;
    if (ctl(hub, 0xa0, kReqGetStatus, 0, 0, b, 4, &got) && got >= 4) {
      if (b[2] & 1) ctl(hub, 0x20, kReqClearFeature, 0, 0, nullptr, 0);
      if (b[2] & 2) ctl(hub, 0x20, kReqClearFeature, 1, 0, nullptr, 0);
    }
  }
  for (unsigned p = 1; p <= hub->hubPorts && hub->used && hub->isHub; ++p)
    if (pending >> p & 1) serviceHubPort(hub, p);
}

inline void Host::serviceHubPort(Device *hub, unsigned port) {
  uint16_t status = 0, change = 0;
  if (!hubPortStatus(hub, port, &status, &change)) return;
  // the change bits of wPortChange and the feature that clears each: USB 2 hubs have connection, enable, suspend, over-current, reset (bits 0..4);
  // USB 3 hubs have connection (0), over-current (3), reset (4), warm reset (5), link state (6), configuration error (7)
  static const uint8_t kChange2[8] = {kHubCPortConnection, kHubCPortEnable, kHubCPortSuspend, kHubCPortOverCurrent, kHubCPortReset, 0, 0, 0};
  static const uint8_t kChange3[8] = {kHubCPortConnection, 0, 0, kHubCPortOverCurrent, kHubCPortReset, kHubCBhPortReset, kHubCPortLinkState, kHubCPortConfigError};
  const uint8_t *feature = hub->hubSuper ? kChange3 : kChange2;
  for (unsigned bit = 0; bit < 8; ++bit)
    if ((change >> bit & 1) && feature[bit]) ctl(hub, 0x23, kReqClearFeature, feature[bit], static_cast<uint16_t>(port), nullptr, 0);
  Device *child = hub->child[port];
  const bool connected = (status & 1) != 0;
  if (child && (!connected || (change & 1))) {
    UH_LOG("slot %u port %u: %s\n", hub->slot, port, connected ? "reconnected" : "disconnected");
    hub->child[port] = nullptr;
    removeDevice(child);
    child = nullptr;
  }
  if (child || !connected) return;
  hc_->delayUs(100000);   // debounce
  unsigned speed = 0;
  if (!hubResetPort(hub, port, &speed)) {
    UH_LOG("slot %u port %u: reset failed\n", hub->slot, port);
    return;
  }
  UH_LOG("slot %u port %u: %s speed device\n", hub->slot, port, speedName(speed));
  child = enumerate(hub, port, hub->rootPort, speed);
  if (child) hub->child[port] = child;
}

// ---- HID

inline void Host::queueHid(HidIface *h) {
  h->done = 0;
  for (unsigned i = 0; i < kHidBuffers; ++i)
    if (!hc_->queueTransfer(h->dev->slot, h->dci, h->bufferBus + kHidBufferBytes * i, h->maxPacket)) {
      UH_LOG("slot %u: cannot queue the HID transfer\n", h->dev->slot);
      h->active = false;
      return;
    }
}

inline bool Host::setupHid(Device *d, const ConfigInfo &ci) {
  int ifaceOf[2] = {-1, -1}, epOf[2] = {-1, -1};   // 0 keyboard, 1 mouse
  for (unsigned i = 0; i < ci.interfaceCount; ++i) {
    const InterfaceInfo &f = ci.interfaces[i];
    if (f.alt != 0 || f.cls != 3 || f.sub != 1 || (f.proto != 1 && f.proto != 2)) continue;
    const unsigned kind = f.proto == 2 ? 1 : 0;
    if (ifaceOf[kind] >= 0) continue;
    for (unsigned e = 0; e < ci.endpointCount; ++e)
      if (ci.endpoints[e].iface == i && (ci.endpoints[e].attributes & 3) == 3 && (ci.endpoints[e].address & 0x80)) {
        ifaceOf[kind] = static_cast<int>(i);
        epOf[kind] = static_cast<int>(e);
        break;
      }
  }
  if (ifaceOf[0] < 0 && ifaceOf[1] < 0) return false;
  UH_LOG("slot %u: HID boot keyboard %s, boot mouse %s\n", d->slot, ifaceOf[0] >= 0 ? "yes" : "no", ifaceOf[1] >= 0 ? "yes" : "no");
  if (!ctl(d, 0x00, kReqSetConfiguration, ci.value, 0, nullptr, 0)) return false;
  xhci::EpConfig cfgs[2];
  unsigned n = 0;
  for (unsigned kind = 0; kind < 2; ++kind) {
    if (ifaceOf[kind] < 0) continue;
    const InterfaceInfo &f = ci.interfaces[ifaceOf[kind]];
    const EndpointInfo &e = ci.endpoints[epOf[kind]];
    HidIface &h = d->hid[kind];
    h = HidIface();
    h.isMouse = kind == 1;
    h.iface = f.number;
    h.host = this;
    h.dev = d;
    cfgs[n] = makeEpConfig(d->speed, e);
    h.dci = cfgs[n].dci;
    h.maxPacket = cfgs[n].maxPacket ? (cfgs[n].maxPacket > kHidBufferBytes ? kHidBufferBytes : cfgs[n].maxPacket) : (kind ? 4 : 8);
    ++n;
    bool report = false;
    if (kind == 1 && f.hidReportLength) {
      // the report protocol, with the layout the report descriptor gives (the boot protocol has no wheel)
      uint8_t desc[xhci::kCtrlBuffer] = {};
      const unsigned want = f.hidReportLength > sizeof desc ? sizeof desc : f.hidReportLength;
      uint16_t got = 0;
      if (ctl(d, 0x81, kReqGetDescriptor, kDescHidReport << 8, f.number, desc, static_cast<uint16_t>(want), &got) && got == want &&
          usbhid::parseMouseLayout(desc, got, &h.layout) && h.layout.length <= h.maxPacket) {
        report = true;
        UH_LOG("slot %u: mouse report protocol: id %d, %u bytes, buttons@%d/%u x@%d/%u y@%d/%u wheel@%d/%u\n", d->slot, h.layout.hasId ? h.layout.reportId : -1,
               h.layout.length, h.layout.buttons.bit, h.layout.buttons.size, h.layout.x.bit, h.layout.x.size, h.layout.y.bit, h.layout.y.size, h.layout.wheel.bit,
               h.layout.wheel.size);
      } else {
        UH_LOG("slot %u: mouse report descriptor not understood, boot protocol\n", d->slot);
      }
    }
    h.reportProtocol = report;
    ctl(d, 0x21, kHidReqSetProtocol, report ? 1 : 0, f.number, nullptr, 0);   // a device that stalls these still works
    ctl(d, 0x21, kHidReqSetIdle, 0, f.number, nullptr, 0);
    if (kind == 1 && sink_ && sink_->mouseProtocol) sink_->mouseProtocol(report);
  }
  if (!hc_->configureEndpoints(d->slot, cfgs, n, nullptr)) return false;
  for (unsigned kind = 0; kind < 2; ++kind) {
    if (ifaceOf[kind] < 0) continue;
    HidIface &h = d->hid[kind];
    h.buffer = hc_->allocBuffer(d->slot, kHidBuffers * kHidBufferBytes, &h.bufferBus);
    if (!h.buffer || !hc_->registerEndpoint(d->slot, h.dci, hidTrampoline, &h)) return false;
    h.active = true;
    queueHid(&h);
  }
  UH_LOG("slot %u: HID polling started\n", d->slot);
  return true;
}

inline void Host::onHid(HidIface *h, const xhci::TransferEvent &e) {
  if (!h->active) return;
  if (e.code != xhci::kCcSuccess && e.code != xhci::kCcShortPacket) {
    if (statErrors_++ < 8) UH_LOG("slot %u: HID transfer failed, completion code %u\n", h->dev->slot, e.code);
    h->needRecover = true;
    return;
  }
  const unsigned idx = h->done;
  h->done = (h->done + 1) % kHidBuffers;
  const uint8_t *data = h->buffer + kHidBufferBytes * idx;
  const unsigned len = e.residual <= h->maxPacket ? h->maxPacket - e.residual : 0;
  ++statReports_;
  if (h->isMouse) {
    int32_t buttons = 0, dx = 0, dy = 0, wheel = 0;
    if (h->reportProtocol && h->layout.valid) {
      const usbhid::MouseLayout &l = h->layout;
      const unsigned skip = l.hasId ? 1 : 0;
      if (!(l.hasId && (len < 1 || data[0] != l.reportId)) && len > skip) {
        buttons = usbhid::fieldValue(data + skip, len - skip, l.buttons);
        dx = usbhid::fieldValue(data + skip, len - skip, l.x);
        dy = usbhid::fieldValue(data + skip, len - skip, l.y);
        wheel = usbhid::fieldValue(data + skip, len - skip, l.wheel);
        if (sink_ && sink_->mouse) sink_->mouse(buttons, dx, dy, wheel);
      }
    } else if (len >= 3) {
      buttons = data[0];
      dx = static_cast<int8_t>(data[1]);
      dy = static_cast<int8_t>(data[2]);
      wheel = len > 3 ? static_cast<int8_t>(data[3]) : 0;
      if (sink_ && sink_->mouse) sink_->mouse(buttons, dx, dy, wheel);
    }
  } else if (sink_ && sink_->keyboard) {
    uint8_t report[8] = {};
    for (unsigned i = 0; i < 8 && i < len; ++i) report[i] = data[i];
    sink_->keyboard(report, 8);
  }
  // the buffer is free again: queue it behind the others
  if (!hc_->queueTransfer(h->dev->slot, h->dci, h->bufferBus + kHidBufferBytes * idx, h->maxPacket)) h->needRecover = true;
}

}  // namespace usb
