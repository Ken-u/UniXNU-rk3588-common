// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// XhciModel.h -- a model of an xHCI host controller and of the USB devices behind its ports, for tests/test-xhci.cpp.
//
// It follows the specification where the driver depends on it and is strict about what the driver writes: a command ring or transfer ring TRB with
// the wrong cycle bit is not seen, a Link TRB is followed (and toggles the cycle state only with TC), contexts are checked against the topology
// (speed, route string, root port, transaction translator), a halted endpoint ignores its doorbell, Set TR Dequeue is refused unless the endpoint is
// stopped, a PORTSC write with PED set disables the port, the event ring is checked for overflow against ERDP. Time advances in delayUs().
#pragma once
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <map>
#include <set>
#include <vector>

#include "../UsbDesc.h"
#include "../XhciCore.h"

namespace model {

using xhci::Trb;

struct Request {
  uint8_t type, request;
  uint16_t value, index, length;
};

// A USB device (or hub) behind a port.
struct Dev {
  unsigned speed = xhci::kSpeedHigh;
  std::vector<uint8_t> device, config, reportDesc;
  std::map<uint16_t, std::vector<uint8_t>> strings;
  // the state the host puts it in
  int address = 0;
  int configuration = 0;
  int protocol = -1, idle = -1, altSetting = -1, hubDepth = -1;
  std::vector<Request> log;
  std::set<unsigned> stallRequests;                    // bRequest values that stall
  std::map<unsigned, unsigned> stallInterrupt;         // endpoint address -> number of times the endpoint halts before it works
  bool unresponsive = false;                           // never completes a control transfer (NAKs forever)
  // interrupt IN data by endpoint address
  std::map<unsigned, std::deque<std::vector<uint8_t>>> reports;
  // a hub
  struct HubPort {
    Dev *child = nullptr;
    uint16_t status = 0, change = 0;
    bool power = false, resetting = false;
  };
  bool isHub = false, superHub = false;
  std::vector<HubPort> ports;                          // index 0 is port 1
  uint16_t hubChange = 0;
  std::vector<uint8_t> hubDescriptor;
  // what the xHC knows of it
  int slot = 0;
  Dev *parentHub = nullptr;
  unsigned parentPort = 0;

  uint16_t statusChangeBitmap() const {
    uint16_t m = hubChange ? 1 : 0;
    for (size_t i = 0; i < ports.size(); ++i)
      if (ports[i].change) m |= 1u << (i + 1);
    return m;
  }
  void connectChild(unsigned port, Dev *d) {
    HubPort &p = ports[port - 1];
    p.child = d;
    d->parentHub = this;
    d->parentPort = port;
    p.status |= 1;
    p.change |= 1;
    if (superHub) p.status |= 0x0200;   // powered
  }
  void disconnectChild(unsigned port) {
    HubPort &p = ports[port - 1];
    if (p.child) p.child->parentHub = nullptr;
    p.child = nullptr;
    p.status &= ~3u;
    p.change |= 1;
  }
};

struct Endpoint {
  uint32_t state = 0;                                  // xhci::EpState
  uint64_t deq = 0;
  unsigned dcs = 1;
  uint16_t maxPacket = 0;
  uint32_t type = 0, interval = 0, avg = 0, esit = 0, burst = 0;
  uint64_t nextUs = 0;                                 // an interrupt endpoint is served once per interval
};

struct Slot {
  bool enabled = false;
  uint32_t state = 0;
  uint32_t dev0 = 0, dev1 = 0, dev2 = 0, dev3 = 0;     // the slot context as the xHC holds it
  Dev *dev = nullptr;
  unsigned address = 0;
  Endpoint ep[32];
};

struct Model {
  // ---- configuration
  unsigned nPorts = 2, maxSlots = 32, csz = 64, scratch = 1;
  uint64_t dmaBase = 0x64000000;
  std::vector<uint8_t> mem;
  // ---- registers
  uint32_t usbcmd = 0, usbsts = 1, config = 0;
  uint64_t dcbaap = 0, crcr = 0;
  uint32_t erstsz = 0, iman = 0;
  uint64_t erstba = 0, erdp = 0;
  // ---- state
  uint64_t cmdDeq = 0;
  unsigned cmdCcs = 1;
  uint64_t evBase = 0;
  unsigned evSize = 0, evEnq = 0, evCycle = 1;
  Slot slots[64];
  Dev *portDev[32] = {};
  uint32_t portsc[32] = {};                            // index = port number (1 based)
  bool usb3Port[32] = {};
  uint64_t timeUs = 0;
  // ---- counters and failures
  unsigned commands = 0, transferEvents = 0, resets = 0, portDisableWrites = 0, eventOverflow = 0, ringErrors = 0;
  unsigned commandFailures = 0;
  std::vector<std::string> problems;
  std::map<unsigned, unsigned> commandCount;
  unsigned failNextCommand = 0;                        // a completion code to return for the next command (0: none)
  bool hostSystemError = false;

  Model() {}
  void init() {
    mem.assign(1 << 20, 0);
    for (unsigned p = 1; p <= nPorts; ++p) {
      usb3Port[p] = p == nPorts;   // the last port is the USB 3 one (as on the RK3588: port 1 USB 2, port 2 USB 3)
      portsc[p] = 5u << xhci::kPortPlsShift | xhci::kPortPp;   // RxDetect, powered
    }
  }
  void problem(const char *fmt, unsigned a = 0, unsigned b = 0, unsigned c = 0) {
    char text[200];
    snprintf(text, sizeof text, fmt, a, b, c);
    problems.push_back(text);
    printf("MODEL PROBLEM: %s\n", text);
  }
  uint8_t *at(uint64_t bus, size_t bytes = 1) {
    if (bus < dmaBase || bus + bytes > dmaBase + mem.size()) {
      problem("access outside the DMA window: %#x (+%u)", static_cast<unsigned>(bus), static_cast<unsigned>(bytes));
      static uint8_t scratchArea[4096];
      return scratchArea;
    }
    return mem.data() + (bus - dmaBase);
  }
  uint32_t r32(uint64_t bus) { return *reinterpret_cast<uint32_t *>(at(bus, 4)); }
  void w32(uint64_t bus, uint32_t v) { *reinterpret_cast<uint32_t *>(at(bus, 4)) = v; }
  uint64_t r64(uint64_t bus) { return static_cast<uint64_t>(r32(bus + 4)) << 32 | r32(bus); }

  // ---- registers
  static constexpr uint32_t kOp = 0x20, kRt = 0x680, kDb = 0x700, kXcap = 0x880;
  uint32_t hcs1() const { return maxSlots | 1u << 8 | nPorts << 24; }
  uint32_t hcs2() const { return (scratch >> 5 & 0x1f) << 21 | (scratch & 0x1f) << 27 | 0xf1; }
  uint32_t hcc1() const { return (csz == 64 ? xhci::kHccCsz : 0u) | (kXcap / 4) << 16; }

  uint32_t read32(uint32_t off) {
    if (off == 0) return 0x01100000 | kOp;
    if (off == xhci::kHcsParams1) return hcs1();
    if (off == xhci::kHcsParams2) return hcs2();
    if (off == xhci::kHcsParams3) return 0x07ff000a;
    if (off == xhci::kHccParams1) return hcc1();
    if (off == xhci::kDbOff) return kDb;
    if (off == xhci::kRtsOff) return kRt;
    if (off == xhci::kHccParams2) return 0x2f;
    if (off == kXcap) return 0x02000802;                    // supported protocol, USB 2.00, next at +8 dwords
    if (off == kXcap + 4) return 0x20425355;
    if (off == kXcap + 8) return 1 | (nPorts - 1) << 8;     // ports 1 .. nPorts-1
    if (off == kXcap + 0x20) return 0x03000002;             // supported protocol, USB 3.00, last
    if (off == kXcap + 0x24) return 0x20425355;
    if (off == kXcap + 0x28) return nPorts | 1u << 8;
    if (off >= kOp && off < kOp + 0x400) {
      switch (off - kOp) {
        case xhci::kUsbCmd: return usbcmd;
        case xhci::kUsbSts: return usbsts;
        case xhci::kPageSize: return 1;
        case xhci::kConfig: return config;
        case xhci::kCrcr: return static_cast<uint32_t>(crcr & 8);
        case xhci::kCrcr + 4: return 0;
        case xhci::kDcbaap: return static_cast<uint32_t>(dcbaap);
        case xhci::kDcbaap + 4: return static_cast<uint32_t>(dcbaap >> 32);
        default: return 0;
      }
    }
    if (off >= kOp + xhci::kPortBase && off < kOp + xhci::kPortBase + 0x10 * nPorts) {
      const unsigned port = (off - kOp - xhci::kPortBase) / 0x10 + 1;
      if ((off - kOp - xhci::kPortBase) % 0x10 == 0 && port <= nPorts) return portsc[port];
      return 0;
    }
    if (off >= kRt + 0x20 && off < kRt + 0x40) {
      switch (off - kRt - 0x20) {
        case xhci::kIman: return iman;
        case xhci::kErstSz: return erstsz;
        case xhci::kErdp: return static_cast<uint32_t>(erdp);
        case xhci::kErdp + 4: return static_cast<uint32_t>(erdp >> 32);
        default: return 0;
      }
    }
    return 0;
  }

  void write32(uint32_t off, uint32_t v) {
    if (off >= kOp && off < kOp + 0x400) {
      const uint32_t o = off - kOp;
      if (o == xhci::kUsbCmd) {
        if (v & xhci::kCmdHcReset) {
          ++resets;
          hostReset();
          return;
        }
        usbcmd = v;
        if (v & xhci::kCmdRun) {
          if (usbsts & xhci::kStsHalted) checkBeforeRun();
          usbsts &= ~xhci::kStsHalted;
        } else {
          usbsts |= xhci::kStsHalted;
        }
      } else if (o == xhci::kUsbSts) {
        usbsts &= ~(v & (xhci::kStsEint | xhci::kStsHse | xhci::kStsPcd));
      } else if (o == xhci::kConfig) {
        config = v;
      } else if (o == xhci::kCrcr) {
        crcr = (crcr & ~0xffffffffull) | v;
      } else if (o == xhci::kCrcr + 4) {
        crcr = (crcr & 0xffffffffull) | static_cast<uint64_t>(v) << 32;
        cmdDeq = crcr & ~0x3full;
        cmdCcs = crcr & 1;
      } else if (o == xhci::kDcbaap) {
        dcbaap = (dcbaap & ~0xffffffffull) | v;
      } else if (o == xhci::kDcbaap + 4) {
        dcbaap = (dcbaap & 0xffffffffull) | static_cast<uint64_t>(v) << 32;
      }
      return;
    }
    if (off >= kOp + xhci::kPortBase && off < kOp + xhci::kPortBase + 0x10 * nPorts) {
      const unsigned port = (off - kOp - xhci::kPortBase) / 0x10 + 1;
      if ((off - kOp - xhci::kPortBase) % 0x10 == 0 && port <= nPorts) writePortsc(port, v);
      return;
    }
    if (off >= kRt + 0x20 && off < kRt + 0x40) {
      const uint32_t o = off - kRt - 0x20;
      if (o == xhci::kIman) {
        iman = (iman & ~xhci::kImanIe) | (v & xhci::kImanIe);
        if (v & xhci::kImanIp) iman &= ~xhci::kImanIp;
      } else if (o == xhci::kErstSz) {
        erstsz = v & 0xffff;
      } else if (o == xhci::kErstBa) {
        erstba = (erstba & ~0xffffffffull) | v;
      } else if (o == xhci::kErstBa + 4) {
        erstba = (erstba & 0xffffffffull) | static_cast<uint64_t>(v) << 32;
        loadEventRing();
      } else if (o == xhci::kErdp) {
        erdp = (erdp & ~0xffffffffull) | (v & ~8u);   // bit 3 (EHB) is write 1 to clear and not kept
      } else if (o == xhci::kErdp + 4) {
        erdp = (erdp & 0xffffffffull) | static_cast<uint64_t>(v) << 32;
      }
      return;
    }
    if (off >= kDb && off < kDb + 4 * 64) {
      doorbell((off - kDb) / 4, v & 0xff);
      return;
    }
  }

  void checkBeforeRun() {
    if (dcbaap & 0x3f || !dcbaap) problem("DCBAAP %#x is not 64 byte aligned", static_cast<unsigned>(dcbaap));
    if (!evSize || evSize < 16) problem("the event ring segment has %u TRBs", evSize);
    if (!(crcr & ~0x3full)) problem("the command ring pointer is not set", 0);
    if (scratch) {
      const uint64_t array = r64(dcbaap);
      if (!array || (array & 0x3f)) problem("scratchpad array pointer %#x", static_cast<unsigned>(array));
      else
        for (unsigned i = 0; i < scratch; ++i) {
          const uint64_t page = r64(array + 8ull * i);
          if (!page || (page & 0xfff)) problem("scratchpad page %u is at %#x", i, static_cast<unsigned>(page));
        }
    } else if (r64(dcbaap)) {
      problem("a scratchpad array is given for a controller that wants none", 0);
    }
    if (config == 0 || config > maxSlots) problem("MaxSlotsEn %u (controller has %u)", config, maxSlots);
    if (dcbaap + 8ull * (config + 1) > dmaBase + mem.size()) problem("the DCBAA does not fit", 0);
  }

  void hostReset() {
    usbcmd = 0;
    usbsts = xhci::kStsHalted;
    config = 0;
    dcbaap = crcr = erstba = erdp = 0;
    erstsz = iman = 0;
    evEnq = 0;
    evCycle = 1;
    for (auto &s : slots) s = Slot();
    for (unsigned p = 1; p <= nPorts; ++p) {
      if (portDev[p]) {
        portsc[p] = xhci::kPortCcs | xhci::kPortPp | (usb3Port[p] ? xhci::kPortPed : 0);   // connection state is kept; changes are lost
        portsc[p] |= portDev[p]->speed << xhci::kPortSpeedShift;
      }
    }
  }

  void loadEventRing() {
    // the one-entry segment table
    evBase = r64(erstba);
    evSize = r32(erstba + 8) & 0xffff;
    evEnq = 0;
    evCycle = 1;
    if (erdp != evBase) problem("ERDP %#x does not start at the segment %#x", static_cast<unsigned>(erdp), static_cast<unsigned>(evBase));
  }

  // ---- events
  void postEvent(uint32_t lo, uint32_t hi, uint32_t status, uint32_t control) {
    if (!evSize) return;
    const uint64_t deqIndex = ((erdp & ~0xfull) - evBase) / 16;
    if ((evEnq + 1) % evSize == deqIndex % evSize) {
      ++eventOverflow;
      problem("event ring overflow", 0);
      return;
    }
    const uint64_t at_ = evBase + 16ull * evEnq;
    w32(at_, lo);
    w32(at_ + 4, hi);
    w32(at_ + 8, status);
    w32(at_ + 12, (control & ~1u) | evCycle);
    if (++evEnq == evSize) {
      evEnq = 0;
      evCycle ^= 1;
    }
    usbsts |= xhci::kStsEint;
    iman |= xhci::kImanIp;
  }
  void postCommandCompletion(uint64_t trb, unsigned code, unsigned slotId, uint32_t param = 0) {
    postEvent(static_cast<uint32_t>(trb), static_cast<uint32_t>(trb >> 32), code << 24 | (param & 0xffffff),
              xhci::kEvCommandCompletion << xhci::kTrbTypeShift | slotId << 24);
  }
  void postTransfer(unsigned slot, unsigned dci, uint64_t trb, unsigned code, unsigned residual) {
    ++transferEvents;
    postEvent(static_cast<uint32_t>(trb), static_cast<uint32_t>(trb >> 32), code << 24 | (residual & 0xffffff),
              xhci::kEvTransfer << xhci::kTrbTypeShift | dci << 16 | slot << 24);
  }
  void postPortChange(unsigned port) {
    postEvent(port << 24, 0, 1u << 24, xhci::kEvPortStatusChange << xhci::kTrbTypeShift);
  }

  // ---- ports
  void attach(unsigned port, Dev *d) {
    portDev[port] = d;
    uint32_t v = portsc[port] & xhci::kPortPp;
    v |= xhci::kPortCcs | xhci::kPortCsc | d->speed << xhci::kPortSpeedShift;
    if (usb3Port[port]) v |= xhci::kPortPed | xhci::kPortPec;   // a SuperSpeed port is enabled by link training
    portsc[port] = v;
    postPortChange(port);
  }
  void detach(unsigned port) {
    portDev[port] = nullptr;
    portsc[port] = (portsc[port] & xhci::kPortPp) | xhci::kPortCsc | 5u << xhci::kPortPlsShift;
    for (auto &s : slots) {
      (void)s;
    }
    postPortChange(port);
  }
  bool portStuck[32] = {};                             // a reset that never completes
  unsigned warmResets = 0, hotResets = 0;
  void attachInactive(unsigned port, Dev *d) {          // a SuperSpeed device whose link did not train: Inactive, needs a warm reset
    portDev[port] = d;
    portsc[port] = (portsc[port] & xhci::kPortPp) | xhci::kPortCcs | xhci::kPortCsc | xhci::kPlsInactive << xhci::kPortPlsShift;
    inactive[port] = true;
    postPortChange(port);
  }
  bool inactive[32] = {};
  void writePortsc(unsigned port, uint32_t v) {
    uint32_t &p = portsc[port];
    if (v & xhci::kPortPed) {
      ++portDisableWrites;
      p &= ~xhci::kPortPed;
      problem("PORTSC write with PED set (disables port %u)", port);
    }
    p &= ~(v & xhci::kPortChange);   // write 1 to clear
    if (v & xhci::kPortPp) p |= xhci::kPortPp;
    if ((v & xhci::kPortWpr) && !usb3Port[port]) problem("a warm reset on USB 2 port %u", port);
    if ((v & xhci::kPortWpr) && (p & xhci::kPortCcs) && !portStuck[port]) {
      ++warmResets;
      inactive[port] = false;
      p |= xhci::kPortPed | xhci::kPortPrc | xhci::kPortWrc;
      p &= ~xhci::kPortPlsMask;
      p = (p & ~xhci::kPortSpeedMask) | portDev[port]->speed << xhci::kPortSpeedShift;
      postPortChange(port);
    } else if ((v & xhci::kPortPr) && (p & xhci::kPortCcs) && !portStuck[port] && !inactive[port]) {
      ++hotResets;
      p |= xhci::kPortPed | xhci::kPortPrc;
      p &= ~xhci::kPortPlsMask;
      p = (p & ~xhci::kPortSpeedMask) | portDev[port]->speed << xhci::kPortSpeedShift;
      postPortChange(port);
    }
  }

  // ---- slots, contexts
  uint32_t csize() const { return csz; }
  uint64_t deviceContextBus(unsigned slot) { return r64(dcbaap + 8ull * slot); }
  Dev *deviceAt(unsigned rootPort, uint32_t route, bool *ok) {
    *ok = false;
    if (rootPort < 1 || rootPort > nPorts || !portDev[rootPort]) return nullptr;
    Dev *d = portDev[rootPort];
    for (; route; route >>= 4) {
      const unsigned p = route & 0xf;
      if (!d->isHub || p < 1 || p > d->ports.size() || !d->ports[p - 1].child) return nullptr;
      d = d->ports[p - 1].child;
    }
    *ok = true;
    return d;
  }

  void doCommand(uint64_t trbBus, const Trb &t) {
    ++commands;
    const unsigned type = t.control >> xhci::kTrbTypeShift & 0x3f, slotId = t.control >> 24, ep = t.control >> 16 & 0x1f;
    ++commandCount[type];
    if (failNextCommand) {
      const unsigned code = failNextCommand;
      failNextCommand = 0;
      postCommandCompletion(trbBus, code, slotId);
      return;
    }
    const uint64_t param = static_cast<uint64_t>(t.hi) << 32 | t.lo;
    switch (type) {
      case xhci::kCmdEnableSlot: {
        for (unsigned s = 1; s <= (config & 0xff); ++s) {
          if (!slots[s].enabled) {
            slots[s] = Slot();
            slots[s].enabled = true;
            postCommandCompletion(trbBus, xhci::kCcSuccess, s);
            return;
          }
        }
        postCommandCompletion(trbBus, xhci::kCcNoSlots, 0);
        return;
      }
      case xhci::kCmdDisableSlot: {
        if (slotId < 1 || slotId > 63 || !slots[slotId].enabled) {
          postCommandCompletion(trbBus, xhci::kCcSlotNotEnabled, slotId);
          return;
        }
        if (slots[slotId].dev) slots[slotId].dev->slot = 0;
        slots[slotId] = Slot();
        postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
        return;
      }
      case xhci::kCmdAddressDevice: addressDevice(trbBus, slotId, param, t.control); return;
      case xhci::kCmdConfigureEndpoint: configureEndpoint(trbBus, slotId, param); return;
      case xhci::kCmdEvaluateContext: evaluateContext(trbBus, slotId, param); return;
      case xhci::kCmdStopEndpoint: {
        if (!slotOk(slotId) || ep < 1 || ep > 31 || slots[slotId].ep[ep].state == xhci::kEpDisabled) {
          postCommandCompletion(trbBus, xhci::kCcContextState, slotId);
          return;
        }
        slots[slotId].ep[ep].state = xhci::kEpStopped;
        syncEpState(slotId, ep);
        postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
        return;
      }
      case xhci::kCmdResetEndpoint: {
        if (!slotOk(slotId) || ep < 1 || ep > 31 || slots[slotId].ep[ep].state != xhci::kEpHalted) {
          postCommandCompletion(trbBus, xhci::kCcContextState, slotId);
          return;
        }
        slots[slotId].ep[ep].state = xhci::kEpStopped;
        syncEpState(slotId, ep);
        postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
        return;
      }
      case xhci::kCmdSetTrDequeue: {
        if (!slotOk(slotId) || ep < 1 || ep > 31 || slots[slotId].ep[ep].state != xhci::kEpStopped) {
          postCommandCompletion(trbBus, xhci::kCcContextState, slotId);
          return;
        }
        Endpoint &e = slots[slotId].ep[ep];
        e.deq = param & ~0xfull;
        e.dcs = param & 1;
        postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
        return;
      }
      case xhci::kCmdNoOp: postCommandCompletion(trbBus, xhci::kCcSuccess, 0); return;
      default:
        problem("unknown command type %u", type);
        postCommandCompletion(trbBus, xhci::kCcTrb, slotId);
    }
  }
  bool slotOk(unsigned s) { return s >= 1 && s < 64 && slots[s].enabled; }
  void syncEpState(unsigned slotId, unsigned ep) {
    const uint64_t out = deviceContextBus(slotId);
    if (out) w32(out + ep * csize(), (r32(out + ep * csize()) & ~7u) | slots[slotId].ep[ep].state);
  }

  void addressDevice(uint64_t trbBus, unsigned slotId, uint64_t inBus, uint32_t control) {
    if (!slotOk(slotId)) {
      postCommandCompletion(trbBus, xhci::kCcSlotNotEnabled, slotId);
      return;
    }
    Slot &s = slots[slotId];
    if (inBus & 0x3f) problem("input context at %#x is not 64 byte aligned", static_cast<unsigned>(inBus));
    const uint32_t drop = r32(inBus), add = r32(inBus + 4);
    if (drop != 0 || (add & 3) != 3) {
      problem("Address Device with drop %#x add %#x", drop, add);
      postCommandCompletion(trbBus, xhci::kCcParameter, slotId);
      return;
    }
    const uint64_t sc = inBus + csize(), e0 = inBus + 2 * csize();
    const uint32_t d0 = r32(sc), d1 = r32(sc + 4), d2 = r32(sc + 8);
    const unsigned speed = d0 >> 20 & 0xf, entries = d0 >> 27, rootPort = d1 >> 16 & 0xff, route = d0 & 0xfffff;
    bool ok = false;
    Dev *dev = deviceAt(rootPort, route, &ok);
    if (!ok || !dev) {
      problem("Address Device for an empty place: root port %u route %#x", rootPort, route);
      postCommandCompletion(trbBus, xhci::kCcParameter, slotId);
      return;
    }
    if (speed != dev->speed) {
      problem("Address Device slot speed %u, the device is %u", speed, dev->speed);
      postCommandCompletion(trbBus, xhci::kCcParameter, slotId);
      return;
    }
    if (entries != 1) problem("Address Device with %u context entries", entries);
    const unsigned type = r32(e0 + 4) >> 3 & 7, mps = r32(e0 + 4) >> 16;
    if (type != xhci::kEpControl || !mps) {
      problem("EP0 context: type %u max packet %u", type, mps);
      postCommandCompletion(trbBus, xhci::kCcParameter, slotId);
      return;
    }
    // a low or full speed device behind a high speed hub needs the transaction translator (parent hub slot and port) and nothing else does
    const bool needsTt = (speed == xhci::kSpeedLow || speed == xhci::kSpeedFull) && dev->parentHub && hubChainHasHighSpeed(dev);
    const unsigned ttSlot = d2 & 0xff, ttPort = d2 >> 8 & 0xff;
    if (needsTt) {
      Dev *tt = ttHub(dev, nullptr);
      if (!tt || tt->slot != static_cast<int>(ttSlot) || ttPort != ttPortOf(dev)) problem("transaction translator of the device: slot %u port %u", ttSlot, ttPort);
    } else if (ttSlot || ttPort) {
      problem("a TT is given (slot %u port %u) for a device that needs none", ttSlot, ttPort);
    }
    s.dev = dev;
    dev->slot = slotId;
    s.address = slotId;   // the xHC picks the address
    dev->address = static_cast<int>(s.address);
    s.dev0 = d0;
    s.dev1 = d1;
    s.dev2 = d2;
    s.state = xhci::kSlotAddressed;
    Endpoint &ep0 = s.ep[1];
    ep0.state = xhci::kEpRunning;
    ep0.maxPacket = static_cast<uint16_t>(mps);
    ep0.type = type;
    const uint64_t deqBits = static_cast<uint64_t>(r32(e0 + 12)) << 32 | r32(e0 + 8);
    ep0.deq = deqBits & ~0xfull;
    ep0.dcs = deqBits & 1;
    const uint64_t out = deviceContextBus(slotId);
    if (!out || (out & 0x3f)) problem("DCBAA[%u] is %#x", slotId, static_cast<unsigned>(out));
    // output context: slot with the state and address, EP0
    for (unsigned i = 0; i < 2 * csize() / 4; ++i) w32(out + 4ull * i, 0);
    w32(out, d0);
    w32(out + 4, d1);
    w32(out + 8, d2);
    w32(out + 12, s.address | xhci::kSlotAddressed << 27);
    for (unsigned i = 0; i < 5; ++i) w32(out + csize() + 4 * i, r32(e0 + 4 * i));
    w32(out + csize(), (r32(out + csize()) & ~7u) | xhci::kEpRunning);
    (void)control;
    postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
  }
  static bool hubChainHasHighSpeed(Dev *d) {
    for (Dev *h = d->parentHub; h; h = h->parentHub)
      if (h->speed == xhci::kSpeedHigh) return true;
    return false;
  }
  // the high speed hub that holds the TT of d: the closest high speed ancestor; its port towards d
  static Dev *ttHub(Dev *d, unsigned *port) {
    Dev *child = d;
    for (Dev *h = d->parentHub; h; child = h, h = h->parentHub)
      if (h->speed == xhci::kSpeedHigh) {
        if (port) *port = child->parentPort;
        return h;
      }
    return nullptr;
  }
  static unsigned ttPortOf(Dev *d) {
    unsigned port = 0;
    ttHub(d, &port);
    return port;
  }

  void configureEndpoint(uint64_t trbBus, unsigned slotId, uint64_t inBus) {
    if (!slotOk(slotId) || slots[slotId].state < xhci::kSlotAddressed) {
      postCommandCompletion(trbBus, xhci::kCcContextState, slotId);
      return;
    }
    Slot &s = slots[slotId];
    const uint32_t drop = r32(inBus), add = r32(inBus + 4);
    const uint64_t sc = inBus + csize();
    unsigned highest = 0;
    for (unsigned i = 2; i < 32; ++i)
      if (add >> i & 1) highest = i;
    if (drop) problem("Configure Endpoint with drop flags %#x", drop);
    if (add & 1) {
      const unsigned entries = r32(sc) >> 27;
      if (entries < highest) problem("Configure Endpoint: context entries %u below the highest added endpoint %u", entries, highest);
      const uint32_t d0 = r32(sc), d1 = r32(sc + 4), d2 = r32(sc + 8);
      if ((d0 & 0xfffff) != (s.dev0 & 0xfffff) || (d1 >> 16 & 0xff) != (s.dev1 >> 16 & 0xff)) problem("Configure Endpoint changed the route or the root port", 0);
      s.dev0 = d0;
      s.dev1 = d1;
      s.dev2 = d2;
    }
    const uint64_t out = deviceContextBus(slotId);
    for (unsigned dci = 2; dci < 32; ++dci) {
      if (!(add >> dci & 1)) continue;
      const uint64_t e = inBus + (1 + dci) * csize();
      Endpoint &ep = s.ep[dci];
      ep.type = r32(e + 4) >> 3 & 7;
      ep.maxPacket = static_cast<uint16_t>(r32(e + 4) >> 16);
      ep.interval = r32(e) >> 16 & 0xff;
      ep.burst = r32(e + 4) >> 8 & 0xff;
      ep.avg = r32(e + 16) & 0xffff;
      ep.esit = r32(e + 16) >> 16 | (r32(e) >> 24 & 0xff) << 16;
      const uint64_t deqBits = static_cast<uint64_t>(r32(e + 12)) << 32 | r32(e + 8);
      ep.deq = deqBits & ~0xfull;
      ep.dcs = deqBits & 1;
      ep.state = xhci::kEpRunning;
      for (unsigned i = 0; i < 5; ++i) w32(out + dci * csize() + 4 * i, r32(e + 4 * i));
      w32(out + dci * csize(), (r32(out + dci * csize()) & ~7u) | xhci::kEpRunning);
    }
    if (add & 1) {
      w32(out, s.dev0);
      w32(out + 4, s.dev1);
      w32(out + 8, s.dev2);
    }
    w32(out + 12, (r32(out + 12) & ~(0x1fu << 27)) | xhci::kSlotConfigured << 27);
    s.state = xhci::kSlotConfigured;
    postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
  }

  void evaluateContext(uint64_t trbBus, unsigned slotId, uint64_t inBus) {
    if (!slotOk(slotId)) {
      postCommandCompletion(trbBus, xhci::kCcSlotNotEnabled, slotId);
      return;
    }
    const uint32_t add = r32(inBus + 4);
    if (add & 2) {
      const uint64_t e0 = inBus + 2 * csize();
      slots[slotId].ep[1].maxPacket = static_cast<uint16_t>(r32(e0 + 4) >> 16);
      const uint64_t out = deviceContextBus(slotId);
      w32(out + csize() + 4, (r32(out + csize() + 4) & 0xffff) | static_cast<uint32_t>(slots[slotId].ep[1].maxPacket) << 16);
      ++evaluateCount;
    }
    postCommandCompletion(trbBus, xhci::kCcSuccess, slotId);
  }
  unsigned evaluateCount = 0;

  // ---- doorbells and rings
  void doorbell(unsigned target, unsigned value) {
    if (usbsts & xhci::kStsHalted) return;
    if (target == 0) {
      runCommandRing();
      return;
    }
    if (!slotOk(target) || value < 1 || value > 31) return;
    Endpoint &ep = slots[target].ep[value];
    if (ep.state == xhci::kEpHalted || ep.state == xhci::kEpDisabled) return;
    if (ep.state == xhci::kEpStopped) {
      ep.state = xhci::kEpRunning;
      syncEpState(target, value);
    }
    runEndpoint(target, value);
  }

  void runCommandRing() {
    for (unsigned guard = 0; guard < 256; ++guard) {
      const uint64_t b = cmdDeq;
      Trb t = {r32(b), r32(b + 4), r32(b + 8), r32(b + 12)};
      if ((t.control & 1) != cmdCcs) return;
      if ((t.control >> xhci::kTrbTypeShift & 0x3f) == xhci::kTrbLink) {
        cmdDeq = static_cast<uint64_t>(t.hi) << 32 | t.lo;
        if (t.control & xhci::kTrbToggleCycle) cmdCcs ^= 1;
        continue;
      }
      cmdDeq += 16;
      doCommand(b, t);
    }
  }

  struct Pending {
    uint64_t trb, buffer;
    uint32_t length;
    unsigned control;
  };

  // advance an endpoint: control TDs on EP0, Normal TRBs elsewhere
  void runEndpoint(unsigned slotId, unsigned dci) {
    Slot &s = slots[slotId];
    Endpoint &ep = s.ep[dci];
    for (unsigned guard = 0; guard < 1000; ++guard) {
      uint64_t b = ep.deq;
      Trb t = {r32(b), r32(b + 4), r32(b + 8), r32(b + 12)};
      if ((t.control & 1) != ep.dcs) return;
      const unsigned type = t.control >> xhci::kTrbTypeShift & 0x3f;
      if (type == xhci::kTrbLink) {
        ep.deq = static_cast<uint64_t>(t.hi) << 32 | t.lo;
        if (t.control & xhci::kTrbToggleCycle) ep.dcs ^= 1;
        continue;
      }
      if (dci == 1) {
        if (!runControlTd(slotId, ep)) return;
      } else {
        if (!runNormal(slotId, dci, ep, b, t)) return;
      }
    }
  }

  Trb readTrb(uint64_t b) { return Trb{r32(b), r32(b + 4), r32(b + 8), r32(b + 12)}; }
  // The n-th TRB (0: the first) from the dequeue pointer, following links, when all of them up to it are valid for the endpoint's cycle state.
  bool peek(Endpoint &ep, unsigned n, uint64_t *bus, Trb *out) {
    uint64_t b = ep.deq;
    unsigned dcs = ep.dcs, seen = 0;
    for (unsigned guard = 0; guard < 4096; ++guard) {
      const Trb t = readTrb(b);
      if ((t.control & 1) != dcs) return false;
      if ((t.control >> xhci::kTrbTypeShift & 0x3f) == xhci::kTrbLink) {
        b = static_cast<uint64_t>(t.hi) << 32 | t.lo;
        if (t.control & xhci::kTrbToggleCycle) dcs ^= 1;
        continue;
      }
      if (seen == n) {
        *bus = b;
        *out = t;
        return true;
      }
      ++seen;
      b += 16;
    }
    return false;
  }
  // Consume n TRBs (links on the way are followed).
  void advance(Endpoint &ep, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
      Trb t = readTrb(ep.deq);
      while ((t.control >> xhci::kTrbTypeShift & 0x3f) == xhci::kTrbLink) {
        ep.deq = static_cast<uint64_t>(t.hi) << 32 | t.lo;
        if (t.control & xhci::kTrbToggleCycle) ep.dcs ^= 1;
        t = readTrb(ep.deq);
      }
      ep.deq += 16;
    }
  }

  unsigned controlTds = 0, shortData = 0;
  bool runControlTd(unsigned slotId, Endpoint &ep) {
    uint64_t setupBus = 0, dataBus = 0, statusBus = 0;
    Trb setup, data = {}, status;
    if (!peek(ep, 0, &setupBus, &setup)) return false;
    if ((setup.control >> xhci::kTrbTypeShift & 0x3f) != xhci::kTrbSetupStage) {
      problem("a control TD does not start with a Setup Stage TRB (type %u)", setup.control >> xhci::kTrbTypeShift & 0x3f);
      ++ringErrors;
      return false;
    }
    if (!(setup.control & xhci::kTrbIdt) || setup.status != 8) problem("the Setup Stage TRB: IDT %u length %u", setup.control >> 6 & 1, setup.status);
    Request rq;
    rq.type = static_cast<uint8_t>(setup.lo);
    rq.request = static_cast<uint8_t>(setup.lo >> 8);
    rq.value = static_cast<uint16_t>(setup.lo >> 16);
    rq.index = static_cast<uint16_t>(setup.hi);
    rq.length = static_cast<uint16_t>(setup.hi >> 16);
    const unsigned trt = setup.control >> 16 & 3;
    const bool hasData = rq.length != 0, in = (rq.type & 0x80) != 0;
    if (trt != (hasData ? (in ? 3u : 2u) : 0u)) problem("transfer type %u of the Setup TRB for a request of %u bytes", trt, rq.length);
    unsigned n = 1;
    if (hasData) {
      if (!peek(ep, 1, &dataBus, &data)) return false;
      if ((data.control >> xhci::kTrbTypeShift & 0x3f) != xhci::kTrbDataStage) problem("Data Stage TRB type %u", data.control >> xhci::kTrbTypeShift & 0x3f);
      if (((data.control & xhci::kTrbDirIn) != 0) != in) problem("Data Stage direction %u for request type %#x", data.control >> 16 & 1, rq.type);
      if (data.status != rq.length) problem("Data Stage length %u for a request of %u", data.status, rq.length);
      n = 2;
    }
    if (!peek(ep, n, &statusBus, &status)) return false;
    if ((status.control >> xhci::kTrbTypeShift & 0x3f) != xhci::kTrbStatusStage) problem("Status Stage TRB type %u", status.control >> xhci::kTrbTypeShift & 0x3f);
    if (((status.control & xhci::kTrbDirIn) != 0) != (!(in && hasData))) problem("Status Stage direction %u", status.control >> 16 & 1);
    if (!(status.control & xhci::kTrbIoc)) problem("the Status Stage TRB has no IOC", 0);
    Dev *dev = s_dev(slotId);
    if (!dev) return false;
    ++controlTds;
    std::vector<uint8_t> payload;
    if (hasData && !in) {
      const uint8_t *src = at(static_cast<uint64_t>(data.hi) << 32 | data.lo, rq.length);
      payload.assign(src, src + rq.length);
    }
    std::vector<uint8_t> reply;
    const int result = deviceControl(dev, rq, payload, &reply);   // 0 ok, 1 stall, 2 no answer
    if (result == 2) return false;   // the TD stays queued: the device NAKs
    if (result == 1) {
      Endpoint &e = ep;
      e.state = xhci::kEpHalted;
      syncEpState(slotId, 1);
      postTransfer(slotId, 1, hasData ? dataBus : statusBus, xhci::kCcStall, hasData ? rq.length : 0);
      return false;
    }
    if (hasData && in) {
      const size_t take = reply.size() < rq.length ? reply.size() : rq.length;
      uint8_t *dst = at(static_cast<uint64_t>(data.hi) << 32 | data.lo, rq.length);
      memcpy(dst, reply.data(), take);
      if (take < rq.length) {
        ++shortData;
        if (!(data.control & xhci::kTrbIsp)) problem("a short data stage without ISP", 0);
        postTransfer(slotId, 1, dataBus, xhci::kCcShortPacket, rq.length - static_cast<unsigned>(take));
      }
    }
    postTransfer(slotId, 1, statusBus, xhci::kCcSuccess, 0);
    advance(ep, n + 1);
    return true;
  }
  Dev *s_dev(unsigned slotId) { return slots[slotId].dev; }

  // what a device does with a control request: 0 ok (reply filled), 1 stall, 2 no answer
  int deviceControl(Dev *d, const Request &rq, const std::vector<uint8_t> &out, std::vector<uint8_t> *reply) {
    d->log.push_back(rq);
    if (d->unresponsive) return 2;
    if (d->stallRequests.count(rq.request)) return 1;
    const unsigned type = rq.type & 0x60;   // 0 standard, 0x20 class
    const unsigned recipient = rq.type & 0x1f;
    if (type == 0) {
      if (rq.request == usb::kReqGetDescriptor && (rq.type & 0x80)) {
        const unsigned dt = rq.value >> 8;
        if (dt == usb::kDescDevice) *reply = d->device;
        else if (dt == usb::kDescConfig) *reply = d->config;
        else if (dt == usb::kDescHidReport) *reply = d->reportDesc;
        else return 1;
        return 0;
      }
      if (rq.request == usb::kReqSetConfiguration) {
        d->configuration = rq.value;
        return 0;
      }
      if (rq.request == usb::kReqSetInterface) {
        d->altSetting = rq.value;
        return 0;
      }
      if (rq.request == usb::kReqGetStatus) {
        *reply = {0, 0};
        return 0;
      }
      return 1;
    }
    if (type == 0x20 && recipient == 1) {   // HID class requests to the interface
      if (rq.request == usb::kHidReqSetProtocol) {
        d->protocol = rq.value;
        return 0;
      }
      if (rq.request == usb::kHidReqSetIdle) {
        d->idle = rq.value >> 8;
        return 0;
      }
      return 1;
    }
    if (type == 0x20 && d->isHub) return hubControl(d, rq, out, reply, recipient);
    return 1;
  }

  int hubControl(Dev *h, const Request &rq, const std::vector<uint8_t> &, std::vector<uint8_t> *reply, unsigned recipient) {
    if (rq.request == usb::kReqGetDescriptor && (rq.type & 0x80)) {
      if ((rq.value >> 8) != (h->superHub ? usb::kDescSsHub : usb::kDescHub)) return 1;
      *reply = h->hubDescriptor;
      return 0;
    }
    if (rq.request == usb::kReqSetHubDepth) {
      if (!h->superHub) return 1;
      h->hubDepth = rq.value;
      return 0;
    }
    if (recipient == 0) {   // the hub
      if (rq.request == usb::kReqGetStatus) {
        *reply = {0, 0, static_cast<uint8_t>(h->hubChange), static_cast<uint8_t>(h->hubChange >> 8)};
        return 0;
      }
      if (rq.request == usb::kReqClearFeature) {
        h->hubChange &= ~(1u << rq.value);
        return 0;
      }
      return 1;
    }
    if (recipient != 3 || rq.index < 1 || rq.index > h->ports.size()) return 1;
    Dev::HubPort &p = h->ports[rq.index - 1];
    if (rq.request == usb::kReqGetStatus) {
      *reply = {static_cast<uint8_t>(p.status), static_cast<uint8_t>(p.status >> 8), static_cast<uint8_t>(p.change), static_cast<uint8_t>(p.change >> 8)};
      return 0;
    }
    if (rq.request == usb::kReqSetFeature) {
      if (rq.value == usb::kHubPortPower) {
        p.power = true;
        p.status |= 0x0100;
        return 0;
      }
      if (rq.value == usb::kHubPortReset) {
        if (!(p.status & 1)) return 0;
        p.status |= 2;                                   // enabled
        if (!h->superHub && p.child) {                   // the speed the device was attached at
          p.status &= ~0x0600u;
          if (p.child->speed == xhci::kSpeedLow) p.status |= 0x0200;
          else if (p.child->speed == xhci::kSpeedHigh) p.status |= 0x0400;
        }
        p.change |= 1u << 4;                             // reset complete
        return 0;
      }
      return 1;
    }
    if (rq.request == usb::kReqClearFeature) {
      if (rq.value >= 16 && rq.value <= 20) p.change &= ~(1u << (rq.value - 16));
      else if (rq.value == usb::kHubCPortLinkState) p.change &= ~(1u << 5);
      else if (rq.value == usb::kHubCPortConfigError) p.change &= ~(1u << 7);
      else return 1;
      return 0;
    }
    return 1;
  }

  // Normal TRBs: an interrupt endpoint is served when its device has a report; one TRB per report.
  unsigned normalTrbs = 0;
  bool runNormal(unsigned slotId, unsigned dci, Endpoint &ep, uint64_t bus, const Trb &t) {
    const unsigned type = t.control >> xhci::kTrbTypeShift & 0x3f;
    if (type != xhci::kTrbNormal) {
      problem("unexpected TRB type %u on endpoint %u", type, dci);
      ++ringErrors;
      return false;
    }
    Dev *dev = s_dev(slotId);
    if (!dev) return false;
    if (ep.type == xhci::kEpInterruptIn && timeUs < ep.nextUs) return false;   // the next poll of the endpoint is not due yet
    std::vector<uint8_t> report;
    if (!deviceInterrupt(dev, dci, &report)) return false;   // NAK: the TRB stays
    if (ep.type == xhci::kEpInterruptIn) ep.nextUs = timeUs + (125ull << ep.interval);
    if (dci & 1) {
      auto st = dev->stallInterrupt.find(0x80 | dci >> 1);
      if (st != dev->stallInterrupt.end() && st->second) {   // the transfer fails: the endpoint halts, the report is lost
        --st->second;
        ep.state = xhci::kEpHalted;
        syncEpState(slotId, dci);
        postTransfer(slotId, dci, bus, xhci::kCcStall, t.status & 0x1ffff);
        return false;
      }
    }
    const uint32_t length = t.status & 0x1ffff;
    const size_t take = report.size() < length ? report.size() : length;
    memcpy(at(static_cast<uint64_t>(t.hi) << 32 | t.lo, length), report.data(), take);
    ++normalTrbs;
    if (t.control & (xhci::kTrbIoc | xhci::kTrbIsp)) postTransfer(slotId, dci, bus, take < length ? xhci::kCcShortPacket : xhci::kCcSuccess, length - static_cast<unsigned>(take));
    advance(ep, 1);
    return true;
  }
  bool deviceInterrupt(Dev *d, unsigned dci, std::vector<uint8_t> *report) {
    if (dci & 1) {
      const unsigned address = 0x80 | dci >> 1;
      if (d->isHub && d->statusChangeBitmap()) {
        const uint16_t m = d->statusChangeBitmap();
        *report = {static_cast<uint8_t>(m), static_cast<uint8_t>(m >> 8)};
        return true;
      }
      auto it = d->reports.find(address);
      if (it != d->reports.end() && !it->second.empty()) {
        *report = it->second.front();
        it->second.pop_front();
        return true;
      }
    }
    return false;
  }

  // The device side does what it wants between the driver's looks: serve the interrupt endpoints that have a TRB waiting.
  void service() {
    for (unsigned s = 1; s < 64; ++s) {
      if (!slots[s].enabled || !slots[s].dev) continue;
      for (unsigned dci = 2; dci < 32; ++dci) {
        Endpoint &ep = slots[s].ep[dci];
        if (ep.state == xhci::kEpRunning) runEndpoint(s, dci);
      }
    }
  }
};

}  // namespace model
