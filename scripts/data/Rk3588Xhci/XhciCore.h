// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// XhciCore.h -- a polled xHCI host controller driver core (xHCI specification 1.2): command ring, one event ring, transfer rings, device and input
// contexts, slots, port reset. A computation over a register accessor and one DMA window, with no kernel include: Rk3588Xhci.cpp runs it on the
// board's DWC3 controllers, tests/test-xhci.cpp against a model of a controller.
//
// What the caller provides (struct Hw): 32 bit register access at offsets of the controller's block, a delay, a clock, a memory barrier, and a DMA
// window whose bus address the controller can reach (below 4 GiB when the controller is a 32 bit one) and whose mapping is coherent (the kernel maps
// its window as normal non-cacheable memory, so no cache maintenance is done here). A log macro XHCI_LOG(fmt, ...) may be defined before including this.
//
// How it is used: Controller::init() resets and starts the controller. Everything that talks to a device is synchronous (control(), command()): it
// queues, rings the doorbell and polls the event ring until its own completion arrives; events that are not the awaited one are handled on the way
// (port changes are remembered, a transfer event of an endpoint with a handler goes to the handler). Handlers run inside poll() and must not start a
// synchronous operation; they may queue another TRB. One thread drives a controller: nothing here is reentrant against another thread.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "XhciRegs.h"

#ifndef XHCI_LOG
#define XHCI_LOG(...) ((void)0)
#endif

namespace xhci {

constexpr unsigned kMaxSlots = 16;              // device slots used (the controller may have more)
constexpr unsigned kRingTrbs = 64;              // TRBs of a transfer ring, the link TRB included (1 KiB)
constexpr unsigned kCmdRingTrbs = 64;
constexpr unsigned kEventRingTrbs = 128;
constexpr unsigned kRingsPerSlot = 8;           // endpoint 0 and seven more
constexpr unsigned kCtrlBuffer = 1024;          // the control transfer data buffer of a slot (the longest control transfer)
constexpr unsigned kSlotBufferBytes = 4096;

// The layout of a slot's part of the DMA window (all offsets are multiples of 64, the largest alignment needed):
//   0x0000 output device context (2 KiB: 32 entries of up to 64 bytes)   0x1000 input context (4 KiB: 33 entries of up to 64 bytes)
//   0x2000 eight transfer rings of 1 KiB                                 0x4000 buffers (control data 1 KiB, then endpoint buffers)
constexpr uint32_t kSlotAreaBytes = 0x5000;

struct Trb {
  uint32_t lo, hi, status, control;
};

struct Hw {
  void *ctx;
  uint32_t (*read32)(void *ctx, uint32_t offset);
  void (*write32)(void *ctx, uint32_t offset, uint32_t value);
  void (*delayUs)(void *ctx, uint32_t us);
  uint64_t (*nowUs)(void *ctx);
  void (*barrier)(void *ctx);                    // all earlier memory accesses complete before the later ones (dsb sy)
  uint8_t *dma_virt;
  uint64_t dma_bus;
  uint32_t dma_bytes;
};

struct TransferEvent {
  uint8_t slot, dci, code;
  uint32_t residual;                             // bytes not transferred
  uint64_t trb;                                  // bus address of the TRB the event is for
};
typedef void (*TransferHandler)(void *arg, const TransferEvent &event);

// How the device sits on the bus, for the slot context of Address Device.
struct SlotParams {
  uint8_t speed;                                 // SlotSpeed
  uint32_t route;                                // route string: the hub port numbers, the tier closest to the root in the low nibble
  uint8_t rootPort;                              // root hub port of the topmost hub (1 based)
  uint8_t parentSlot, parentPort;                // the high speed hub that holds the transaction translator of a low / full speed device (0: none)
  bool mtt;                                      // that hub has one transaction translator per port
  uint16_t mps0;                                 // initial maximum packet size of the default control endpoint
};

struct EpConfig {
  uint8_t dci;                                   // 2 * endpoint number + (1 for IN)
  uint8_t type;                                  // EpType
  uint16_t maxPacket;
  uint8_t maxBurst;
  uint8_t interval;                              // exponent, in 125 microsecond units
  uint8_t mult;
  uint16_t avgTrbLength;
  uint32_t maxEsitPayload;
};

// What a Configure Endpoint changes in the slot context of a hub: the Hub bit, the number of ports, the transaction translator think time.
struct HubUpdate {
  bool hub;
  uint8_t ports;
  bool mtt;
  uint8_t ttt;                                   // think time, 0..3 (8, 16, 24, 32 full speed bit times)
};

struct Ring {
  volatile Trb *trb;
  uint64_t bus;
  unsigned size;
  unsigned enq;
  uint32_t pcs;                                  // producer cycle state
};

struct EndpointState {
  bool active;
  Ring ring;
  TransferHandler handler;
  void *arg;
  unsigned queued;                               // TRBs queued and not yet reported (the interrupt endpoints use it)
};

struct SlotState {
  bool enabled;
  bool addressed;
  uint8_t speed;
  uint32_t bufferUsed;                           // bytes of the buffer area handed out by allocBuffer
  uint8_t ringOf[32];                            // device context index -> ring number + 1 (0: none)
  unsigned ringsUsed;
  EndpointState ep[32];
};

struct SetupPacket {
  uint8_t bmRequestType, bRequest;
  uint16_t wValue, wIndex, wLength;
};

class Controller {
 public:
  bool init(const Hw &hw);
  bool running() const { return running_; }
  unsigned portCount() const { return maxPorts_; }
  bool portIsUsb3(unsigned port) const { return port < 32 && (usb3Ports_ >> port & 1); }
  uint32_t portStatus(unsigned port);                       // PORTSC, ports numbered from 1
  void clearPortChange(unsigned port, uint32_t bits);       // write 1 to the change bits among `bits`
  bool powerPort(unsigned port);
  bool resetPort(unsigned port, uint32_t *speed);           // hot reset; true when the port is enabled afterwards
  uint32_t takePortEvents();                                // bit n: a Port Status Change Event for port n arrived

  void poll();                                              // handle every event that is there
  int enableSlot();                                         // the slot number, or -1
  bool addressDevice(unsigned slot, const SlotParams &params);
  bool setEp0MaxPacket(unsigned slot, uint16_t mps);
  bool configureEndpoints(unsigned slot, const EpConfig *eps, unsigned count, const HubUpdate *hub);
  bool disableSlot(unsigned slot);
  bool control(unsigned slot, const SetupPacket &setup, void *data, uint16_t *actual, uint32_t timeoutMs = 2000);
  bool recoverEndpoint(unsigned slot, unsigned dci);        // reset a halted endpoint and move its dequeue pointer to the enqueue pointer
  bool registerEndpoint(unsigned slot, unsigned dci, TransferHandler handler, void *arg);
  // An interrupt / bulk IN or OUT transfer of `length` bytes into / out of `buffer` (from allocBuffer). Returns the TRB's bus address (0: ring full).
  uint64_t queueTransfer(unsigned slot, unsigned dci, uint64_t bufferBus, uint32_t length);
  uint8_t *allocBuffer(unsigned slot, uint32_t bytes, uint64_t *bus);   // from the slot's buffer area (64 byte aligned); freed with the slot
  uint32_t contextSize() const { return ctxSize_; }
  uint64_t nowUs() { return hw_.nowUs(hw_.ctx); }
  void delayUs(uint32_t us) { hw_.delayUs(hw_.ctx, us); }
  const uint32_t *deviceContext(unsigned slot) const { return reinterpret_cast<const uint32_t *>(slotVirt(slot)); }
  uint32_t statCommands() const { return statCommands_; }
  uint32_t statEvents() const { return statEvents_; }
  uint32_t statUnexpected() const { return statUnexpected_; }

 private:
  struct Wait {
    bool active, done;
    uint64_t trb, dataTrb;
    unsigned slot, dci;
    uint8_t code;
    uint32_t residual, param, slotId;
    bool dataSeen;
    uint8_t dataCode;
  };

  uint32_t rd(uint32_t off) { return hw_.read32(hw_.ctx, off); }
  void wr(uint32_t off, uint32_t value) { hw_.write32(hw_.ctx, off, value); }
  void wr64(uint32_t off, uint64_t value) {
    wr(off, static_cast<uint32_t>(value));
    wr(off + 4, static_cast<uint32_t>(value >> 32));
  }
  bool waitBits(uint32_t off, uint32_t mask, uint32_t want, uint32_t timeoutUs);
  uint32_t portOffset(unsigned port) const { return opBase_ + kPortBase + 0x10 * (port - 1); }
  bool alloc(uint32_t bytes, uint32_t align, uint8_t **virt, uint64_t *bus);
  uint8_t *slotVirt(unsigned slot) const { return hw_.dma_virt + slotArea_ + (slot - 1) * kSlotAreaBytes; }
  uint64_t slotBus(unsigned slot) const { return hw_.dma_bus + slotArea_ + (slot - 1) * kSlotAreaBytes; }
  uint32_t *outCtx(unsigned slot, unsigned index) const { return reinterpret_cast<uint32_t *>(slotVirt(slot) + index * ctxSize_); }
  uint32_t *inCtx(unsigned slot, unsigned index) const { return reinterpret_cast<uint32_t *>(slotVirt(slot) + 0x1000 + index * ctxSize_); }
  uint64_t inCtxBus(unsigned slot) const { return slotBus(slot) + 0x1000; }
  void initRing(Ring *ring, volatile Trb *trb, uint64_t bus, unsigned size);
  uint64_t putTrb(Ring *ring, uint32_t lo, uint32_t hi, uint32_t status, uint32_t control);   // the TRB's bus address
  void ringDoorbell(unsigned target, unsigned value) { wr(dbBase_ + 4 * target, value); }
  void handleEvent(const Trb &event);
  void processEvents();
  bool runCommand(uint32_t lo, uint32_t hi, uint32_t control, Wait *result = nullptr, uint32_t timeoutMs = 2000);
  EndpointState *endpoint(unsigned slot, unsigned dci);
  Ring *ringFor(unsigned slot, unsigned dci, bool create);
  void freeSlot(unsigned slot);
  void buildEp(uint32_t *ctx, const EpConfig &c, const Ring &ring);

  Hw hw_;
  bool running_;
  uint32_t capLength_, opBase_, rtBase_, dbBase_, ctxSize_, pageBytes_;
  unsigned maxPorts_, maxSlots_;
  uint32_t usb3Ports_;
  uint32_t pendingPorts_;
  uint32_t dmaUsed_, slotArea_;
  uint64_t *dcbaa_;
  Ring cmdRing_;
  volatile Trb *eventRing_;
  uint64_t eventBus_;
  unsigned eventDeq_;
  uint32_t eventCcs_;
  Wait cmdWait_, xferWait_;
  SlotState slots_[kMaxSlots + 1];
  uint32_t statCommands_, statEvents_, statUnexpected_;
};

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Implementation

inline void zeroBytes(void *p, size_t n) {
  volatile uint32_t *w = static_cast<volatile uint32_t *>(p);
  for (size_t i = 0; i < n / 4; ++i) w[i] = 0;
}

inline bool Controller::alloc(uint32_t bytes, uint32_t align, uint8_t **virt, uint64_t *bus) {
  const uint32_t start = (dmaUsed_ + align - 1) & ~(align - 1);
  if (start + bytes > hw_.dma_bytes || start + bytes < start) return false;
  dmaUsed_ = start + bytes;
  zeroBytes(hw_.dma_virt + start, (bytes + 3) & ~3u);
  *virt = hw_.dma_virt + start;
  *bus = hw_.dma_bus + start;
  return true;
}

inline bool Controller::waitBits(uint32_t off, uint32_t mask, uint32_t want, uint32_t timeoutUs) {
  const uint64_t until = hw_.nowUs(hw_.ctx) + timeoutUs;
  while (true) {
    if ((rd(off) & mask) == want) return true;
    if (hw_.nowUs(hw_.ctx) >= until) return (rd(off) & mask) == want;
    hw_.delayUs(hw_.ctx, 100);
  }
}

inline void Controller::initRing(Ring *ring, volatile Trb *trb, uint64_t bus, unsigned size) {
  ring->trb = trb;
  ring->bus = bus;
  ring->size = size;
  ring->enq = 0;
  ring->pcs = 1;
  for (unsigned i = 0; i < size; ++i) {
    trb[i].lo = 0;
    trb[i].hi = 0;
    trb[i].status = 0;
    trb[i].control = 0;
  }
}

// Write one TRB at the enqueue pointer with the producer cycle state, then move on; at the last place of the ring the link TRB goes in (with the
// current cycle state, so the controller follows it) and the state toggles. The cycle bit is written last, after a barrier: the controller treats
// a TRB as its own once the bit matches its own cycle state.
inline uint64_t Controller::putTrb(Ring *ring, uint32_t lo, uint32_t hi, uint32_t status, uint32_t control) {
  volatile Trb *t = &ring->trb[ring->enq];
  const uint64_t bus = ring->bus + 16ull * ring->enq;
  t->lo = lo;
  t->hi = hi;
  t->status = status;
  hw_.barrier(hw_.ctx);
  t->control = (control & ~kTrbCycle) | ring->pcs;
  ++ring->enq;
  if (ring->enq == ring->size - 1) {
    volatile Trb *link = &ring->trb[ring->enq];
    link->lo = static_cast<uint32_t>(ring->bus);
    link->hi = static_cast<uint32_t>(ring->bus >> 32);
    link->status = 0;
    hw_.barrier(hw_.ctx);
    link->control = (kTrbLink << kTrbTypeShift) | kTrbToggleCycle | ring->pcs;
    ring->enq = 0;
    ring->pcs ^= 1;
  }
  hw_.barrier(hw_.ctx);
  return bus;
}

inline bool Controller::init(const Hw &hw) {
  hw_ = hw;
  running_ = false;
  dmaUsed_ = 0;
  pendingPorts_ = 0;
  usb3Ports_ = 0;
  statCommands_ = statEvents_ = statUnexpected_ = 0;
  cmdWait_ = Wait();
  xferWait_ = Wait();
  for (unsigned i = 0; i <= kMaxSlots; ++i) slots_[i] = SlotState();
  const uint32_t cap = rd(kCapLength), hcs1 = rd(kHcsParams1), hcs2 = rd(kHcsParams2), hcc1 = rd(kHccParams1);
  capLength_ = cap & 0xff;
  if (capLength_ < 0x10 || capLength_ > 0x100 || (cap >> 16) < 0x0090 || (cap >> 16) == 0xffff) {
    XHCI_LOG("rk3588-xhci: not an xHCI controller (cap %#x)\n", cap);
    return false;
  }
  opBase_ = capLength_;
  rtBase_ = rd(kRtsOff) & ~0x1fu;
  dbBase_ = rd(kDbOff) & ~3u;
  ctxSize_ = (hcc1 & kHccCsz) ? 64 : 32;
  maxPorts_ = hcs1 >> 24;
  if (maxPorts_ > 31) maxPorts_ = 31;
  maxSlots_ = hcs1 & 0xff;
  if (maxSlots_ > kMaxSlots) maxSlots_ = kMaxSlots;
  if (!maxSlots_ || !(hcs1 >> 8 & 0x7ff)) return false;

  if (!waitBits(opBase_ + kUsbSts, kStsCnr, 0, 1000000)) {
    XHCI_LOG("rk3588-xhci: controller not ready (usbsts %#x)\n", rd(opBase_ + kUsbSts));
    return false;
  }
  if (rd(opBase_ + kUsbCmd) & kCmdRun) {
    wr(opBase_ + kUsbCmd, rd(opBase_ + kUsbCmd) & ~kCmdRun);
    if (!waitBits(opBase_ + kUsbSts, kStsHalted, kStsHalted, 200000)) {
      XHCI_LOG("rk3588-xhci: controller does not halt (usbsts %#x)\n", rd(opBase_ + kUsbSts));
      return false;
    }
  }
  wr(opBase_ + kUsbCmd, kCmdHcReset);
  if (!waitBits(opBase_ + kUsbCmd, kCmdHcReset, 0, 1000000) || !waitBits(opBase_ + kUsbSts, kStsCnr, 0, 1000000)) {
    XHCI_LOG("rk3588-xhci: reset did not finish (usbcmd %#x usbsts %#x)\n", rd(opBase_ + kUsbCmd), rd(opBase_ + kUsbSts));
    return false;
  }

  // the controller's page size: the lowest bit set in PAGESIZE is 2^(12 + n) bytes
  const uint32_t pagesize = rd(opBase_ + kPageSize) & 0xffff;
  if (!pagesize) return false;
  unsigned shift = 0;
  while (!(pagesize >> shift & 1)) ++shift;
  pageBytes_ = 1u << (12 + shift);

  const uint32_t scratch = (hcs2 >> 21 & 0x1f) << 5 | (hcs2 >> 27 & 0x1f);
  uint8_t *virt = nullptr;
  uint64_t bus = 0;
  if (!alloc(8 * (maxSlots_ + 1), 4096, &virt, &bus)) return false;
  dcbaa_ = reinterpret_cast<uint64_t *>(virt);
  const uint64_t dcbaaBus = bus;
  if (scratch) {
    uint8_t *array = nullptr;
    uint64_t arrayBus = 0;
    if (!alloc(8 * scratch, 64, &array, &arrayBus)) return false;
    for (uint32_t i = 0; i < scratch; ++i) {
      uint8_t *page = nullptr;
      uint64_t pageBus = 0;
      if (!alloc(pageBytes_, pageBytes_, &page, &pageBus)) return false;
      reinterpret_cast<uint64_t *>(array)[i] = pageBus;
    }
    dcbaa_[0] = arrayBus;
  }
  if (!alloc(16 * kCmdRingTrbs, 4096, &virt, &bus)) return false;
  initRing(&cmdRing_, reinterpret_cast<volatile Trb *>(virt), bus, kCmdRingTrbs);
  if (!alloc(16 * kEventRingTrbs, 4096, &virt, &bus)) return false;
  eventRing_ = reinterpret_cast<volatile Trb *>(virt);
  eventBus_ = bus;
  eventDeq_ = 0;
  eventCcs_ = 1;
  uint8_t *erst = nullptr;
  uint64_t erstBus = 0;
  if (!alloc(64, 64, &erst, &erstBus)) return false;
  reinterpret_cast<uint64_t *>(erst)[0] = eventBus_;
  reinterpret_cast<uint32_t *>(erst)[2] = kEventRingTrbs;
  slotArea_ = (dmaUsed_ + 4095) & ~4095u;
  if (slotArea_ + kMaxSlots * kSlotAreaBytes > hw_.dma_bytes) {
    XHCI_LOG("rk3588-xhci: the DMA window is too small (%u bytes used before the slots, %u in all)\n", slotArea_, hw_.dma_bytes);
    return false;
  }
  zeroBytes(hw_.dma_virt + slotArea_, kMaxSlots * kSlotAreaBytes);
  dmaUsed_ = slotArea_ + maxSlots_ * kSlotAreaBytes;

  wr(opBase_ + kConfig, maxSlots_);
  wr64(opBase_ + kDcbaap, dcbaaBus);
  wr64(opBase_ + kCrcr, cmdRing_.bus | kCrcrRcs);
  const uint32_t ir = rtBase_ + 0x20;
  wr(ir + kErstSz, 1);
  wr64(ir + kErdp, eventBus_);
  wr64(ir + kErstBa, erstBus);
  wr(ir + kImod, 0);
  hw_.barrier(hw_.ctx);
  wr(opBase_ + kUsbCmd, kCmdRun);
  if (!waitBits(opBase_ + kUsbSts, kStsHalted, 0, 100000)) {
    XHCI_LOG("rk3588-xhci: controller does not run (usbsts %#x)\n", rd(opBase_ + kUsbSts));
    return false;
  }
  running_ = true;

  // which ports are USB 3: the supported protocol capabilities (ID 2) of the extended capability list
  uint32_t off = (hcc1 >> 16) * 4;
  for (unsigned guard = 0; off && guard < 16 && off + 16 <= 0x10000; ++guard) {
    const uint32_t w0 = rd(off);
    if ((w0 & 0xff) == kXcapSupportedProtocol && (w0 >> 24) == 3) {
      const uint32_t w2 = rd(off + 8), first = w2 & 0xff, count = w2 >> 8 & 0xff;
      for (uint32_t p = first; p < first + count && p < 32; ++p) usb3Ports_ |= 1u << p;
    }
    const uint32_t next = w0 >> 8 & 0xff;
    off = next ? off + next * 4 : 0;
  }
  XHCI_LOG("rk3588-xhci: running: %u ports (usb3 mask %#x), %u slots, %u byte contexts, page %u, %u scratchpad pages\n", maxPorts_, usb3Ports_, maxSlots_, ctxSize_,
           pageBytes_, scratch);
  return true;
}

inline uint32_t Controller::portStatus(unsigned port) { return port >= 1 && port <= maxPorts_ ? rd(portOffset(port)) : 0; }

inline void Controller::clearPortChange(unsigned port, uint32_t bits) {
  if (port < 1 || port > maxPorts_) return;
  const uint32_t off = portOffset(port), v = rd(off);
  wr(off, (v & ~kPortPreserve) | (bits & kPortChange));
}

inline bool Controller::powerPort(unsigned port) {
  if (port < 1 || port > maxPorts_) return false;
  const uint32_t off = portOffset(port), v = rd(off);
  if (v & kPortPp) return true;
  wr(off, (v & ~kPortPreserve) | kPortPp);
  hw_.delayUs(hw_.ctx, 20000);
  return (rd(off) & kPortPp) != 0;
}

inline bool Controller::resetPort(unsigned port, uint32_t *speed) {
  if (port < 1 || port > maxPorts_) return false;
  const uint32_t off = portOffset(port);
  uint32_t v = rd(off);
  // a USB 3 port whose link went Inactive (a device that failed to train) leaves it only through a warm reset; a hot reset (PR) is ignored there
  const bool warm = portIsUsb3(port) && ((v & kPortPlsMask) >> kPortPlsShift) == kPlsInactive;
  wr(off, (v & ~kPortPreserve) | (warm ? kPortWpr : kPortPr));
  bool done = false;
  for (unsigned i = 0; i < 600; ++i) {
    hw_.delayUs(hw_.ctx, 1000);
    v = rd(off);
    if (v & (kPortPrc | kPortWrc)) {
      done = true;
      break;
    }
  }
  wr(off, (v & ~kPortPreserve) | kPortPrc | kPortWrc);
  hw_.delayUs(hw_.ctx, 10000);   // reset recovery
  v = rd(off);
  pendingPorts_ &= ~(1u << port);
  if (!done || !(v & kPortPed) || !(v & kPortCcs)) {
    XHCI_LOG("rk3588-xhci: port %u: reset %s, portsc %#x\n", port, done ? "left the port disabled" : "did not finish", v);
    return false;
  }
  if (speed) *speed = (v & kPortSpeedMask) >> kPortSpeedShift;
  return true;
}

inline uint32_t Controller::takePortEvents() {
  const uint32_t m = pendingPorts_;
  pendingPorts_ = 0;
  return m;
}

inline void Controller::handleEvent(const Trb &event) {
  ++statEvents_;
  const uint32_t type = event.control >> kTrbTypeShift & 0x3f, code = event.status >> 24;
  const uint64_t trb = static_cast<uint64_t>(event.hi) << 32 | event.lo;
  if (type == kEvCommandCompletion) {
    if (cmdWait_.active && trb == cmdWait_.trb) {
      cmdWait_.done = true;
      cmdWait_.code = static_cast<uint8_t>(code);
      cmdWait_.param = event.status & 0xffffff;
      cmdWait_.slotId = event.control >> 24;
    } else {
      ++statUnexpected_;
    }
  } else if (type == kEvTransfer) {
    TransferEvent te;
    te.slot = static_cast<uint8_t>(event.control >> 24);
    te.dci = static_cast<uint8_t>(event.control >> 16 & 0x1f);
    te.code = static_cast<uint8_t>(code);
    te.residual = event.status & 0xffffff;
    te.trb = trb;
    if (xferWait_.active && te.slot == xferWait_.slot && te.dci == xferWait_.dci) {
      if (trb == xferWait_.trb) {
        xferWait_.done = true;
        xferWait_.code = te.code;
        if (!xferWait_.dataSeen) xferWait_.residual = te.residual;   // a short data stage keeps its own residual (the status stage has none)
      } else if (trb == xferWait_.dataTrb) {
        xferWait_.dataSeen = true;
        xferWait_.dataCode = te.code;
        xferWait_.residual = te.residual;
        if (te.code != kCcSuccess && te.code != kCcShortPacket) {   // the data stage failed: there is no status stage event
          xferWait_.done = true;
          xferWait_.code = te.code;
        }
      } else if (te.code != kCcSuccess && te.code != kCcShortPacket) {   // the setup stage failed
        xferWait_.done = true;
        xferWait_.code = te.code;
      }
      return;
    }
    EndpointState *ep = te.slot >= 1 && te.slot <= kMaxSlots && te.dci >= 1 && te.dci < 32 ? &slots_[te.slot].ep[te.dci] : nullptr;
    if (ep && ep->active && ep->handler) {
      if (ep->queued) --ep->queued;
      ep->handler(ep->arg, te);
    } else {
      ++statUnexpected_;
    }
  } else if (type == kEvPortStatusChange) {
    const unsigned port = event.lo >> 24;
    if (port >= 1 && port < 32) pendingPorts_ |= 1u << port;
  } else if (type == kEvHostController) {
    XHCI_LOG("rk3588-xhci: host controller event, code %u\n", code);
  } else {
    ++statUnexpected_;
  }
}

// Take every event whose cycle bit matches the consumer cycle state, then tell the controller how far software has read (ERDP, with the event
// handler busy bit written as 1 to clear it).
inline void Controller::processEvents() {
  bool any = false;
  unsigned sinceUpdate = 0;
  const uint32_t ir = rtBase_ + 0x20;
  while (true) {
    volatile Trb *t = &eventRing_[eventDeq_];
    const uint32_t control = t->control;
    if ((control & kTrbCycle) != eventCcs_) break;
    hw_.barrier(hw_.ctx);
    Trb event;
    event.lo = t->lo;
    event.hi = t->hi;
    event.status = t->status;
    event.control = control;
    if (++eventDeq_ == kEventRingTrbs) {
      eventDeq_ = 0;
      eventCcs_ ^= 1;
    }
    any = true;
    handleEvent(event);
    // a handler may queue what makes the controller produce the next event at once: give the controller its room back now and then, or a long run
    // of events could fill the ring before ERDP moves
    if (++sinceUpdate == 32) {
      sinceUpdate = 0;
      wr64(ir + kErdp, (eventBus_ + 16ull * eventDeq_) | kErdpEhb);
    }
  }
  if (any) {
    wr64(ir + kErdp, (eventBus_ + 16ull * eventDeq_) | kErdpEhb);
    wr(ir + kIman, kImanIp);   // clear the interrupt pending bit (write 1 to clear); the interrupt itself is not enabled
    wr(opBase_ + kUsbSts, kStsEint);
  }
}

inline void Controller::poll() {
  if (running_) processEvents();
}

inline bool Controller::runCommand(uint32_t lo, uint32_t hi, uint32_t control, Wait *result, uint32_t timeoutMs) {
  if (!running_) return false;
  cmdWait_ = Wait();
  cmdWait_.active = true;
  cmdWait_.trb = putTrb(&cmdRing_, lo, hi, 0, control);
  ++statCommands_;
  ringDoorbell(0, 0);
  const uint64_t until = hw_.nowUs(hw_.ctx) + 1000ull * timeoutMs;
  while (!cmdWait_.done) {
    processEvents();
    if (cmdWait_.done) break;
    if (hw_.nowUs(hw_.ctx) >= until) {
      XHCI_LOG("rk3588-xhci: command type %u timed out (usbsts %#x crcr %#x)\n", control >> kTrbTypeShift & 0x3f, rd(opBase_ + kUsbSts), rd(opBase_ + kCrcr));
      cmdWait_.active = false;
      return false;
    }
    hw_.delayUs(hw_.ctx, 50);
  }
  cmdWait_.active = false;
  if (result) *result = cmdWait_;
  if (cmdWait_.code != kCcSuccess) {
    XHCI_LOG("rk3588-xhci: command type %u failed, completion code %u\n", control >> kTrbTypeShift & 0x3f, cmdWait_.code);
    return false;
  }
  return true;
}

inline int Controller::enableSlot() {
  Wait result;
  if (!runCommand(0, 0, kCmdEnableSlot << kTrbTypeShift, &result)) return -1;
  const unsigned slot = result.slotId;
  if (slot < 1 || slot > maxSlots_) {
    XHCI_LOG("rk3588-xhci: Enable Slot returned slot %u\n", slot);
    return -1;
  }
  freeSlot(slot);
  slots_[slot].enabled = true;
  return static_cast<int>(slot);
}

inline void Controller::freeSlot(unsigned slot) {
  slots_[slot] = SlotState();
  if (dcbaa_) dcbaa_[slot] = 0;
  zeroBytes(slotVirt(slot), kSlotAreaBytes);
}

inline Ring *Controller::ringFor(unsigned slot, unsigned dci, bool create) {
  SlotState &s = slots_[slot];
  if (dci < 1 || dci > 31) return nullptr;
  if (!s.ringOf[dci]) {
    if (!create || s.ringsUsed >= kRingsPerSlot) return nullptr;
    const unsigned n = s.ringsUsed++;
    s.ringOf[dci] = static_cast<uint8_t>(n + 1);
    initRing(&s.ep[dci].ring, reinterpret_cast<volatile Trb *>(slotVirt(slot) + 0x2000 + 1024 * n), slotBus(slot) + 0x2000 + 1024 * n, kRingTrbs);
    s.ep[dci].active = true;
  }
  return &s.ep[dci].ring;
}

inline EndpointState *Controller::endpoint(unsigned slot, unsigned dci) {
  if (slot < 1 || slot > kMaxSlots || dci < 1 || dci > 31 || !slots_[slot].ep[dci].active) return nullptr;
  return &slots_[slot].ep[dci];
}

inline uint8_t *Controller::allocBuffer(unsigned slot, uint32_t bytes, uint64_t *bus) {
  SlotState &s = slots_[slot];
  uint32_t start = s.bufferUsed ? s.bufferUsed : kCtrlBuffer;
  start = (start + 63) & ~63u;
  if (start + bytes > kSlotBufferBytes) return nullptr;
  s.bufferUsed = start + bytes;
  if (bus) *bus = slotBus(slot) + 0x4000 + start;
  return slotVirt(slot) + 0x4000 + start;
}

inline void Controller::buildEp(uint32_t *ctx, const EpConfig &c, const Ring &ring) {
  ctx[0] = static_cast<uint32_t>(c.mult & 3) << 8 | static_cast<uint32_t>(c.interval) << 16 | (c.maxEsitPayload >> 16 & 0xff) << 24;
  ctx[1] = 3u << 1 | static_cast<uint32_t>(c.type) << 3 | static_cast<uint32_t>(c.maxBurst) << 8 | static_cast<uint32_t>(c.maxPacket) << 16;
  const uint64_t deq = (ring.bus + 16ull * ring.enq) | ring.pcs;
  ctx[2] = static_cast<uint32_t>(deq);
  ctx[3] = static_cast<uint32_t>(deq >> 32);
  ctx[4] = static_cast<uint32_t>(c.avgTrbLength) | (c.maxEsitPayload & 0xffff) << 16;
}

inline bool Controller::addressDevice(unsigned slot, const SlotParams &p) {
  if (slot < 1 || slot > maxSlots_ || !slots_[slot].enabled) return false;
  Ring *ring = ringFor(slot, 1, true);
  if (!ring) return false;
  zeroBytes(slotVirt(slot) + 0x1000, 0x1000);
  zeroBytes(slotVirt(slot), 0x800);
  uint32_t *control = inCtx(slot, 0), *sc = inCtx(slot, 1), *ep0 = inCtx(slot, 2);
  control[1] = 3;   // add the slot context (bit 0) and the default control endpoint (bit 1)
  sc[0] = (p.route & 0xfffff) | static_cast<uint32_t>(p.speed) << kSlotSpeedShift | (p.mtt ? kSlotMtt : 0) | 1u << kSlotEntriesShift;
  sc[1] = static_cast<uint32_t>(p.rootPort) << kSlotRootPortShift;
  sc[2] = static_cast<uint32_t>(p.parentSlot) | static_cast<uint32_t>(p.parentPort) << 8;
  EpConfig c = {};
  c.dci = 1;
  c.type = kEpControl;
  c.maxPacket = p.mps0;
  c.avgTrbLength = 8;
  buildEp(ep0, c, *ring);
  dcbaa_[slot] = slotBus(slot);
  hw_.barrier(hw_.ctx);
  const uint64_t in = inCtxBus(slot);
  if (!runCommand(static_cast<uint32_t>(in), static_cast<uint32_t>(in >> 32), kCmdAddressDevice << kTrbTypeShift | static_cast<uint32_t>(slot) << 24)) {
    dcbaa_[slot] = 0;
    return false;
  }
  slots_[slot].addressed = true;
  slots_[slot].speed = p.speed;
  return true;
}

inline bool Controller::setEp0MaxPacket(unsigned slot, uint16_t mps) {
  if (slot < 1 || slot > maxSlots_ || !slots_[slot].addressed) return false;
  zeroBytes(slotVirt(slot) + 0x1000, 0x1000);
  uint32_t *control = inCtx(slot, 0), *ep0 = inCtx(slot, 2);
  control[1] = 2;   // evaluate the default control endpoint only
  const uint32_t *out = outCtx(slot, 1);
  for (unsigned i = 0; i < 5; ++i) ep0[i] = out[i];
  ep0[1] = (ep0[1] & 0xffffu) | static_cast<uint32_t>(mps) << 16;
  hw_.barrier(hw_.ctx);
  const uint64_t in = inCtxBus(slot);
  return runCommand(static_cast<uint32_t>(in), static_cast<uint32_t>(in >> 32), kCmdEvaluateContext << kTrbTypeShift | static_cast<uint32_t>(slot) << 24);
}

inline bool Controller::configureEndpoints(unsigned slot, const EpConfig *eps, unsigned count, const HubUpdate *hub) {
  if (slot < 1 || slot > maxSlots_ || !slots_[slot].addressed) return false;
  zeroBytes(slotVirt(slot) + 0x1000, 0x1000);
  uint32_t *control = inCtx(slot, 0), *sc = inCtx(slot, 1);
  const uint32_t *outSlot = outCtx(slot, 0);
  for (unsigned i = 0; i < 4; ++i) sc[i] = outSlot[i];
  unsigned last = sc[0] >> kSlotEntriesShift;
  uint32_t add = 1;   // the slot context
  for (unsigned i = 0; i < count; ++i) {
    const EpConfig &c = eps[i];
    Ring *ring = ringFor(slot, c.dci, true);
    if (!ring) return false;
    buildEp(inCtx(slot, 1 + c.dci), c, *ring);
    add |= 1u << c.dci;
    if (c.dci > last) last = c.dci;
  }
  sc[0] = (sc[0] & ~(0x1fu << kSlotEntriesShift)) | static_cast<uint32_t>(last) << kSlotEntriesShift;
  if (hub && hub->hub) {
    sc[0] |= kSlotHub | (hub->mtt ? kSlotMtt : 0);
    sc[1] = (sc[1] & 0x00ffffffu) | static_cast<uint32_t>(hub->ports) << kSlotNumPortsShift;
    sc[2] = (sc[2] & ~(3u << 16)) | static_cast<uint32_t>(hub->ttt & 3) << 16;
  }
  control[1] = add;
  hw_.barrier(hw_.ctx);
  const uint64_t in = inCtxBus(slot);
  return runCommand(static_cast<uint32_t>(in), static_cast<uint32_t>(in >> 32), kCmdConfigureEndpoint << kTrbTypeShift | static_cast<uint32_t>(slot) << 24);
}

inline bool Controller::disableSlot(unsigned slot) {
  if (slot < 1 || slot > maxSlots_ || !slots_[slot].enabled) return false;
  const bool ok = runCommand(0, 0, kCmdDisableSlot << kTrbTypeShift | static_cast<uint32_t>(slot) << 24);
  freeSlot(slot);
  return ok;
}

inline bool Controller::registerEndpoint(unsigned slot, unsigned dci, TransferHandler handler, void *arg) {
  EndpointState *ep = endpoint(slot, dci);
  if (!ep) return false;
  ep->handler = handler;
  ep->arg = arg;
  return true;
}

inline uint64_t Controller::queueTransfer(unsigned slot, unsigned dci, uint64_t bufferBus, uint32_t length) {
  EndpointState *ep = endpoint(slot, dci);
  if (!ep || ep->queued + 2 >= kRingTrbs - 1) return 0;
  const uint64_t trb = putTrb(&ep->ring, static_cast<uint32_t>(bufferBus), static_cast<uint32_t>(bufferBus >> 32), length, kTrbNormal << kTrbTypeShift | kTrbIoc | kTrbIsp);
  ++ep->queued;
  ringDoorbell(slot, dci);
  return trb;
}

inline bool Controller::control(unsigned slot, const SetupPacket &s, void *data, uint16_t *actual, uint32_t timeoutMs) {
  EndpointState *ep = endpoint(slot, 1);
  if (!running_ || !ep || s.wLength > kCtrlBuffer) return false;
  const bool in = (s.bmRequestType & 0x80) != 0, hasData = s.wLength != 0;
  uint8_t *buf = slotVirt(slot) + 0x4000;
  const uint64_t bufBus = slotBus(slot) + 0x4000;
  if (hasData && !in) {
    for (unsigned i = 0; i < s.wLength; ++i) buf[i] = static_cast<uint8_t *>(data)[i];
  } else if (hasData) {
    for (unsigned i = 0; i < s.wLength; ++i) buf[i] = 0;
  }
  const uint32_t setupLo = s.bmRequestType | static_cast<uint32_t>(s.bRequest) << 8 | static_cast<uint32_t>(s.wValue) << 16;
  const uint32_t setupHi = s.wIndex | static_cast<uint32_t>(s.wLength) << 16;
  const uint32_t trt = hasData ? (in ? 3u : 2u) : 0u;
  xferWait_ = Wait();
  xferWait_.slot = slot;
  xferWait_.dci = 1;
  putTrb(&ep->ring, setupLo, setupHi, 8, kTrbSetupStage << kTrbTypeShift | kTrbIdt | trt << 16);
  if (hasData)
    xferWait_.dataTrb = putTrb(&ep->ring, static_cast<uint32_t>(bufBus), static_cast<uint32_t>(bufBus >> 32), s.wLength,
                                kTrbDataStage << kTrbTypeShift | (in ? kTrbDirIn | kTrbIsp : 0));
  xferWait_.trb = putTrb(&ep->ring, 0, 0, 0, kTrbStatusStage << kTrbTypeShift | kTrbIoc | (in && hasData ? 0 : kTrbDirIn));
  xferWait_.active = true;
  ringDoorbell(slot, 1);
  const uint64_t until = hw_.nowUs(hw_.ctx) + 1000ull * timeoutMs;
  [[maybe_unused]] bool timedOut = false;
  while (!xferWait_.done) {
    processEvents();
    if (xferWait_.done) break;
    if (hw_.nowUs(hw_.ctx) >= until) {
      timedOut = true;
      break;
    }
    hw_.delayUs(hw_.ctx, 50);
  }
  xferWait_.active = false;
  const bool ok = xferWait_.done && xferWait_.code == kCcSuccess;
  if (!ok) {
    XHCI_LOG("rk3588-xhci: slot %u control %#x/%#x/%#x/%#x/%u %s (code %u)\n", slot, s.bmRequestType, s.bRequest, s.wValue, s.wIndex, s.wLength,
             timedOut ? "timed out" : "failed", xferWait_.code);
    recoverEndpoint(slot, 1);
    return false;
  }
  uint32_t got = hasData ? s.wLength : 0;
  if (hasData && xferWait_.dataSeen) got = s.wLength >= xferWait_.residual ? s.wLength - xferWait_.residual : 0;   // a short data stage
  if (in && hasData)
    for (unsigned i = 0; i < got; ++i) static_cast<uint8_t *>(data)[i] = buf[i];
  if (actual) *actual = static_cast<uint16_t>(got);
  return true;
}

// After a transfer error or a timeout: stop the endpoint if it still runs, reset it if it is halted, and make the controller start at the next
// TRB we write (Set TR Dequeue Pointer to the enqueue pointer with the current cycle state). The TDs that were queued are abandoned.
inline bool Controller::recoverEndpoint(unsigned slot, unsigned dci) {
  EndpointState *ep = endpoint(slot, dci);
  if (!ep || !running_) return false;
  const uint32_t state = outCtx(slot, dci)[0] & 7;
  if (state == kEpRunning)
    runCommand(0, 0, kCmdStopEndpoint << kTrbTypeShift | static_cast<uint32_t>(dci) << 16 | static_cast<uint32_t>(slot) << 24);
  else if (state == kEpHalted)
    runCommand(0, 0, kCmdResetEndpoint << kTrbTypeShift | static_cast<uint32_t>(dci) << 16 | static_cast<uint32_t>(slot) << 24);
  const uint64_t deq = (ep->ring.bus + 16ull * ep->ring.enq) | ep->ring.pcs;
  ep->queued = 0;
  return runCommand(static_cast<uint32_t>(deq), static_cast<uint32_t>(deq >> 32),
                    kCmdSetTrDequeue << kTrbTypeShift | static_cast<uint32_t>(dci) << 16 | static_cast<uint32_t>(slot) << 24);
}

}  // namespace xhci
