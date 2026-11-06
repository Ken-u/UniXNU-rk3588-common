// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Kernel glue of Rk3588Xhci: the boot argument, the register and DMA window mappings, the register accessor of XhciCore.h and the thread that drives
// the controllers. See Rk3588Xhci.h.
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <stdint.h>

#define XHCI_LOG(...) printf(__VA_ARGS__)
#include "Rk3588Xhci.h"
#include "UsbHost.h"
#include "XhciCore.h"
#include "XhciRegs.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
// Normal non-cacheable memory (the mapping XNU gives its panic log): the controller's DMA is coherent with it without cache maintenance, and
// unaligned accesses and DC ZVA are fine, unlike ml_io_map's Device memory.
extern "C" vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);

namespace {

constexpr unsigned kMaxControllers = 4;

struct Instance {
  const Rk3588XhciController *cfg;
  volatile uint8_t *regs;
  bool up;
  xhci::Controller hc;
  usb::Host host;   // mode 3 and above
  uint32_t lastPortsc[32];
};

uint32_t gMode;
Instance gInst[kMaxControllers];

uint32_t bootArg(const char *name, uint32_t fallback) {
  uint32_t value = fallback;
  if (!PE_parse_boot_argn(name, &value, sizeof(value))) return fallback;
  return value;
}

uint64_t nowUs() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000;
}

void delayUs(uint32_t us) {
  if (us >= 1000) {
    IOSleep((us + 999) / 1000);
    return;
  }
  const uint64_t until = nowUs() + us;
  while (nowUs() < until) {
  }
}

// ---- the register accessor and the rest of struct xhci::Hw
uint32_t hwRead(void *ctx, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(static_cast<Instance *>(ctx)->regs + off); }
void hwWrite(void *ctx, uint32_t off, uint32_t value) {
  __asm__ volatile("dsb sy" ::: "memory");   // the rings and contexts are in memory before the register write that makes the controller look at them
  *reinterpret_cast<volatile uint32_t *>(static_cast<Instance *>(ctx)->regs + off) = value;
  __asm__ volatile("dsb sy" ::: "memory");
}
void hwDelay(void *, uint32_t us) { delayUs(us); }
uint64_t hwNow(void *) { return nowUs(); }
void hwBarrier(void *) { __asm__ volatile("dsb sy" ::: "memory"); }

uint32_t rd32(volatile uint8_t *base, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(base + off); }

// Print the registers as the firmware left them. Nothing is written.
void lookAt(const Rk3588XhciController &c) {
  vm_offset_t virt = ml_io_map(static_cast<vm_offset_t>(c.base), 0x10000);
  if (!virt) {
    printf("rk3588-xhci: %s: cannot map %#llx\n", c.name, static_cast<unsigned long long>(c.base));
    return;
  }
  volatile uint8_t *base = reinterpret_cast<volatile uint8_t *>(virt);
  const uint32_t cap = rd32(base, xhci::kCapLength), hcs1 = rd32(base, xhci::kHcsParams1), hcs2 = rd32(base, xhci::kHcsParams2),
                 hcs3 = rd32(base, xhci::kHcsParams3), hcc1 = rd32(base, xhci::kHccParams1), dboff = rd32(base, xhci::kDbOff),
                 rtsoff = rd32(base, xhci::kRtsOff), hcc2 = rd32(base, xhci::kHccParams2);
  const uint32_t capLength = cap & 0xff;
  printf("rk3588-xhci: %s: caplen %u hciversion %#x hcs1 %#x hcs2 %#x hcs3 %#x hcc1 %#x hcc2 %#x dboff %#x rtsoff %#x\n", c.name, capLength, cap >> 16, hcs1,
         hcs2, hcs3, hcc1, hcc2, dboff, rtsoff);
  printf("rk3588-xhci: %s: %u slots, %u interrupters, %u ports, %s contexts, %s, scratchpad bufs %u, %s addressing\n", c.name, hcs1 & 0xff, (hcs1 >> 8) & 0x7ff,
         hcs1 >> 24, hcc1 & xhci::kHccCsz ? "64 byte" : "32 byte", hcc1 & xhci::kHccPpc ? "port power control" : "no port power control",
         ((hcs2 >> 21) & 0x1f) << 5 | ((hcs2 >> 27) & 0x1f), hcc1 & xhci::kHccAc64 ? "64 bit" : "32 bit");
  if (capLength < 0x20 || capLength > 0x100) return;
  volatile uint8_t *op = base + capLength;
  printf("rk3588-xhci: %s: usbcmd %#x usbsts %#x pagesize %#x config %#x crcr %#x dcbaap %#x\n", c.name, rd32(op, xhci::kUsbCmd), rd32(op, xhci::kUsbSts),
         rd32(op, xhci::kPageSize), rd32(op, xhci::kConfig), rd32(op, xhci::kCrcr), rd32(op, xhci::kDcbaap));
  const unsigned ports = hcs1 >> 24;
  for (unsigned p = 0; p < ports && p < 16; ++p)
    printf("rk3588-xhci: %s: port %u portsc %#x pmsc %#x li %#x\n", c.name, p + 1, rd32(op, xhci::kPortBase + 0x10 * p), rd32(op, xhci::kPortBase + 0x10 * p + 4),
           rd32(op, xhci::kPortBase + 0x10 * p + 8));
  // extended capabilities: a linked list of 32 bit words, ID 2 = supported protocol (which port numbers are USB 2 and which USB 3)
  uint32_t off = (hcc1 >> 16) * 4;
  for (unsigned guard = 0; off && guard < 16 && off + 16 <= 0x8000; ++guard) {
    const uint32_t w0 = rd32(base, off);
    if ((w0 & 0xff) == xhci::kXcapSupportedProtocol) {
      const uint32_t w2 = rd32(base, off + 8);
      printf("rk3588-xhci: %s: xecp %#x: protocol USB %x.%02x name %#x: ports %u..%u, %u speed ids\n", c.name, off, w0 >> 24, (w0 >> 16) & 0xff,
             rd32(base, off + 4), w2 & 0xff, (w2 & 0xff) + ((w2 >> 8) & 0xff) - 1, w2 >> 28);
    } else {
      printf("rk3588-xhci: %s: xecp %#x: id %u (%#x)\n", c.name, off, w0 & 0xff, w0);
    }
    const uint32_t next = (w0 >> 8) & 0xff;
    off = next ? off + next * 4 : 0;
  }
  printf("rk3588-xhci: %s: dwc3 gsnpsid %#x gctl %#x gsts %#x ghwparams0 %#x gusb2phycfg %#x gusb3pipectl %#x gfladj %#x dcfg %#x dsts %#x\n", c.name,
         rd32(base, xhci::kDwc3Gsnpsid), rd32(base, xhci::kDwc3Gctl), rd32(base, xhci::kDwc3Gsts), rd32(base, xhci::kDwc3Ghwparams0),
         rd32(base, xhci::kDwc3Gusb2PhyCfg), rd32(base, xhci::kDwc3Gusb3PipeCtl), rd32(base, xhci::kDwc3Gfladj), rd32(base, xhci::kDwc3Dcfg),
         rd32(base, xhci::kDwc3Dsts));
}

bool bringUp(Instance *in) {
  const Rk3588XhciController &c = *in->cfg;
  vm_offset_t regs = ml_io_map(static_cast<vm_offset_t>(c.base), 0x10000);
  vm_offset_t dma = ml_io_map_wcomb(static_cast<vm_offset_t>(c.dma_phys), c.dma_bytes);
  if (!regs || !dma) {
    printf("rk3588-xhci: %s: cannot map the registers (%d) or the DMA window (%d)\n", c.name, regs != 0, dma != 0);
    return false;
  }
  if (c.dma_phys + c.dma_bytes > (1ull << 32)) {
    printf("rk3588-xhci: %s: the DMA window is above 4 GiB and the controller is a 32 bit one\n", c.name);
    return false;
  }
  in->regs = reinterpret_cast<volatile uint8_t *>(regs);
  xhci::Hw hw = {};
  hw.ctx = in;
  hw.read32 = hwRead;
  hw.write32 = hwWrite;
  hw.delayUs = hwDelay;
  hw.nowUs = hwNow;
  hw.barrier = hwBarrier;
  hw.dma_virt = reinterpret_cast<uint8_t *>(dma);
  hw.dma_bus = c.dma_phys;
  hw.dma_bytes = c.dma_bytes;
  in->up = in->hc.init(hw);
  if (!in->up) {
    const uint32_t opBase = rd32(in->regs, xhci::kCapLength) & 0xff;
    printf("rk3588-xhci: %s: start failed (usbcmd %#x usbsts %#x)\n", c.name, rd32(in->regs, opBase + xhci::kUsbCmd), rd32(in->regs, opBase + xhci::kUsbSts));
  }
  return in->up;
}

// Mode 2: report port changes and reset a port that has a device, to see what is connected and at which speed.
void watchPorts(Instance *in) {
  in->hc.poll();
  const uint32_t events = in->hc.takePortEvents();
  for (unsigned port = 1; port <= in->hc.portCount(); ++port) {
    const uint32_t v = in->hc.portStatus(port);
    if (!(events >> port & 1) && v == in->lastPortsc[port]) continue;
    printf("rk3588-xhci: %s: port %u (%s) portsc %#x%s: %s, speed %u, link state %u\n", in->cfg->name, port, in->hc.portIsUsb3(port) ? "usb3" : "usb2", v,
           events >> port & 1 ? " [event]" : "", v & xhci::kPortCcs ? "connected" : "no device", (v & xhci::kPortSpeedMask) >> xhci::kPortSpeedShift,
           (v & xhci::kPortPlsMask) >> xhci::kPortPlsShift);
    in->hc.clearPortChange(port, v);
    in->lastPortsc[port] = in->hc.portStatus(port);
    if ((v & xhci::kPortCcs) && gMode >= 2) {
      uint32_t speed = 0;
      const bool ok = in->hc.resetPort(port, &speed);
      printf("rk3588-xhci: %s: port %u reset: %s, speed %u, portsc %#x\n", in->cfg->name, port, ok ? "enabled" : "failed", speed, in->hc.portStatus(port));
      in->lastPortsc[port] = in->hc.portStatus(port);
    }
  }
}

usb::HidSink gSink;

void xhciThread(void *, wait_result_t) {
  unsigned count = rk3588_xhci_controller_count < kMaxControllers ? rk3588_xhci_controller_count : kMaxControllers;
  for (unsigned i = 0; i < count; ++i) lookAt(rk3588_xhci_controllers[i]);
  if (gMode < 2) return;
  gSink.keyboard = rk3588_xhci_hid_sink.keyboard;
  gSink.mouse = rk3588_xhci_hid_sink.mouse;
  gSink.mouseProtocol = rk3588_xhci_hid_sink.mouseProtocol;
  for (unsigned i = 0; i < count; ++i) {
    gInst[i].cfg = &rk3588_xhci_controllers[i];
    if (bringUp(&gInst[i]) && gMode >= 3) gInst[i].host.init(&gInst[i].hc, rk3588_xhci_controllers[i].name, &gSink, gMode);
  }
  uint64_t last = nowUs();
  while (true) {
    bool hid = false;
    for (unsigned i = 0; i < count; ++i) {
      if (!gInst[i].up) continue;
      if (gMode >= 3) {
        gInst[i].host.poll();
        hid = hid || gInst[i].host.hidActive();
      } else {
        watchPorts(&gInst[i]);
      }
    }
    IOSleep(hid || gMode < 3 ? 1 : 5);   // three HID transfers are queued per endpoint: a millisecond keeps a 1 kHz mouse from losing reports; without one, 5 ms is plenty
    const uint64_t now = nowUs();
    if (now - last > 30000000) {
      last = now;
      for (unsigned i = 0; i < count; ++i) {
        if (!gInst[i].up) continue;
        printf("rk3588-xhci: %s: events %u unexpected %u commands %u", gInst[i].cfg->name, gInst[i].hc.statEvents(), gInst[i].hc.statUnexpected(),
               gInst[i].hc.statCommands());
        if (gMode >= 3)
          printf(", %u device(s), enumerated %u, hid reports %u, hid errors %u", gInst[i].host.deviceCount(), gInst[i].host.statEnumerated(),
                 gInst[i].host.statReports(), gInst[i].host.statErrors());
        printf("\n");
      }
    }
  }
}

}  // namespace

extern "C" void rk3588_xhci_start(void) {
  gMode = bootArg("rk3588-xhci", 0);
  if (!gMode) return;
  printf("rk3588-xhci: mode %u, %u controllers\n", gMode, rk3588_xhci_controller_count);
  thread_t thread = nullptr;
  if (kernel_thread_start(xhciThread, nullptr, &thread) == KERN_SUCCESS && thread) thread_deallocate(thread);
}
