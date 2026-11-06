// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Xhci: a polled xHCI host driver for the DWC3 controllers of the RK3588, shared by the boards.
//
// What is shared (this directory): the xHCI core (XhciCore.h: rings, contexts, commands, transfers; a pure computation over a memory
// window and a register accessor, so tests/test-xhci.cpp runs it against a model of the controller), the USB device enumeration and hub
// handling (UsbHost.h) and the class drivers built on them. What a board supplies: where its controllers are and a DMA window for each,
// as the table `rk3588_xhci_controllers` in a source file of its own that `build-rk3588-xhci.py` compiles next to these.
//
// The firmware (edk2-rk3588: UsbHcdInitDxe, the combo PHY and USB DP PHY code) powers the PHYs and puts each DWC3 core into host mode; the
// driver resets the xHC itself and takes it from there.
//
// Boot argument rk3588-xhci=N: 0 off (default); 1 look only (map the controllers and print the registers as the firmware left them);
// 2 also reset and start the controllers, and watch the root hub ports; 3 also enumerate what is plugged in (reads the descriptors);
// 4 also hubs; 5 also configure and poll the HID devices and publish their reports through rk3588_xhci_hid_sink.
#pragma once
#include <stdbool.h>
#include <stdint.h>

struct Rk3588XhciController {
  const char *name;
  uint64_t base;          // physical address of the DWC3 block (the xHCI capability registers are at its start)
  uint64_t dma_phys;      // a DMA window the kernel does not use, below 4 GiB (xHC structures are at most 64 bit, many cores only 32)
  uint32_t dma_bytes;
};

#ifdef __cplusplus
extern "C" {
#endif

// The board's controllers and how many there are.
extern const struct Rk3588XhciController rk3588_xhci_controllers[];
extern const unsigned rk3588_xhci_controller_count;

// Where the decoded input goes (mode 5): the board's input path. Any pointer may be null (those reports are dropped). Called from the xHCI thread.
struct Rk3588XhciHidSink {
  void (*keyboard)(const uint8_t *report, unsigned length);                // a boot keyboard report: modifiers, reserved, six keys (8 bytes)
  void (*mouse)(int32_t buttons, int32_t dx, int32_t dy, int32_t wheel);   // one mouse report (buttons: bit 0 left, 1 right, 2 middle)
  void (*mouseProtocol)(bool report);                                      // the protocol the mouse was put in: report (its descriptor's layout, may have a wheel) or boot
};
extern const struct Rk3588XhciHidSink rk3588_xhci_hid_sink;

// Called by the board's probe once the kernel is up; reads the boot arguments and returns at once without rk3588-xhci=N.
void rk3588_xhci_start(void);

#ifdef __cplusplus
}
#endif
