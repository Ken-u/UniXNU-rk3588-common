// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// TSADC of the RK3588 (rockchip_thermal.c, rk3588_tsadc_data: DATA(chn) at 0x2c + 4 * chn, a 9 bit code that rises with the temperature,
// piecewise linear: 215 = -40 C, 285 = 25 C, 350 = 85 C, 395 = 125 C). The sampling is the controller's auto mode: AUTO_PERIOD sets the
// interval, AUTO_SRC_CON the channels, AUTO_CON the enable.
#include <IOKit/IOLib.h>
#include <machine/machine_routines.h>
#include <stdint.h>
#include "Rk3588Thermal.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

namespace rk3588 {
namespace {

constexpr uint64_t kCruBase = 0xfd7c0000, kTsadcBase = 0xfec00000;
// CRU: clk_tsadc is CLKSEL_CON(41) bit 8 (0 GPLL, 1 24 MHz) over an 8 bit divider and CLKGATE_CON(12) bit 1, pclk_tsadc is CLKGATE_CON(12) bit 0;
// the two resets are SOFTRST_CON(12) bits 0 (pclk) and 1.
constexpr uint32_t kCruSel41 = 0x300 + 41 * 4, kCruGate12 = 0x800 + 12 * 4, kCruRst12 = 0xa00 + 12 * 4;
constexpr uint32_t kUserCon = 0x00, kAutoCon = 0x04, kAutoSrcCon = 0x0c, kHtIntEn = 0x14, kHshutGpioIntEn = 0x18, kHshutCruIntEn = 0x1c;
constexpr uint32_t kData0 = 0x2c, kHtIntDebounce = 0x14c, kHtShutDebounce = 0x150, kAutoPeriod = 0x154, kAutoPeriodHt = 0x158;

volatile uint32_t *gCru, *gTsadc;
bool gUp;

uint32_t rd(volatile uint32_t *base, uint32_t off) { return base[off / 4]; }
void wr(volatile uint32_t *base, uint32_t off, uint32_t value) {
  base[off / 4] = value;
  __asm__ volatile("dsb sy" ::: "memory");
}

struct Point {
  unsigned code;
  int milli;
};
constexpr Point kTable[] = {{215, -40000}, {285, 25000}, {350, 85000}, {395, 125000}};

}  // namespace

int thermalInit() {
  if (gUp) return 0;
  gCru = reinterpret_cast<volatile uint32_t *>(ml_io_map(kCruBase, 0x1000));
  if (!gCru) return -1;
  const uint32_t sel = rd(gCru, kCruSel41), gate = rd(gCru, kCruGate12), reset = rd(gCru, kCruRst12);
  printf("rk3588-dvfs: thermal: CRU sel41 %08x gate12 %08x softrst12 %08x (pclk %s, clk %s, reset %s %s)\n", sel, gate, reset, gate & 1 ? "GATED" : "on",
         gate & 2 ? "GATED" : "on", reset & 1 ? "ASSERTED" : "released", reset & 2 ? "ASSERTED" : "released");
  if (gate & 3) {
    // the firmware left the controller unclocked: 2 MHz from the 24 MHz oscillator (the vendor's assigned-clock-rates), gates open, resets released
    wr(gCru, kCruSel41, (0x1ffu << 16) | (1u << 8) | 11);
    wr(gCru, kCruGate12, 3u << 16);
    IODelay(20);
  }
  if (reset & 3) {
    wr(gCru, kCruRst12, 3u << 16);
    IODelay(20);
  }
  gTsadc = reinterpret_cast<volatile uint32_t *>(ml_io_map(kTsadcBase, 0x400));
  if (!gTsadc) return -2;
  printf("rk3588-dvfs: thermal: tsadc user %08x auto %08x src %08x ht-int-en %08x shut-gpio %08x shut-cru %08x period %u\n", rd(gTsadc, kUserCon), rd(gTsadc, kAutoCon),
         rd(gTsadc, kAutoSrcCon), rd(gTsadc, kHtIntEn), rd(gTsadc, kHshutGpioIntEn), rd(gTsadc, kHshutCruIntEn), rd(gTsadc, kAutoPeriod));
  if (!(rd(gTsadc, kAutoCon) & 1)) {
    wr(gTsadc, kAutoPeriod, 5000);
    wr(gTsadc, kAutoPeriodHt, 5000);
    wr(gTsadc, kHtIntDebounce, 4);
    wr(gTsadc, kHtShutDebounce, 4);
    wr(gTsadc, kAutoCon, 1u << 24);   // shutdown polarity bit written as 0 (active low), as the vendor driver does
  }
  // sample every channel; the shutdown interrupts (kHshutGpioIntEn, kHshutCruIntEn) stay as they are
  wr(gTsadc, kAutoSrcCon, (0x7fu << 16) | 0x7f);
  wr(gTsadc, kAutoCon, (1u << 16) | 1u);
  IODelay(20000);
  gUp = true;
  return 0;
}

int thermalRead(unsigned channel, int *milli) {
  if (!gUp || channel >= kThermalChannels) return -1;
  const unsigned code = rd(gTsadc, kData0 + channel * 4) & 0x1ff;
  if (code <= kTable[0].code || code >= 455) return -1;
  unsigned i = 0;
  while (i + 2 < sizeof kTable / sizeof kTable[0] && code >= kTable[i + 1].code) ++i;
  const Point &a = kTable[i], &b = kTable[i + 1];
  *milli = a.milli + int((int64_t(int(code) - int(a.code)) * (b.milli - a.milli)) / int(b.code - a.code));
  return 0;
}

int thermalMean() {
  int64_t sum = 0;
  int n = 0;
  for (unsigned c = 0; c < kThermalChannels; ++c) {
    int t = 0;
    if (c != kThermalNpu && !thermalRead(c, &t)) {
      sum += t;
      ++n;
    }
  }
  return n ? int(sum / n) : INT32_MIN;
}

int thermalHottest() {
  int hottest = INT32_MIN;
  for (unsigned c = 0; c < kThermalChannels; ++c) {
    int t = 0;
    if (c != kThermalNpu && !thermalRead(c, &t) && t > hottest) hottest = t;
  }
  return hottest;
}

}  // namespace rk3588
