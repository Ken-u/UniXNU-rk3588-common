// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The GPU as the frequency management sees it (Rk3588Gpu.h). Registers: the Mali-G610's control block at 0xfb000000, the CRU (clk_gpu_src is
// CLKSEL_CON(158), mux bits 7:5 over GPLL, CPLL, AUPLL, NPLL, SPLL and a divider in bits 4:0; its gates are CLKGATE_CON(66)) and the PMU's
// power domain status (domain 12 is the GPU). The Mali driver sets the clock source itself, not through the clock service, and turns the domain
// on and never off again.
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include "Rk3588Domain.h"
#include "Rk3588Governor.h"
#include "Rk3588Gpu.h"
#include "Rk3588Pmic.h"
#include "Rk3588Scmi.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

namespace rk3588 {
namespace {

constexpr uint64_t kCruBase = 0xfd7c0000, kPmuBase = 0xfd8d8000, kGpuBase = 0xfb000000;
constexpr uint32_t kSel158 = 0x578, kSel159 = 0x57c, kGate66 = 0x908;
// CLKSEL_CON(158): the clock service's PVTPLL setting is bit 14, the source and divider the Mali driver writes bits 7:0
constexpr uint32_t kSel158Mask = 0x40ff;
constexpr uint32_t kPmuRepair = 0x290, kGpuOnBit = 1u << 1;   // the domain's repair status: the vendor driver's "power domain 12 is on"
constexpr uint32_t kGpuIdReg = 0x0, kGpuCommand = 0x30, kGpuStatus = 0x34, kCycleLo = 0x90, kCycleHi = 0x94, kShaderReady = 0x140;
constexpr uint32_t kGpuId = 0xa8670005, kTimeEnable = 0x103;   // GPU_COMMAND: start the cycle counter (what the Mali driver's own clock probe writes)

volatile uint32_t *gCru, *gPmu, *gGpu;

inline uint32_t cru(uint32_t off) { return gCru[off / 4]; }
inline void cruWrite(uint32_t off, uint32_t value) {
  gCru[off / 4] = value;
  __asm__ volatile("dsb sy" ::: "memory");
}
inline uint32_t gpuRead(uint32_t off) { return gGpu[off / 4]; }
inline void gpuWrite(uint32_t off, uint32_t value) {
  gGpu[off / 4] = value;
  __asm__ volatile("dsb sy" ::: "memory");
}
inline uint64_t counter() {
  uint64_t v;
  __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(v));
  return v;
}

// pll_rk3588: 24 MHz * (m + k / 65536) / p / 2^s (CON0 m [9:0]; CON1 p [5:0], s [8:6], power-down bit 13; CON2 k); `con` is the index of CON0.
unsigned pllMhz(unsigned con) {
  const uint32_t c0 = gCru[con], c1 = gCru[con + 1], c2 = gCru[con + 2];
  const uint64_t m = c0 & 0x3ff, p = c1 & 0x3f, s = (c1 >> 6) & 7, k = c2 & 0xffff;
  if (!p || (c1 & (1u << 13))) return 0;
  return unsigned(((((24000ull * (m * 65536 + k)) / 65536 / p) >> s) + 500) / 1000);
}

uint64_t cycles() {
  uint32_t hi = gpuRead(kCycleHi), lo = gpuRead(kCycleLo);
  const uint32_t again = gpuRead(kCycleHi);
  if (again != hi) {
    hi = again;
    lo = gpuRead(kCycleLo);
  }
  return (uint64_t(hi) << 32) | lo;
}

// Writes a rail voltage and waits for the regulator's ramp when it goes up.
int railSet(const Rk3588DvfsRail &rail, uint32_t from, uint32_t to) {
  if (to < rail.min_uv || to > rail.max_uv || rail.kind != kRk3588DvfsRk806) return -10;
  const int r = rk3588_rk806_buck_set_uv(rail.index, to);
  if (!r && to > from) IODelay((to - from) / (rail.slew_uv_per_us ? rail.slew_uv_per_us : 1) + 30);
  return r;
}

void report(const char *when, const Rk3588DvfsRail &rail) {
  uint64_t hz = 0;
  const int g = rk3588_scmi_clock_rate_get(kRk3588ScmiClkGpu, &hz);
  uint32_t uv = 0;
  const int v = voltageGet(rail, &uv);
  const uint32_t sel = cru(kSel158);
  const unsigned first = gpuMeasureMhz(), second = gpuMeasureMhz();
  printf("rk3588-dvfs: gpu %s: service %u MHz (%d); sel158 %08x (source %u, divider %u) sel159 %08x gate66 %08x; gpll %u cpll %u aupll %u npll %u v0pll %u MHz; "
         "rail %u mV (%d); measured %u %u MHz; status %08x shaders %08x\n",
         when, mhz(hz), g, sel, (sel >> 5) & 7, (sel & 0x1f) + 1, cru(kSel159), cru(kGate66), pllMhz(112), pllMhz(104), pllMhz(96), pllMhz(120), pllMhz(88), uv / 1000, v,
         first, second, gpuRead(kGpuStatus), gpuRead(kShaderReady));
}

}  // namespace

bool gpuAttach(unsigned waitMs) {
  if (!gGpu) {
    gCru = reinterpret_cast<volatile uint32_t *>(ml_io_map(kCruBase, 0x1000));
    gPmu = reinterpret_cast<volatile uint32_t *>(ml_io_map(kPmuBase, 0x400));
  }
  if (!gCru || !gPmu) return false;
  // the domain on and the clocks the driver opens (clk_gpu_src and clk_gpu: CLKGATE_CON(66) bits 1 and 4, 0 = open)
  for (unsigned waited = 0;; waited += 250) {
    if ((gPmu[kPmuRepair / 4] & kGpuOnBit) && !(cru(kGate66) & ((1u << 1) | (1u << 4)))) break;
    if (waited >= waitMs) return false;
    IOSleep(250);
  }
  if (!gGpu) gGpu = reinterpret_cast<volatile uint32_t *>(ml_io_map(kGpuBase, 0x1000));
  if (!gGpu || gpuRead(kGpuIdReg) != kGpuId) return false;
  return true;
}

unsigned gpuMeasureMhz() {
  if (!gGpu) return 0;
  gpuWrite(kGpuCommand, kTimeEnable);
  IODelay(200);
  const uint64_t c0 = cycles(), t0 = counter();
  IODelay(5000);
  const uint64_t c1 = cycles(), t1 = counter();
  uint64_t frequency = 0;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
  if (!frequency || t1 <= t0 || c1 < c0) return 0;
  return unsigned((c1 - c0) * frequency / (t1 - t0) / 1000000);
}

void gpuSweep(const Rk3588DvfsRail &rail) {
  uint32_t uv0 = 0;
  const uint32_t sel0 = cru(kSel158);
  if (voltageGet(rail, &uv0)) {
    printf("rk3588-dvfs: gpu sweep: the rail does not answer\n");
    return;
  }
  uint64_t rates[40] = {};
  unsigned count = 0;
  int ranged = 0;
  if (rk3588_scmi_clock_rates(kRk3588ScmiClkGpu, rates, 40, &count, &ranged) || ranged || !count) {
    printf("rk3588-dvfs: gpu sweep: no rate list\n");
    return;
  }
  report("start", rail);
  // the rail at the vendor's limit for every operating point, so that no rate of the service is too fast for it
  const uint32_t top = rail.max_uv < 850000 ? rail.max_uv : 850000;
  int r = railSet(rail, uv0, top);
  printf("rk3588-dvfs: gpu sweep: rail %u -> %u mV: %d\n", uv0 / 1000, top / 1000, r);
  if (r) return;
  for (unsigned i = count; i-- > 0;) {
    const uint64_t t0 = mach_absolute_time();
    const int set = rk3588_scmi_clock_rate_set(kRk3588ScmiClkGpu, rates[i]);
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
    IODelay(300);
    char when[40];
    snprintf(when, sizeof when, "set %u (%d, %u us)", mhz(rates[i]), set, unsigned(ns / 1000));
    report(when, rail);
  }
  // back to what the Mali driver had set
  cruWrite(kSel158, (kSel158Mask << 16) | (sel0 & kSel158Mask));
  IODelay(300);
  report("restored", rail);
  r = railSet(rail, top, uv0);
  printf("rk3588-dvfs: gpu sweep: rail back to %u mV: %d\n", uv0 / 1000, r);
  report("end", rail);
}

void gpuVoltageSweep(const Rk3588DvfsRail &rail) {
  uint32_t uv0 = 0;
  const uint32_t sel0 = cru(kSel158);
  if (voltageGet(rail, &uv0)) {
    printf("rk3588-dvfs: gpu vsweep: the rail does not answer\n");
    return;
  }
  report("start", rail);
  constexpr unsigned kSettings[] = {900, 800, 700, 600, 500, 400, 300};
  constexpr uint32_t kHigh = 850000, kLow = 675000, kStep = 12500;
  uint32_t uv = uv0;
  if (railSet(rail, uv, kHigh)) return;
  uv = kHigh;
  for (const unsigned setting : kSettings) {
    if (uv < kHigh && railSet(rail, uv, kHigh)) break;
    uv = kHigh;
    if (rk3588_scmi_clock_rate_set(kRk3588ScmiClkGpu, uint64_t(setting) * 1000000)) {
      printf("rk3588-dvfs: gpu vsweep: the service refused %u MHz\n", setting);
      continue;
    }
    IODelay(300);
    char line[300] = {};
    unsigned used = 0;
    for (uint32_t v = kHigh; v >= kLow && used + 12 < sizeof line; v -= kStep) {
      if (v < uv) {
        if (railSet(rail, uv, v)) break;
        uv = v;
        IODelay(150);
      }
      const unsigned measured = gpuMeasureMhz();
      used += unsigned(snprintf(line + used, sizeof line - used, " %u.%u:%u", v / 1000, v % 1000 / 100, measured));
      if (measured < setting * 6 / 10) break;   // well under the nominal rate: no point going lower
    }
    printf("rk3588-dvfs: gpu vsweep: setting %u MHz, mV:measured MHz%s\n", setting, line);
  }
  if (uv < kHigh) railSet(rail, uv, kHigh);
  cruWrite(kSel158, (kSel158Mask << 16) | (sel0 & kSel158Mask));
  IODelay(300);
  railSet(rail, kHigh, uv0);
  report("end", rail);
}

void gpuWatch() {
  unsigned quiet = 0;
  for (;;) {
    const uint64_t start = mach_absolute_time();
    uint64_t span = 0;
    unsigned samples = 0, active = 0, power = 0, shaders = 0;
    uint32_t seen = 0;
    while (span < 50 * 1000 * 1000) {   // 50 ms
      const uint32_t status = gpuRead(kGpuStatus);
      seen |= status;
      ++samples;
      active += status & 1;
      power += (status >> 1) & 1;
      shaders += gpuRead(kShaderReady) != 0;
      IODelay(4);
      absolutetime_to_nanoseconds(mach_absolute_time() - start, &span);
    }
    if (active || power || shaders || ++quiet >= 40) {
      printf("rk3588-dvfs: gpu watch: %u samples: busy %u%% power %u%% shaders on %u%% (status bits %x)\n", samples, active * 100 / samples, power * 100 / samples,
             shaders * 100 / samples, seen);
      quiet = 0;
    }
  }
}

// ---- the governor --------------------------------------------------------------------------------------------------------------------

namespace {

// The operating points: the service's rate setting with the vendor's voltage for it (rk3588s.dtsi gpu_opp_table, the typical-die column). From 300 MHz
// up the service runs the GPU from its PVTPLL, a ring oscillator that follows the supply, so a point only has its nominal speed at its own voltage and a low
// supply makes it slower, not unsafe; 200 MHz is GPLL / 6, a fixed clock. `actual` is what the cycle counter measured on this board at the point's
// voltage (mode 10), for the log and the power model; the 800 and 900 settings share one ring and differ by the voltage only.
struct GpuOpp {
  unsigned setting, actual;
  uint32_t uv;
};
constexpr GpuOpp kOpp[] = {{200, 198, 675000}, {300, 310, 675000}, {400, 396, 675000}, {500, 502, 675000},
                           {600, 598, 675000}, {700, 719, 700000}, {800, 852, 750000}, {900, 957, 800000}};
constexpr unsigned kOpps = sizeof kOpp / sizeof kOpp[0];
constexpr unsigned kNone = ~0u;
// what the Mali driver leaves: CPLL / 2 at the 750 mV the firmware sets; the model compares with it
constexpr unsigned kBootMhz = 750;
constexpr uint32_t kBootUv = 750000;
constexpr uint64_t kGpuCoefficient = 2982;   // uW / MHz / V^2, rk3588s.dtsi gpu node
constexpr unsigned kSampleMs = 1, kIdleSampleMs = 8, kWindowMs = 25, kDownWindows = 3, kIdleAfterMs = 1000, kMaxFailures = 5;
constexpr unsigned kHispeedPermille = 400, kTargetPermille = 300;
constexpr uint64_t kStatsNs = 10000000000ull;

struct GpuState {
  unsigned current = kNone;
  uint32_t uv = 0;
  unsigned failures = 0;
};

unsigned lowestAtLeast(unsigned floorLevel, unsigned ceiling, uint64_t mhzNeeded) {
  unsigned i = floorLevel;
  while (i < ceiling && kOpp[i].actual < mhzNeeded) ++i;
  return i;
}

// Rate and voltage together. The voltage of a faster point first, a slower point's last; the service must report the setting asked for.
int gpuLevelSet(const Rk3588DvfsRail &rail, GpuState &g, unsigned to) {
  if (to == g.current) return 0;
  const GpuOpp &o = kOpp[to];
  if (o.uv > g.uv) {
    const int r = railSet(rail, g.uv, o.uv);
    if (r) return r;
    g.uv = o.uv;
  }
  int r = rk3588_scmi_clock_rate_set(kRk3588ScmiClkGpu, uint64_t(o.setting) * 1000000);
  if (r) return 20 + r;
  uint64_t hz = 0;
  rk3588_scmi_clock_rate_get(kRk3588ScmiClkGpu, &hz);
  if (mhz(hz) != o.setting) {
    printf("rk3588-dvfs: gpu: the service reads %u MHz after a set to %u\n", mhz(hz), o.setting);
    return 30;
  }
  g.current = to;
  if (o.uv < g.uv) {
    r = railSet(rail, g.uv, o.uv);
    if (r) return r;
    g.uv = o.uv;
  }
  return 0;
}

}  // namespace

void gpuGovernorRun(const Rk3588DvfsRail &rail, bool apply) {
  unsigned minMhz = 0, maxMhz = 800, boostMs = 200;
  PE_parse_boot_argn("rk3588-gpu-min", &minMhz, sizeof minMhz);
  PE_parse_boot_argn("rk3588-gpu-max", &maxMhz, sizeof maxMhz);
  PE_parse_boot_argn("rk3588-gpu-boost-ms", &boostMs, sizeof boostMs);
  unsigned floorLevel = 0, ceiling = kOpps - 1;
  while (floorLevel + 1 < kOpps && kOpp[floorLevel].setting < minMhz) ++floorLevel;
  while (ceiling > floorLevel && kOpp[ceiling].setting > maxMhz) --ceiling;
  GpuState g;
  if (voltageGet(rail, &g.uv)) {
    printf("rk3588-dvfs: gpu governor: the rail does not answer\n");
    return;
  }
  uint64_t boostAbs = 0;
  nanoseconds_to_absolutetime(uint64_t(boostMs) * 1000000, &boostAbs);
  printf("rk3588-dvfs: gpu governor: points %u..%u MHz (settings), boost %u ms, window %u ms%s\n", kOpp[floorLevel].setting, kOpp[ceiling].setting, boostMs, kWindowMs,
         apply ? "" : " (watching only)");
  // from the Mali driver's fixed 750 MHz to the top point: its voltage first (750 mV is already enough for the 800 setting)
  if (apply && gpuLevelSet(rail, g, ceiling)) printf("rk3588-dvfs: gpu governor: could not reach the top point\n");

  // watching only, the point the governor would be at is tracked as if it had been set; the clock and voltage of the model stay the Mali driver's
  if (!apply) g.current = ceiling;
  const auto currentMhz = [&] { return apply && g.current != kNone ? kOpp[g.current].actual : kBootMhz; };
  const auto currentLevel = [&] { return g.current != kNone ? g.current : ceiling; };

  uint64_t idleAbs = 0;
  nanoseconds_to_absolutetime(uint64_t(kIdleAfterMs) * 1000000, &idleAbs);
  uint64_t windowStart = mach_absolute_time(), lastBusyAt = windowStart, boostEnd = 0, statsNs = 0;
  unsigned samples = 0, busy = 0, down = 0, changes = 0, boosts = 0, activity = rk3588_dvfs_activity();
  uint64_t residency[kOpps] = {}, busyNs = 0, mhzNs = 0, powerNowNs = 0, powerBootNs = 0;
  bool disabled = false;
  for (;;) {
    IOSleep(mach_absolute_time() - lastBusyAt > idleAbs ? kIdleSampleMs : kSampleMs);
    const uint64_t t = mach_absolute_time();
    if (boostAbs) {
      const unsigned now = rk3588_dvfs_activity();
      if (now != activity) {
        activity = now;
        if (t >= boostEnd) ++boosts;
        boostEnd = t + boostAbs;
      }
    }
    ++samples;
    if (gpuRead(kGpuStatus) & 1) {
      ++busy;
      lastBusyAt = t;
    }
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(t - windowStart, &ns);
    if (ns < uint64_t(kWindowMs) * 1000000) continue;
    windowStart = t;

    const unsigned load = busy * 1000 / samples;
    samples = busy = 0;
    unsigned top = ceiling;
    for (unsigned step = governorThrottleSteps(); step && top > floorLevel; --step) --top;
    const unsigned cur = currentLevel();
    unsigned want = top;
    if (load < kHispeedPermille && t >= boostEnd) want = lowestAtLeast(floorLevel, top, uint64_t(kOpp[cur].actual) * load / kTargetPermille);
    unsigned target = cur;
    if (cur > top) {
      target = top;   // too hot: down now
      down = 0;
    } else if (want > cur) {
      target = want;
      down = 0;
    } else if (want < cur) {
      if (++down >= kDownWindows) {
        target = want;
        down = 0;
      }
    } else {
      down = 0;
    }
    if (apply && !disabled && target != cur) {
      const int r = gpuLevelSet(rail, g, target);
      ++changes;
      if (r && r != 30 && ++g.failures >= kMaxFailures) {
        printf("rk3588-dvfs: gpu governor: error %d, back to the top point and left alone\n", r);
        gpuLevelSet(rail, g, ceiling);
        disabled = true;
      }
    } else if (!apply && target != g.current) {
      g.current = target;   // as if set
      ++changes;
    }

    // the model: the GPU's dynamic power is C f V^2 while it works (busy share `load`); the same work at the boot setting takes load * f / f_boot of the time
    const uint64_t mv = apply ? g.uv / 1000 : kBootUv / 1000, mhzCur = currentMhz();
    const uint64_t work = uint64_t(load) * mhzCur * kGpuCoefficient;
    powerNowNs += work * mv * mv / 1000000000 * ns / 1000;
    powerBootNs += work * (kBootUv / 1000) * (kBootUv / 1000) / 1000000000 * ns / 1000;
    busyNs += uint64_t(load) * ns / 10;
    mhzNs += mhzCur * ns;
    residency[currentLevel()] += ns;
    statsNs += ns;
    if (statsNs >= kStatsNs) {
      char line[160] = {};
      unsigned used = 0;
      for (unsigned level = 0; level < kOpps && used + 14 < sizeof line; ++level) {
        const unsigned percent = unsigned(residency[level] * 100 / statsNs);
        if (percent) used += unsigned(snprintf(line + used, sizeof line - used, " %u:%u%%", kOpp[level].setting, percent));
      }
      printf("rk3588-dvfs: gov gpu: %u changes, mean %u MHz, busy %u%%, %u mV;%s; boosts %u\n", changes, unsigned(mhzNs / statsNs), unsigned(busyNs / statsNs),
             unsigned(apply ? g.uv / 1000 : kBootUv / 1000), line, boosts);
      printf("rk3588-dvfs: gov gpu power: model dynamic power %u mW, the same work at the boot setting (%u MHz, %u mV) %u mW (%d%% less)\n", unsigned(powerNowNs / statsNs),
             kBootMhz, kBootUv / 1000, unsigned(powerBootNs / statsNs), powerBootNs ? int(100 - powerNowNs * 100 / powerBootNs) : 0);
      changes = 0;
      statsNs = busyNs = mhzNs = powerNowNs = powerBootNs = 0;
      for (auto &r : residency) r = 0;
    }
  }
}

}  // namespace rk3588
