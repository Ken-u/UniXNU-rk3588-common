// SPDX-License-Identifier: BSD-2-Clause-Patent
#include <IOKit/IOLib.h>
#include "Rk3588Domain.h"
#include "Rk3588Pmic.h"
#include "Rk3588Scmi.h"

namespace rk3588 {
namespace {

// The operating points of the firmware's clock service with the voltage each needs, from edk2-rockchip's CpuPerformance.c (Rockchip's
// vendor tables for a typical die; a better die could run on less). The firmware lists 600 MHz for the Cortex-A55 cores only, and neither
// 2256 nor 2352 MHz for the Cortex-A76 cores.
constexpr Opp kLittleOpp[] = {{408, 675000},  {600, 675000},  {816, 675000},  {1008, 675000},
                              {1200, 712500}, {1416, 762500}, {1608, 850000}, {1800, 950000}};
constexpr Opp kBigOpp[] = {{408, 675000},  {816, 675000},   {1008, 675000},  {1200, 675000}, {1416, 725000}, {1608, 762500},
                           {1800, 850000}, {2016, 925000},  {2208, 987500},  {2304, 1000000}, {2400, 1000000}};

// Writes a voltage inside the rail's limits and waits for the regulator's ramp when it goes up.
int voltageSet(Domain &d, uint32_t uv) {
  const Rk3588DvfsRail &rail = *d.rail;
  if (uv < rail.min_uv || uv > rail.max_uv) return -10;
  const int r = rail.kind == kRk3588DvfsRk806 ? rk3588_rk806_buck_set_uv(rail.index, uv) : rk3588_rk860x_set_uv(rail.index, rail.address, uv);
  if (r) return r;
  if (uv > d.uv) IODelay((uv - d.uv) / (rail.slew_uv_per_us ? rail.slew_uv_per_us : 1) + 30);
  d.uv = uv;
  return 0;
}

}  // namespace

int voltageGet(const Rk3588DvfsRail &rail, uint32_t *uv) {
  unsigned id = 0;
  return rail.kind == kRk3588DvfsRk806 ? rk3588_rk806_buck_get_uv(rail.index, uv) : rk3588_rk860x_get_uv(rail.index, rail.address, uv, &id);
}

bool domainSetUp(Domain &d, const Rk3588DvfsRail &rail, int cpu, unsigned cpus, const char *name) {
  const bool little = rail.clock == kRk3588ScmiClkCpuL;
  const Opp *table = little ? kLittleOpp : kBigOpp;
  const unsigned entries = little ? sizeof kLittleOpp / sizeof kLittleOpp[0] : sizeof kBigOpp / sizeof kBigOpp[0];
  uint64_t rates[40] = {};
  unsigned count = 0;
  int ranged = 0;
  if (rk3588_scmi_clock_rates(rail.clock, rates, 40, &count, &ranged) || ranged || !count) return false;
  d.rail = &rail;
  d.name = name;
  d.cpu = cpu;
  d.cpus = cpus;
  d.count = 0;
  for (unsigned i = 0; i < entries; ++i)
    for (unsigned j = 0; j < count; ++j)
      if (mhz(rates[j]) == table[i].mhz && d.count < 16) d.level[d.count++] = table[i];
  uint64_t now = 0;
  if (!d.count || rk3588_scmi_clock_rate_get(rail.clock, &now) || voltageGet(rail, &d.uv)) return false;
  d.current = d.count - 1;
  for (unsigned i = 0; i < d.count; ++i)
    if (d.level[i].mhz <= mhz(now)) d.current = i;
  d.dsuCurrent = d.current;
  if (d.hasDsu()) {
    uint64_t dsu = 0;
    if (!rk3588_scmi_clock_rate_get(kRk3588ScmiClkDsu, &dsu))
      for (unsigned i = 0; i < d.count; ++i)
        if (d.level[i].mhz <= mhz(dsu) + 12) d.dsuCurrent = i;   // the service reads 396 and 594 for 408 and 600
  }
  return true;
}

int levelSet(Domain &d, unsigned to, unsigned dsu) {
  if (!d.hasDsu() || dsu < to) dsu = to;
  // the point the voltage has to suit: the cores', and on the domain that has the DSU the faster of the two
  const bool shared = d.hasDsu();
  const unsigned before = shared && d.dsuCurrent > d.current ? d.dsuCurrent : d.current, after = to > dsu ? to : dsu;
  int r = 0;
  if (after > before) {
    r = voltageSet(d, d.level[after].uv > d.uv ? d.level[after].uv : d.uv);
    if (r) return r;
  }
  if (to != d.current) {
    r = rk3588_scmi_clock_rate_set(d.rail->clock, uint64_t(d.level[to].mhz) * 1000000);
    if (r) return 20 + r;
    uint64_t got = 0;
    rk3588_scmi_clock_rate_get(d.rail->clock, &got);
    d.current = to;
    if (d.hasDsu()) d.dsuCurrent = to;   // the service sets the DSU along with the cores
    if (mhz(got) != d.level[to].mhz) {
      // the service rounded: the point in force is the one it reports, and the voltage of the faster point is still in place
      for (unsigned i = 0; i < d.count; ++i)
        if (d.level[i].mhz == mhz(got)) d.current = i;
      if (d.hasDsu()) d.dsuCurrent = d.current;
      return 30;
    }
  }
  if (d.hasDsu() && d.dsuCurrent != dsu) {
    r = rk3588_scmi_clock_rate_set(kRk3588ScmiClkDsu, uint64_t(d.level[dsu].mhz) * 1000000);
    if (r) return 40 + r;
    d.dsuCurrent = dsu;
  }
  if (after < before) {
    r = voltageSet(d, d.level[after].uv);
    if (r) return r;
  }
  return 0;
}

}  // namespace rk3588
