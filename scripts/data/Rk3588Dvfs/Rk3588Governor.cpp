// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The CPU frequency governor of the RK3588 boards. Every 16 ms a cluster's busy share is the share of the window its busiest CPU was not idle (the
// idle time comes from XNU's recount, through rk3588_cpu_idle_mach of patches/xnu/rk3588-cpu-idle.patch).
//
//  * The cluster gets the lowest point at or above 1.25 x busy x its current rate (schedutil's margin), and its top point once a CPU is 90 % busy
//    (a hispeed jump). It goes up at once and down only after 4 windows in a row that asked for less.
//  * The Cortex-A76 clusters do not go below half their top rate unless the busiest CPU stayed under 10 % for 12 windows, and a touch (the board's
//    rk3588_dvfs_activity moved) holds them at the top for rk3588-dvfs-boost-ms, so that an app launch or a scroll starts at full speed.
//  * The DSU, which every cluster's cache traffic passes through, follows the faster Cortex-A76 cluster at three quarters of its rate (a load from the
//    L3 takes 18 ns with the DSU at 1800 MHz and 52 ns at 400 MHz), and the Cortex-A55 domain's voltage follows the faster of its cores and the DSU.
//  * A cluster whose clock service, regulator or bus fails five times is put back at its top point and left alone.
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include "Rk3588Governor.h"

extern "C" uint64_t rk3588_cpu_idle_mach(int cpu);

namespace rk3588 {
namespace {

constexpr unsigned kWindowMs = 16;
constexpr unsigned kDownWindows = 4, kQuietWindows = 12, kMaxFailures = 5;
constexpr uint64_t kHispeedPermille = 900, kQuietPermille = 100;
constexpr uint64_t kStatsNs = 10000000000ull;

struct State {
  unsigned down = 0, quiet = 0, failures = 0;
  bool disabled = false;
  uint32_t changes = 0;
  uint64_t mhzNs = 0, busyNs = 0, residency[16] = {};
};

void lowestAtLeast(const Domain &d, uint64_t mhzNeeded, unsigned *index) {
  unsigned i = 0;
  while (i + 1 < d.count && d.level[i].mhz < mhzNeeded) ++i;
  *index = i;
}

}  // namespace

void governorRun(Domain *domains, unsigned count) {
  unsigned floorMhz = 0, boostMs = 1000;
  PE_parse_boot_argn("rk3588-dvfs-min", &floorMhz, sizeof floorMhz);
  PE_parse_boot_argn("rk3588-dvfs-boost-ms", &boostMs, sizeof boostMs);
  uint64_t boostAbs = 0;
  nanoseconds_to_absolutetime(uint64_t(boostMs) * 1000000, &boostAbs);

  State state[3];
  Domain *little = nullptr;
  unsigned bigs = 0;
  Domain *big[2] = {};
  for (unsigned i = 0; i < count && i < 3; ++i) {
    if (domains[i].hasDsu()) little = &domains[i];
    else if (bigs < 2) big[bigs++] = &domains[i];
  }
  unsigned floorLevel[3] = {};   // the boot-arg floor as the index of the lowest point at or above it
  for (unsigned i = 0; i < count && i < 3; ++i) {
    unsigned at = 0;
    lowestAtLeast(domains[i], floorMhz, &at);
    floorLevel[i] = at;
  }
  printf("rk3588-dvfs: governor: %u domains, floor %u MHz, boost %u ms, window %u ms\n", count, floorMhz, boostMs, kWindowMs);

  uint64_t lastIdle[16] = {};
  unsigned cpusSeen = unsigned(ml_get_max_cpu_number()) + 1;
  if (cpusSeen > 16) cpusSeen = 16;
  for (unsigned c = 0; c < cpusSeen; ++c) lastIdle[c] = rk3588_cpu_idle_mach(int(c));
  uint64_t lastT = mach_absolute_time(), boostEnd = 0, windowNs = 0, dsuMhzNs = 0;
  unsigned activity = rk3588_dvfs_activity(), boosts = 0;

  for (;;) {
    IOSleep(kWindowMs);
    const uint64_t t = mach_absolute_time(), dt = t - lastT;
    lastT = t;
    if (!dt) continue;
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(dt, &ns);
    windowNs += ns;

    if (boostAbs) {
      const unsigned now = rk3588_dvfs_activity();
      if (now != activity) {
        activity = now;
        if (t >= boostEnd) ++boosts;
        boostEnd = t + boostAbs;
      }
    }
    const bool boost = t < boostEnd;

    // the busy share of each CPU, per mille; a domain's is its busiest CPU's
    uint64_t busyCpu[16] = {};
    for (unsigned c = 0; c < cpusSeen; ++c) {
      const uint64_t idle = rk3588_cpu_idle_mach(int(c));
      if (idle == ~0ULL) continue;   // not running
      const uint64_t di = idle - lastIdle[c];
      lastIdle[c] = idle;
      busyCpu[c] = di >= dt ? 0 : (dt - di) * 1000 / dt;
    }

    unsigned want[3] = {};
    unsigned bigMhz = 0;
    for (unsigned i = 0; i < count && i < 3; ++i) {
      Domain &d = domains[i];
      State &s = state[i];
      if (s.disabled) continue;
      uint64_t busy = 0;
      for (unsigned c = d.cpu; c < d.cpu + d.cpus && c < cpusSeen; ++c)
        if (busyCpu[c] > busy) busy = busyCpu[c];
      const unsigned top = d.count - 1;
      unsigned w = top;
      if (busy < kHispeedPermille) {
        lowestAtLeast(d, uint64_t(d.level[d.current].mhz) * busy * 125 / 100000, &w);
      }
      if (w < floorLevel[i]) w = floorLevel[i];
      if (!d.hasDsu()) {
        s.quiet = busy < kQuietPermille ? s.quiet + 1 : 0;
        if (boost) {
          w = top;
        } else if (s.quiet < kQuietWindows) {
          while (w + 1 < d.count && d.level[w].mhz * 2 < d.level[top].mhz) ++w;   // not under half the top rate while there is work
        }
      }
      want[i] = w;
      s.busyNs += busy * ns / 10;
    }

    // the Cortex-A76 clusters first: the DSU follows what they settle at
    for (unsigned b = 0; b < bigs; ++b) {
      Domain &d = *big[b];
      const unsigned i = unsigned(&d - domains);
      State &s = state[i];
      if (s.disabled) continue;
      unsigned target = d.current;
      if (want[i] > d.current) {
        target = want[i];
        s.down = 0;
      } else if (want[i] < d.current) {
        if (++s.down >= kDownWindows) {
          target = want[i];
          s.down = 0;
        }
      } else {
        s.down = 0;
      }
      if (target != d.current) {
        const int r = levelSet(d, target);
        ++s.changes;
        if (r && r != 30 && ++s.failures >= kMaxFailures) {
          printf("rk3588-dvfs: governor: %s: error %d, back to the top point and left alone\n", d.name, r);
          levelSet(d, d.count - 1);
          s.disabled = true;
        }
      }
      if (d.level[d.current].mhz > bigMhz) bigMhz = d.level[d.current].mhz;
    }

    if (little) {
      const unsigned i = unsigned(little - domains);
      State &s = state[i];
      if (!s.disabled) {
        unsigned dsuWant = 0;
        lowestAtLeast(*little, boost ? 100000 : uint64_t(bigMhz) * 3 / 4, &dsuWant);
        unsigned target = little->current;
        if (want[i] > little->current) {
          target = want[i];
          s.down = 0;
        } else if (want[i] < little->current) {
          if (++s.down >= kDownWindows) {
            target = want[i];
            s.down = 0;
          }
        } else {
          s.down = 0;
        }
        if (target != little->current || dsuWant != little->dsuCurrent) {
          const int r = levelSet(*little, target, dsuWant);
          if (target != little->current || r) ++s.changes;
          if (r && r != 30 && ++s.failures >= kMaxFailures) {
            printf("rk3588-dvfs: governor: %s: error %d, back to the top point and left alone\n", little->name, r);
            levelSet(*little, little->count - 1);
            s.disabled = true;
          }
        }
        dsuMhzNs += uint64_t(little->level[little->dsuCurrent].mhz) * ns;
      }
    }

    for (unsigned i = 0; i < count && i < 3; ++i) {
      state[i].mhzNs += uint64_t(domains[i].level[domains[i].current].mhz) * ns;
      state[i].residency[domains[i].current] += ns;
    }

    if (windowNs >= kStatsNs) {
      for (unsigned i = 0; i < count && i < 3; ++i) {
        State &s = state[i];
        char residency[160] = {};
        unsigned used = 0;
        for (unsigned level = 0; level < domains[i].count && used + 14 < sizeof residency; ++level) {
          const unsigned percent = unsigned(s.residency[level] * 100 / windowNs);
          if (percent) used += unsigned(snprintf(residency + used, sizeof residency - used, " %u:%u%%", domains[i].level[level].mhz, percent));
        }
        printf("rk3588-dvfs: gov %s: %u changes, mean %u MHz, busy %u%%, %u mV;%s\n", domains[i].name, s.changes, unsigned(s.mhzNs / windowNs),
               unsigned(s.busyNs / windowNs), domains[i].uv / 1000, residency);
        s.changes = 0;
        s.mhzNs = s.busyNs = 0;
        for (auto &r : s.residency) r = 0;
      }
      printf("rk3588-dvfs: gov dsu: mean %u MHz; boosts %u\n", unsigned(dsuMhzNs / windowNs), boosts);
      windowNs = 0;
      dsuMhzNs = 0;
    }
  }
}

}  // namespace rk3588
