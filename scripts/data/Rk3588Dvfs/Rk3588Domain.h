// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The CPU clusters as frequency domains: operating points (rate and voltage), and the change from one point to another with the voltage
// and the clock ordered so that the cores never run faster than their voltage allows.
//
// The Cortex-A55 clock also drives the DSU (the shared cache and its snoop logic, which every cluster's memory traffic passes through):
// the clock service sets the DSU to the Cortex-A55 rate whenever that is set, and takes a rate for the DSU on its own afterwards. The DSU
// is supplied by the Cortex-A55 rail, so on that domain the rail's voltage follows the faster of the two.
#pragma once
#include <stdint.h>
#include "Rk3588Dvfs.h"

namespace rk3588 {

struct Opp {
  uint32_t mhz, uv;
};

struct Domain {
  const Rk3588DvfsRail *rail = nullptr;
  const char *name = "";
  int cpu = -1;          // the first logical CPU of the cluster
  unsigned cpus = 0;     // how many
  Opp level[16] = {};    // the table's points the firmware also offers, ascending
  unsigned count = 0;
  unsigned current = 0;      // index of the point the cores run at
  unsigned dsuCurrent = 0;   // the point the DSU runs at (the Cortex-A55 domain only)
  uint32_t uv = 0;           // the voltage last written or read
  bool hasDsu() const { return rail && rail->clock == kRk3588ScmiClkCpuL; }
};

inline unsigned mhz(uint64_t hz) { return unsigned((hz + 500000) / 1000000); }

int voltageGet(const Rk3588DvfsRail &rail, uint32_t *uv);

// Builds the point list of one rail from the firmware's rate list and reads the voltage and rate it stands at.
bool domainSetUp(Domain &d, const Rk3588DvfsRail &rail, int cpu, unsigned cpus, const char *name);

// Moves the domain to point `to` and, on the domain that has the DSU, the DSU to point `dsu` (at least `to`). Voltage first when the faster
// of the two points rises, clock first when it falls. 0 on success; the clock service must read back the rate asked for (a mismatch
// leaves `current` at what it reports and returns 30).
int levelSet(Domain &d, unsigned to, unsigned dsu);
inline int levelSet(Domain &d, unsigned to) { return levelSet(d, to, to); }

}  // namespace rk3588
