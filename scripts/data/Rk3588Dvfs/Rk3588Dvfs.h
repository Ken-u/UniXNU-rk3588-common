// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Dvfs: frequency and voltage management shared by the RK3588 boards.
//
// What is shared (this directory): the firmware's clock service (Rk3588Scmi.h), the regulator controllers (Rk3588Pmic.h), the operating
// point tables of the Cortex-A55 and Cortex-A76 clusters and the logic that orders voltage and clock changes.
// What a board supplies: which regulator feeds which cluster, and its limits, as the table `rk3588_dvfs_rails` in a source file of its own
// that `build-rk3588-dvfs.py` compiles next to these (the board lists the file as an argument of that archive step).
//
// Why the voltage is part of it: above 600 MHz the firmware clocks the cores from a PVTPLL, a ring oscillator whose speed follows the
// supply. The rate the service reports is only the setting of that oscillator. At the 1.0 V the firmware leaves on the Cortex-A76 rails
// every setting from 1608 MHz up runs at about 2230 MHz, and 816 MHz runs at about 1220 MHz; the nominal clock is only reached at the
// operating point's own voltage.
#pragma once
#include <stdint.h>
#include "Rk3588Scmi.h"

enum Rk3588DvfsRegulator {
  kRk3588DvfsRk806 = 1,    // RK806 bucks, SPI2 chip select 0: `index` is the buck number (1..10)
  kRk3588DvfsRk860x = 2,   // RK8602 / RK8603: `index` is the I2C bus (0, 1), `address` the 7 bit address
};

struct Rk3588DvfsRail {
  uint32_t clock;               // the SCMI clock this rail feeds (kRk3588ScmiClkCpuL, kRk3588ScmiClkCpuB01, ...)
  Rk3588DvfsRegulator kind;
  uint8_t index;
  uint8_t address;
  uint32_t min_uv, max_uv;      // the rail's limits; a voltage outside them is never written
  uint32_t slew_uv_per_us;      // how fast the regulator ramps, for the wait after a change
};

#ifdef __cplusplus
extern "C" {
#endif

// The board's table (one entry per rail it wants managed) and its length.
extern const struct Rk3588DvfsRail rk3588_dvfs_rails[];
extern const unsigned rk3588_dvfs_rail_count;

// The board's input activity: a counter that moves whenever the user touches or presses something (0 for a board without an input device).
// The governor lifts the Cortex-A76 clusters to their top point for a while when it moves, so that an app launch or a scroll starts at full speed
// instead of climbing from the lowest point.
unsigned rk3588_dvfs_activity(void);

// Called by the board's probe once the kernel is up; reads the boot-arguments (rk3588-dvfs, ...).
void rk3588_dvfs_start(void);

#ifdef __cplusplus
}
#endif
