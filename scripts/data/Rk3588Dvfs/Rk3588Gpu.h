// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The Mali-G610's clock and activity, as seen from the frequency management: what the GPU runs at, whether it is busy, and a look at what the
// firmware's clock service (SCMI clock 5) does to the clock source when it is asked for another rate. The GPU itself belongs to the Mali driver
// (UniXNU-evb10 package/Mali): this only reads its status registers and the clock registers, once the driver has the power domain on.
#pragma once
#include <stdint.h>
#include "Rk3588Dvfs.h"

namespace rk3588 {

// Waits (up to `waitMs`) for the GPU's power domain to be on with its clocks open and the GPU to answer with its id; maps the registers.
// False when the Mali driver did not bring the GPU up (mali=0 or a failed start): nothing is touched then.
bool gpuAttach(unsigned waitMs);

// The clock the GPU really runs at, in MHz: its cycle counter against the architected counter over about 5 ms. 0 when the GPU is not attached.
unsigned gpuMeasureMhz();

// Mode 8: with the rail at its limit voltage, every rate of the clock service in turn: what the service reports, what it leaves in the clock
// registers and what the cycle counter measures; ends with the clock registers and the rail as they were.
void gpuSweep(const Rk3588DvfsRail &rail);

// Mode 10: for each PVTPLL setting of the clock service (900 .. 300 MHz) the rail walks down from 850 mV in 12.5 mV steps to 675 mV and the cycle counter says
// what the clock really runs at there (the PVTPLL follows the supply: the setting is only nominal at the operating point's own voltage); ends as mode 8.
void gpuVoltageSweep(const Rk3588DvfsRail &rail);

// Mode 9: the GPU's busy share (GPU_STATUS) and whether its shader cores are powered, sampled in 50 ms windows on this thread; does not return.
void gpuWatch();

// The GPU governor (mode 6, and mode 7 watching only): samples GPU_STATUS about every millisecond (every 8 ms after a second without work), and every
// ~25 ms picks the operating point from the busy share: its top point above 40 % busy, otherwise the lowest that would keep the GPU about 30 % busy;
// up at once, down after 3 windows; a touch (rk3588_dvfs_activity) holds it at the top for rk3588-gpu-boost-ms. Rate and voltage are set together, the
// voltage first on the way up and last on the way down. Runs on the calling thread and does not return.
// Boot-args: rk3588-gpu=0 leaves the GPU as the Mali driver set it (750 MHz from CPLL at 750 mV), rk3588-gpu-min / rk3588-gpu-max=MHz the lowest and highest
// setting of the clock service it may use (default 0 and 800), rk3588-gpu-boost-ms (default 200).
void gpuGovernorRun(const Rk3588DvfsRail &rail, bool apply);

}  // namespace rk3588
