// SPDX-License-Identifier: BSD-2-Clause-Patent
#pragma once
#include "Rk3588Domain.h"

namespace rk3588 {

// The CPU frequency governor. Runs on the calling thread and does not return. `domains` are set up (domainSetUp) and stay in use.
// With `apply` false it only watches: the same windows, busy shares and temperatures are logged, no point is ever changed (the firmware's clocks stay),
// which is the baseline the governor is compared with.
void governorRun(Domain *domains, unsigned count, bool apply);

// How many steps the over-temperature limit has taken off the top of the CPU clusters' points just now (0 when it is cool or the governor is only
// watching): the GPU governor lowers its own ceiling by the same number.
unsigned governorThrottleSteps();

}  // namespace rk3588
