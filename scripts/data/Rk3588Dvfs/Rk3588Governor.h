// SPDX-License-Identifier: BSD-2-Clause-Patent
#pragma once
#include "Rk3588Domain.h"

namespace rk3588 {

// The CPU frequency governor. Runs on the calling thread and does not return. `domains` are set up (domainSetUp) and stay in use.
void governorRun(Domain *domains, unsigned count);

}  // namespace rk3588
