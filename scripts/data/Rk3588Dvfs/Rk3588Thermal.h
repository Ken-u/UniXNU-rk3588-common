// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The RK3588's temperature sensors (the TSADC at 0xfec00000, seven channels), read by polling. Nothing here arms the controller's own
// over-temperature shutdown: only the sampling is switched on.
#pragma once
#include <stdint.h>

namespace rk3588 {

enum ThermalChannel {
  kThermalSoc = 0,
  kThermalBig0 = 1,    // the Cortex-A76 cores 0 and 1
  kThermalBig1 = 2,    // the Cortex-A76 cores 2 and 3
  kThermalLittle = 3,  // the Cortex-A55 cores
  kThermalCenter = 4,
  kThermalGpu = 5,
  kThermalNpu = 6,
  kThermalChannels = 7,
};

// Maps the controller and, when the firmware left its clocks stopped or it in reset, brings it up; then switches the sampling on. 0 on success.
int thermalInit();

// The temperature of a channel in millidegrees Celsius; 0 on success, -1 when the sensor has not produced a usable code.
int thermalRead(unsigned channel, int *milliCelsius);

// The mean of the SoC, CPU, centre and GPU channels (the NPU's is left out: nothing runs on it), millidegrees; INT32_MIN when none reads.
int thermalMean();

// The hottest of the CPU, SoC-centre and GPU channels, millidegrees; INT32_MIN when none reads.
int thermalHottest();

}  // namespace rk3588
