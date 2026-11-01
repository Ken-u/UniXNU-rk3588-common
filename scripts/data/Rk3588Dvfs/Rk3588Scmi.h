// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Scmi: the SCMI client the RK3588 firmware (BL31) offers over an SMC (ARM DEN0056). The device tree of every RK3588 board
// has `/firmware/scmi { compatible = "arm,scmi-smc"; arm,smc-id = <0x82000010>; shmem = <&scmi_shmem>; }` with the shared memory at
// 0x10f000 (0x100 bytes). BL31 owns the clocks of the CPU clusters, the DSU, the DDR, the GPU and the NPU behind it.
//
// The calls are synchronous and serialised by one lock; they sleep, so a caller must be a thread. Nothing here changes a voltage:
// that is the board's regulator hook (Rk3588Dvfs.h).
#pragma once
#include <stdint.h>

// Clock ids of the firmware (dt-bindings/clock/rockchip,rk3588-cru.h, SCMI_CLK_*).
enum {
  kRk3588ScmiClkCpuL = 0,    // the four Cortex-A55 cores
  kRk3588ScmiClkDsu = 1,
  kRk3588ScmiClkCpuB01 = 2,  // the Cortex-A76 cores 0 and 1 (MPIDR 0x400, 0x500)
  kRk3588ScmiClkCpuB23 = 3,  // the Cortex-A76 cores 2 and 3 (MPIDR 0x600, 0x700)
  kRk3588ScmiClkDdr = 4,
  kRk3588ScmiClkGpu = 5,
  kRk3588ScmiClkNpu = 6,
};

#ifdef __cplusplus
extern "C" {
#endif

// Maps the shared memory and allocates the lock. 0 on success; repeatable.
int rk3588_scmi_init(void);

// One command of `protocol`. `tx` / `rx` are payload words (little endian), `rx_words` receives the response payload size (the
// status word excluded), `status` the SCMI status of the firmware (0 success, -1 not supported, -2 invalid parameters ...).
// Returns 0 when the exchange completed (whatever the status), negative when the channel failed (-1 busy, -2 SMC refused, -3 token or
// length mismatch).
int rk3588_scmi_call(uint8_t protocol, uint8_t message, const uint32_t *tx, unsigned tx_words, uint32_t *rx, unsigned rx_capacity,
                     unsigned *rx_words, int32_t *status);

// Base protocol: the firmware's vendor strings and implementation version.
int rk3588_scmi_base(char vendor[17], char sub_vendor[17], uint32_t *implementation);

// Clock protocol (0x14). Each returns 0 on success.
int rk3588_scmi_clock_version(uint32_t *version);
int rk3588_scmi_clock_count(unsigned *clocks);
int rk3588_scmi_clock_attributes(uint32_t id, char name[17], uint32_t *attributes);
int rk3588_scmi_clock_rate_get(uint32_t id, uint64_t *hz);
int rk3588_scmi_clock_rate_set(uint32_t id, uint64_t hz);
// The discrete rates of the clock in ascending order of the firmware's list, at most `capacity`; *count receives the number returned,
// *ranged is set when the firmware describes a range (min, max, step) instead of a list (then rates[0..2] are those three).
int rk3588_scmi_clock_rates(uint32_t id, uint64_t *rates, unsigned capacity, unsigned *count, int *ranged);

#ifdef __cplusplus
}
#endif
