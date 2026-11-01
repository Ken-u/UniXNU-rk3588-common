// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Pmic: access to the regulators the RK3588 boards use for the CPU clusters and the GPU, polled, from a thread:
//   * the Rockchip RK806 PMIC on SPI2 (0xfeb20000, chip select 0): bucks 1..10; the boards put the Cortex-A55 rail (DCDC_REG2) and the
//     GPU rail (DCDC_REG1) there;
//   * the Rockchip RK8602 / RK8603 bucks on I2C0 (0xfd880000) and I2C1 (0xfea90000): the Cortex-A76 rails on the boards that use them.
// The controllers are set up by the firmware (pins, clocks, speed); nothing here routes a pin or opens a clock gate. Which rail feeds
// what, and the limits, are the board's: see Rk3588Dvfs.h.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// RK806 (SPI2, chip select 0). 0 on success, negative on a timeout.
int rk3588_rk806_read(uint8_t reg, uint8_t *value);
int rk3588_rk806_write(uint8_t reg, uint8_t value);
// Buck `n` (1..10): the voltage of the ON selector in microvolts; the setter rounds up to the selector step and reads the selector back.
int rk3588_rk806_buck_get_uv(unsigned n, uint32_t *microvolts);
int rk3588_rk806_buck_set_uv(unsigned n, uint32_t microvolts);

// RK860x on I2C bus 0 or 1 at a 7 bit address. `id` receives the die id (8: RK8600/01, 10: RK8602/03).
int rk3588_rk860x_get_uv(unsigned bus, uint8_t address, uint32_t *microvolts, unsigned *id);
int rk3588_rk860x_set_uv(unsigned bus, uint8_t address, uint32_t microvolts);

// Raw access for the probe.
int rk3588_i2c_read_reg(unsigned bus, uint8_t address, uint8_t reg, uint8_t *value);
// What the firmware left in the I2C controller's CLKDIV register and in SPI2's registers (CTRLR0, BAUDR, SER, SR), for the log.
int rk3588_pmic_state(unsigned bus, uint32_t *clkdiv, uint32_t spi[4]);

#ifdef __cplusplus
}
#endif
