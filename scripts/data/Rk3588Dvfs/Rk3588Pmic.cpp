// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Pmic: see Rk3588Pmic.h.
//
// The I2C master is the polled one of U-Boot's rk_i2c.c (GPL-2.0+, Rockchip) rewritten for the kernel, as in package/Evb10Audio of the
// EVB10 repository, with the controller's base as a member. The SPI master follows the register use of edk2-rockchip's SpiLib.c and
// RK806.c (BSD-2-Clause-Patent, Copyright (c) 2023 Mario Balanica and Rockchip): a 3 byte header (read 0x00 / write 0x80, register, 0),
// the register written as a fourth byte or read back in a second, receive-only transfer with the chip select held by SER. The RK860x
// registers and voltage encodings are those of edk2-rockchip's Rk860xRegulatorDxe.c.
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <stdint.h>
#include <string.h>
#include "Rk3588Pmic.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

namespace {

constexpr uint64_t kI2cBase[2] = {0xfd880000ULL, 0xfea90000ULL};
constexpr uint64_t kSpi2Base = 0xfeb20000ULL;

inline uint32_t rd(volatile uint32_t *base, uint32_t off) { return base[off / 4]; }
inline void wr(volatile uint32_t *base, uint32_t off, uint32_t value) { base[off / 4] = value; }

uint64_t nowUs() {
  uint64_t ns;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000;
}

uint32_t divUp(uint64_t a, uint64_t b) { return static_cast<uint32_t>((a + b - 1) / b); }

IOLock *gLock;
bool initialised() {
  if (!gLock) gLock = IOLockAlloc();
  return gLock != nullptr;
}

// ---------------------------------------------------------------------------------------------------------------- I2C

// I2C registers (rk3399-style controller)
enum : uint32_t {
  kCon = 0x00, kClkDiv = 0x04, kMrxAddr = 0x08, kMrxRaddr = 0x0c, kMtxCnt = 0x10, kMrxCnt = 0x14, kIen = 0x18, kIpd = 0x1c,
  kTxData = 0x100, kRxData = 0x200,
};
enum : uint32_t {
  kConEn = 1u << 0, kConStart = 1u << 3, kConStop = 1u << 4, kConLastAck = 1u << 5, kConActAck = 1u << 6,
  kModeTx = 0, kModeTrx = 1, kModeRx = 2,
  kIntMbtf = 1u << 2, kIntMbrf = 1u << 3, kIntStart = 1u << 4, kIntStop = 1u << 5, kIntNak = 1u << 6, kIntAll = 0x7f,
};
constexpr unsigned kFifo = 32;

struct I2c {
  volatile uint32_t *base = nullptr;
  unsigned cfg = 0;
  static constexpr uint64_t kTimeoutUs = 100000;

  bool map(unsigned bus) {
    if (base) return true;
    base = reinterpret_cast<volatile uint32_t *>(ml_io_map(kI2cBase[bus], 0x1000));
    return base != nullptr;
  }

  void disable() {
    wr(base, kIen, 0);
    wr(base, kIpd, kIntAll);
    wr(base, kCon, 0);
  }

  // The firmware's divider is kept (it ran this bus at 100 kHz for the same regulators); a controller it never used gets U-Boot's
  // rk_i2c_adapter_clk values for a 100 kHz bus from a 100 MHz clock.
  void setSpeed() {
    cfg = (1u << 8) | (1u << 12);  // SDA_CFG(1) | STA_CFG(1)
    if (rd(base, kClkDiv)) return;
    const uint32_t clkRate = 100000000, sclRate = 100000;
    const uint64_t rateKhz = divUp(clkRate, 1000), sclKhz = divUp(sclRate, 1000);
    const uint32_t minTotal = divUp(rateKhz, sclKhz * 8);
    uint32_t minHigh = divUp(rateKhz * (1000 + 4000), 8ULL * 1000000);
    uint32_t minLow = divUp(rateKhz * (300 + 4700), 8ULL * 1000000);
    minHigh = minHigh < 1 ? 2 : minHigh;
    minLow = minLow < 1 ? 2 : minLow;
    const uint32_t minHold = minHigh + minLow;
    uint32_t high, low;
    if (minHold >= minTotal) {
      high = minHigh; low = minLow;
    } else {
      const uint32_t extra = minTotal - minHold;
      const uint32_t extraLow = divUp(static_cast<uint64_t>(minLow) * extra, minHold);
      low = minLow + extraLow;
      high = minHigh + (extra - extraLow);
    }
    high--; low--;
    wr(base, kClkDiv, (high << 16) | low);
  }

  int waitIpd(uint32_t bit, uint32_t alsoNak) {
    const uint64_t began = nowUs();
    for (;;) {
      const uint32_t ipd = rd(base, kIpd);
      if ((alsoNak & kIntNak) && (ipd & kIntNak)) {
        wr(base, kIpd, kIntNak);
        return -2;  // no acknowledge
      }
      if (ipd & bit) {
        wr(base, kIpd, bit);
        return 0;
      }
      if (nowUs() - began > kTimeoutUs) return -1;
      IODelay(1);
    }
  }

  int startBit(uint32_t con) {
    wr(base, kIpd, kIntAll);
    wr(base, kIen, kIntStart);
    wr(base, kCon, kConEn | kConStart | cfg | con);
    const int r = waitIpd(kIntStart, 0);
    if (r) return r;
    wr(base, kCon, kConEn | cfg | con);  // clean the start bit
    return 0;
  }

  int stopBit() {
    wr(base, kIpd, kIntAll);
    wr(base, kCon, kConEn | cfg | kConStop);
    wr(base, kIen, kIntStop);
    const int r = waitIpd(kIntStop, 0);
    IODelay(1);
    return r;
  }

  int write(uint8_t chip, const uint8_t *buf, unsigned len) {
    unsigned remain = len + 1;
    bool first = true, next = false;
    while (remain) {
      const unsigned now = remain > kFifo ? kFifo : remain;
      const unsigned words = (now + 3) / 4;
      for (unsigned i = 0; i < words; ++i) {
        uint32_t data = 0;
        for (unsigned j = 0; j < 4; ++j) {
          if (i * 4 + j == now) break;
          if (first && i == 0 && j == 0) data |= static_cast<uint32_t>(chip) << 1;
          else data |= static_cast<uint32_t>(*buf++) << (j * 8);
        }
        wr(base, kTxData + i * 4, data);
      }
      first = false;
      if (!next) {
        const int r = startBit(kConEn | (kModeTx << 1));
        if (r) return r;
        next = true;
      } else {
        wr(base, kCon, kConEn | (kModeTx << 1) | cfg);
      }
      wr(base, kIen, kIntMbtf | kIntNak);
      wr(base, kMtxCnt, now);
      const int r = waitIpd(kIntMbtf, kIntNak);
      if (r) return r;
      remain -= now;
    }
    return 0;
  }

  int read(uint8_t chip, uint8_t *buf, unsigned len, bool snd) {
    unsigned remain = len;
    bool chunk = false;
    if (snd) wr(base, kCon, 0);
    wr(base, kMrxAddr, (1u << 24) | (static_cast<uint32_t>(chip) << 1) | 1u);
    wr(base, kMrxRaddr, 0);
    while (remain) {
      uint32_t con;
      unsigned now;
      if (remain > kFifo) {
        con = kConEn;
        now = kFifo;
      } else {
        con = kConEn | kConLastAck;
        now = remain;
      }
      const unsigned words = (now + 3) / 4;
      if (chunk) {
        con |= kModeRx << 1;
        wr(base, kCon, con | cfg);
      } else {
        con |= kModeTrx << 1;
        const int r = startBit(con);
        if (r) return r;
      }
      wr(base, kIen, kIntMbrf | kIntNak);
      wr(base, kMrxCnt, now);
      const int r = waitIpd(kIntMbrf, kIntNak);
      if (r) return r;
      for (unsigned i = 0; i < words; ++i) {
        const uint32_t data = rd(base, kRxData + i * 4);
        for (unsigned j = 0; j < 4; ++j) {
          if (i * 4 + j == now) break;
          *buf++ = static_cast<uint8_t>(data >> (j * 8));
        }
      }
      remain -= now;
      chunk = true;
    }
    return 0;
  }

  // Write, then (after a repeated start) read; always ends with a stop.
  int xfer(uint8_t chip, const uint8_t *wbuf, unsigned wlen, uint8_t *rbuf, unsigned rlen) {
    int r = 0;
    setSpeed();
    cfg |= kConActAck;
    if (wlen) r = write(chip, wbuf, wlen);
    if (!r && rlen) r = read(chip, rbuf, rlen, wlen != 0);
    stopBit();
    disable();
    return r;
  }
} gI2c[2];

int i2cRead(unsigned bus, uint8_t address, uint8_t reg, uint8_t *value) {
  if (bus > 1 || !initialised() || !gI2c[bus].map(bus)) return -1;
  IOLockLock(gLock);
  const int r = gI2c[bus].xfer(address, &reg, 1, value, 1);
  IOLockUnlock(gLock);
  return r;
}

int i2cWrite(unsigned bus, uint8_t address, uint8_t reg, uint8_t value) {
  if (bus > 1 || !initialised() || !gI2c[bus].map(bus)) return -1;
  const uint8_t bytes[2] = {reg, value};
  IOLockLock(gLock);
  const int r = gI2c[bus].xfer(address, bytes, 2, nullptr, 0);
  IOLockUnlock(gLock);
  return r;
}

// ---------------------------------------------------------------------------------------------------------------- SPI2

enum : uint32_t {
  kSpiCtrl0 = 0x00, kSpiCtrl1 = 0x04, kSpiEnable = 0x08, kSpiSer = 0x0c, kSpiBaud = 0x10, kSpiTxThreshold = 0x14, kSpiRxThreshold = 0x18,
  kSpiRxLevel = 0x20, kSpiStatus = 0x24, kSpiDmaCtrl = 0x3c, kSpiDmaTxLevel = 0x40, kSpiDmaRxLevel = 0x44, kSpiTxData = 0x400, kSpiRxData = 0x800,
};
// CTRLR0: 8 bit frames (DFS 1), clock polarity low, phase first edge, chip select hold 0, SSD one cycle, big endian, MSB first, 8 bit APB
// transform, master; the transfer mode (bits 19:18) is set per phase.
constexpr uint32_t kSpiCtrl0Base = (1u << 0) | (1u << 10) | (1u << 11) | (1u << 13);
constexpr uint32_t kSpiXferTx = 1u << 18, kSpiXferRx = 2u << 18;
constexpr unsigned kSpiFifo = 64;

volatile uint32_t *gSpi;

bool spiMap() {
  if (!gSpi) gSpi = reinterpret_cast<volatile uint32_t *>(ml_io_map(kSpi2Base, 0x1000));
  return gSpi != nullptr;
}

bool spiIdle() {
  const uint64_t began = nowUs();
  while (rd(gSpi, kSpiStatus) & 1) {
    if (nowUs() - began > 100000) return false;
    IODelay(1);
  }
  return true;
}

void spiConfigure(uint32_t transfer) {
  wr(gSpi, kSpiEnable, 0);
  wr(gSpi, kSpiCtrl0, kSpiCtrl0Base | transfer);
  wr(gSpi, kSpiTxThreshold, kSpiFifo / 2 - 1);
  wr(gSpi, kSpiRxThreshold, kSpiFifo / 2 - 1);
  wr(gSpi, kSpiDmaTxLevel, kSpiFifo / 2 - 1);
  wr(gSpi, kSpiDmaRxLevel, 0);
  wr(gSpi, kSpiBaud, 12);   // 24 MHz (XIN_OSC0, CRU_CLKSEL_CON59 as the firmware set it) / 12 = 2 MHz
}

// One RK806 register access on chip select 0.
int rk806Transfer(bool write, uint8_t reg, uint8_t *value) {
  if (!initialised() || !spiMap()) return -1;
  IOLockLock(gLock);
  int result = 0;
  uint8_t header[4] = {uint8_t(write ? 0x80 : 0x00), reg, 0x00, write ? *value : uint8_t(0)};
  wr(gSpi, kSpiSer, rd(gSpi, kSpiSer) | 1u);
  spiConfigure(kSpiXferTx);
  wr(gSpi, kSpiEnable, 1);
  for (unsigned i = 0; i < (write ? 4u : 3u); ++i) wr(gSpi, kSpiTxData, header[i]);
  if (!spiIdle()) result = -1;
  wr(gSpi, kSpiEnable, 0);
  if (!write && !result) {
    spiConfigure(kSpiXferRx);
    wr(gSpi, kSpiCtrl1, 0);   // one frame
    wr(gSpi, kSpiEnable, 1);
    const uint64_t began = nowUs();
    while (!rd(gSpi, kSpiRxLevel)) {
      if (nowUs() - began > 100000) { result = -1; break; }
      IODelay(1);
    }
    if (!result) *value = uint8_t(rd(gSpi, kSpiRxData));
    if (!spiIdle()) result = -1;
    wr(gSpi, kSpiEnable, 0);
  }
  wr(gSpi, kSpiSer, rd(gSpi, kSpiSer) & ~1u);
  IOLockUnlock(gLock);
  return result;
}

constexpr uint8_t kRk806BuckOn = 0x1a;   // RK806_BUCK_ON_VSEL(n) = 0x1a + n - 1

}  // namespace

extern "C" int rk3588_rk806_read(uint8_t reg, uint8_t *value) { return rk806Transfer(false, reg, value); }
extern "C" int rk3588_rk806_write(uint8_t reg, uint8_t value) { return rk806Transfer(true, reg, &value); }

// 500 mV + 6.25 mV per step up to selector 0xa0 (1.5 V); above that 25 mV steps, which this code does not use.
extern "C" int rk3588_rk806_buck_get_uv(unsigned n, uint32_t *microvolts) {
  uint8_t selector = 0;
  if (n < 1 || n > 10) return -1;
  const int r = rk806Transfer(false, uint8_t(kRk806BuckOn + n - 1), &selector);
  if (r) return r;
  *microvolts = selector <= 0xa0 ? 500000 + 6250u * selector : 1500000 + 25000u * (selector - 0xa0);
  return 0;
}

extern "C" int rk3588_rk806_buck_set_uv(unsigned n, uint32_t microvolts) {
  if (n < 1 || n > 10 || microvolts < 500000 || microvolts > 1500000) return -1;
  uint8_t selector = uint8_t(divUp(microvolts - 500000, 6250));
  const int r = rk806Transfer(true, uint8_t(kRk806BuckOn + n - 1), &selector);
  if (r) return r;
  uint8_t back = 0;
  const int check = rk806Transfer(false, uint8_t(kRk806BuckOn + n - 1), &back);
  return check ? check : (back == selector ? 0 : -4);
}

extern "C" int rk3588_i2c_read_reg(unsigned bus, uint8_t address, uint8_t reg, uint8_t *value) { return i2cRead(bus, address, reg, value); }

namespace {
// RK8602 / RK8603 (die id 10): VSEL0_B (0x06) holds the selector, 500 mV + 6.25 mV per step; the enable bit is in register 0x00.
// RK8600 / RK8601 (die id 8): VSEL0_A (0x00), 6 bit, 712.5 mV + 12.5 mV per step.
int rk860xInfo(unsigned bus, uint8_t address, unsigned *id, uint8_t *reg, uint8_t *mask, uint32_t *min, uint32_t *step) {
  uint8_t value = 0;
  const int r = i2cRead(bus, address, 0x03, &value);
  if (r) return r;
  *id = value & 0x0f;
  if (*id == 10) { *reg = 0x06; *mask = 0xff; *min = 500000; *step = 6250; return 0; }
  if (*id == 8) { *reg = 0x00; *mask = 0x3f; *min = 712500; *step = 12500; return 0; }
  return -3;
}
}  // namespace

extern "C" int rk3588_rk860x_get_uv(unsigned bus, uint8_t address, uint32_t *microvolts, unsigned *id) {
  uint8_t reg = 0, mask = 0, value = 0;
  uint32_t min = 0, step = 0;
  int r = rk860xInfo(bus, address, id, &reg, &mask, &min, &step);
  if (r) return r;
  r = i2cRead(bus, address, reg, &value);
  if (r) return r;
  *microvolts = min + step * (value & mask);
  return 0;
}

extern "C" int rk3588_rk860x_set_uv(unsigned bus, uint8_t address, uint32_t microvolts) {
  uint8_t reg = 0, mask = 0, value = 0;
  uint32_t min = 0, step = 0;
  unsigned id = 0;
  int r = rk860xInfo(bus, address, &id, &reg, &mask, &min, &step);
  if (r) return r;
  if (microvolts < min) return -1;
  const uint32_t selector = divUp(microvolts - min, step);
  if (selector > mask) return -1;
  // keep the enable bit and the other bits of the register the selector shares with them
  r = i2cRead(bus, address, reg, &value);
  if (r) return r;
  value = uint8_t((value & ~mask) | selector);
  r = i2cWrite(bus, address, reg, value);
  if (r) return r;
  uint8_t back = 0;
  r = i2cRead(bus, address, reg, &back);
  return r ? r : ((back & mask) == selector ? 0 : -4);
}

extern "C" int rk3588_pmic_state(unsigned bus, uint32_t *clkdiv, uint32_t spi[4]) {
  if (bus > 1 || !gI2c[bus].map(bus) || !spiMap()) return -1;
  *clkdiv = rd(gI2c[bus].base, kClkDiv);
  spi[0] = rd(gSpi, kSpiCtrl0);
  spi[1] = rd(gSpi, kSpiBaud);
  spi[2] = rd(gSpi, kSpiSer);
  spi[3] = rd(gSpi, kSpiStatus);
  return 0;
}
