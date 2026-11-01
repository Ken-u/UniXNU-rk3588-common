// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Scmi: SCMI over SMC for the RK3588 firmware. See Rk3588Scmi.h.
//
// Transport (ARM DEN0056, shared memory based transport, "arm,scmi-smc" of the Linux binding):
//   shmem + 0x04 channel_status   bit 0 FREE (the agent may write), bit 1 ERROR
//   shmem + 0x10 flags            bit 0 interrupt enable (left clear: the SMC returns when the response is in place)
//   shmem + 0x14 length           message header + payload, in bytes
//   shmem + 0x18 message header   id [7:0], type [9:8] (0 command), protocol [17:10], token [27:18]
//   shmem + 0x1c payload          the response starts with the int32 status
// The sender waits for FREE, clears it, writes length, header and payload and executes `smc` with the function id of the device tree
// in x0. The firmware sets FREE again when the response is complete.
#include <IOKit/IOLib.h>
#include <machine/machine_routines.h>
#include <stdint.h>
#include <string.h>
#include "Rk3588Scmi.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

namespace {

constexpr uint64_t kShmemPhys = 0x10f000ULL;
constexpr uint64_t kShmemSize = 0x100;
constexpr uint64_t kSmcFunctionId = 0x82000010ULL;
constexpr unsigned kOffStatus = 0x04, kOffFlags = 0x10, kOffLength = 0x14, kOffHeader = 0x18, kOffPayload = 0x1c;
constexpr uint32_t kStatusFree = 1, kStatusError = 2;
constexpr unsigned kPayloadWords = (kShmemSize - kOffPayload) / 4;

constexpr uint8_t kProtoBase = 0x10, kProtoClock = 0x14;
constexpr uint8_t kMsgVersion = 0x0, kMsgAttributes = 0x1;
constexpr uint8_t kBaseVendor = 0x3, kBaseSubVendor = 0x4, kBaseImplementation = 0x5;
constexpr uint8_t kClockAttributes = 0x3, kClockDescribeRates = 0x4, kClockRateSet = 0x5, kClockRateGet = 0x6;

volatile uint32_t *gShmem;
IOLock *gLock;
uint32_t gToken;

uint32_t rd(unsigned offset) { return gShmem[offset / 4]; }
void wr(unsigned offset, uint32_t value) { gShmem[offset / 4] = value; }

// SMC32 fast call. SMCCC 1.0 leaves x4..x17 unpredictable.
uint64_t smc(uint64_t function) {
  register uint64_t x0 __asm__("x0") = function;
  register uint64_t x1 __asm__("x1") = 0;
  register uint64_t x2 __asm__("x2") = 0;
  register uint64_t x3 __asm__("x3") = 0;
  __asm__ volatile("dsb sy\n\tsmc #0"
                   : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
                   :
                   : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
  return x0;
}

bool waitFree(unsigned microseconds) {
  for (unsigned i = 0; i < microseconds; ++i) {
    if (rd(kOffStatus) & kStatusFree) return true;
    IODelay(1);
  }
  return false;
}

void copyName(char name[17], const uint32_t *words) {
  memcpy(name, words, 16);
  name[16] = 0;
}

}  // namespace

extern "C" int rk3588_scmi_init(void) {
  if (gShmem && gLock) return 0;
  if (!gLock) gLock = IOLockAlloc();
  if (!gShmem) gShmem = reinterpret_cast<volatile uint32_t *>(ml_io_map(kShmemPhys, kShmemSize));
  return gShmem && gLock ? 0 : -1;
}

extern "C" int rk3588_scmi_call(uint8_t protocol, uint8_t message, const uint32_t *tx, unsigned tx_words, uint32_t *rx, unsigned rx_capacity,
                                unsigned *rx_words, int32_t *status) {
  if (rk3588_scmi_init() || tx_words > kPayloadWords) return -1;
  int result = 0;
  IOLockLock(gLock);
  do {
    if (!waitFree(100000)) { result = -1; break; }
    const uint32_t token = (++gToken) & 0x3ff;
    wr(kOffStatus, rd(kOffStatus) & ~kStatusFree);
    wr(kOffFlags, 0);
    wr(kOffLength, 4 + 4 * tx_words);
    wr(kOffHeader, uint32_t(message) | (uint32_t(protocol) << 10) | (token << 18));
    for (unsigned i = 0; i < tx_words; ++i) wr(kOffPayload + 4 * i, tx[i]);
    const uint64_t returned = smc(kSmcFunctionId);
    if (int32_t(returned) == -1) { result = -2; break; }
    if (!waitFree(100000)) { result = -1; break; }
    const uint32_t length = rd(kOffLength), header = rd(kOffHeader);
    if (((header >> 18) & 0x3ff) != token || length < 8 || length > kShmemSize - kOffLength || (rd(kOffStatus) & kStatusError)) {
      result = -3;
      break;
    }
    const unsigned words = (length - 4) / 4;  // the status word and the payload
    if (status) *status = int32_t(rd(kOffPayload));
    if (rx_words) *rx_words = words - 1;
    for (unsigned i = 1; i < words && rx && i - 1 < rx_capacity; ++i) rx[i - 1] = rd(kOffPayload + 4 * i);
  } while (false);
  IOLockUnlock(gLock);
  return result;
}

namespace {

// A call that must also succeed in the firmware's eyes.
int checked(uint8_t protocol, uint8_t message, const uint32_t *tx, unsigned tx_words, uint32_t *rx, unsigned rx_capacity, unsigned *rx_words) {
  int32_t status = 0;
  const int result = rk3588_scmi_call(protocol, message, tx, tx_words, rx, rx_capacity, rx_words, &status);
  if (result) return result;
  return status ? 100 + (status < 0 ? -status : status) : 0;
}

}  // namespace

extern "C" int rk3588_scmi_base(char vendor[17], char sub_vendor[17], uint32_t *implementation) {
  uint32_t words[8] = {};
  unsigned count = 0;
  int result = checked(kProtoBase, kBaseVendor, nullptr, 0, words, 8, &count);
  if (result) return result;
  copyName(vendor, words);
  memset(words, 0, sizeof words);
  result = checked(kProtoBase, kBaseSubVendor, nullptr, 0, words, 8, &count);
  if (result) return result;
  copyName(sub_vendor, words);
  result = checked(kProtoBase, kBaseImplementation, nullptr, 0, words, 8, &count);
  if (result) return result;
  *implementation = words[0];
  return 0;
}

extern "C" int rk3588_scmi_clock_version(uint32_t *version) {
  unsigned count = 0;
  return checked(kProtoClock, kMsgVersion, nullptr, 0, version, 1, &count);
}

extern "C" int rk3588_scmi_clock_count(unsigned *clocks) {
  uint32_t words[4] = {};
  unsigned count = 0;
  const int result = checked(kProtoClock, kMsgAttributes, nullptr, 0, words, 4, &count);
  if (!result) *clocks = words[0] & 0xffff;
  return result;
}

extern "C" int rk3588_scmi_clock_attributes(uint32_t id, char name[17], uint32_t *attributes) {
  uint32_t words[8] = {};
  unsigned count = 0;
  const int result = checked(kProtoClock, kClockAttributes, &id, 1, words, 8, &count);
  if (result) return result;
  *attributes = words[0];
  copyName(name, words + 1);
  return 0;
}

extern "C" int rk3588_scmi_clock_rate_get(uint32_t id, uint64_t *hz) {
  uint32_t words[2] = {};
  unsigned count = 0;
  const int result = checked(kProtoClock, kClockRateGet, &id, 1, words, 2, &count);
  if (!result) *hz = uint64_t(words[0]) | (uint64_t(words[1]) << 32);
  return result;
}

extern "C" int rk3588_scmi_clock_rate_set(uint32_t id, uint64_t hz) {
  // flags 0: wait for completion, the rate is rounded as the firmware sees fit
  const uint32_t words[4] = {0, id, uint32_t(hz), uint32_t(hz >> 32)};
  unsigned count = 0;
  return checked(kProtoClock, kClockRateSet, words, 4, nullptr, 0, &count);
}

extern "C" int rk3588_scmi_clock_rates(uint32_t id, uint64_t *rates, unsigned capacity, unsigned *count, int *ranged) {
  *count = 0;
  *ranged = 0;
  unsigned index = 0;
  for (unsigned rounds = 0; rounds < 16; ++rounds) {
    const uint32_t request[2] = {id, index};
    uint32_t words[kPayloadWords] = {};
    unsigned got = 0;
    const int result = checked(kProtoClock, kClockDescribeRates, request, 2, words, kPayloadWords, &got);
    if (result) return result;
    const unsigned returned = words[0] & 0xfff, remaining = words[0] >> 16;
    const bool range = (words[0] >> 12) & 1;
    if (range) {
      *ranged = 1;
      for (unsigned i = 0; i < 3 && i < capacity; ++i) rates[i] = uint64_t(words[1 + 2 * i]) | (uint64_t(words[2 + 2 * i]) << 32);
      *count = 3;
      return 0;
    }
    for (unsigned i = 0; i < returned && 1 + 2 * i + 1 < kPayloadWords; ++i) {
      if (*count < capacity) rates[*count] = uint64_t(words[1 + 2 * i]) | (uint64_t(words[2 + 2 * i]) << 32);
      ++*count;
    }
    if (!remaining || !returned) break;
    index += returned;
  }
  if (*count > capacity) *count = capacity;
  return 0;
}
