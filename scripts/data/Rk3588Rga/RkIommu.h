// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// RkIommu: the Rockchip IOMMU "v2" (the one in front of the RK3588's RGA3 cores, the VOP, the video blocks): its page tables and its register sequences.
// No kernel types: the page tables are built in a memory window the caller hands over, the registers are reached through an accessor, so the host test
// (tests/) runs the same code against a model of the hardware.
//
// All of it is taken from the vendor kernel's open driver (drivers/iommu/rockchip-iommu.c, the rk_mk_dte_v2 / rk_mk_pte_v2 / rk_iommu_enable paths);
// nothing is copied, the numbers are facts about the hardware.
//
// Two-level tables of a 32-bit IO virtual address space: bits 31:22 index the directory (1024 words, one 4 KiB page), bits 21:12 index a page table
// (1024 words, one 4 KiB page), bits 11:0 are the offset in a 4 KiB page. A word holds a 40-bit physical address spread over bits 31:4:
//   31:12 address bits 31:12, 11:8 address bits 35:32, 7:4 address bits 39:36, then (page table entries) 3 security, 2 writable, 1 readable, 0 valid.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace rkiommu {

constexpr uint32_t kPageSize = 4096;
constexpr uint32_t kEntries = 1024;

// Register offsets from the IOMMU's base (RGA3 core 0: 0xfdb60f00).
enum : uint32_t {
  kRegDteAddr = 0x00, kRegStatus = 0x04, kRegCommand = 0x08, kRegPageFaultAddr = 0x0c, kRegZapOneLine = 0x10, kRegIntRaw = 0x14, kRegIntClear = 0x18,
  kRegIntMask = 0x1c, kRegIntStatus = 0x20, kRegAutoGating = 0x24,
};
enum : uint32_t {
  kCmdEnablePaging = 0, kCmdDisablePaging = 1, kCmdEnableStall = 2, kCmdDisableStall = 3, kCmdZapCache = 4, kCmdPageFaultDone = 5, kCmdForceReset = 6,
};
enum : uint32_t {
  kStatusPaging = 1u << 0, kStatusPageFault = 1u << 1, kStatusStallActive = 1u << 2, kStatusIdle = 1u << 3, kStatusReplayEmpty = 1u << 4,
  kStatusPageFaultWrite = 1u << 5, kStatusStallNotActive = 1u << 31,
};
enum : uint32_t { kIntPageFault = 1, kIntBusError = 2 };
constexpr uint32_t kAutoGatingNoFetchLimit = 1u << 31;   // the vendor driver's "workaround for iommu blocked"

// A physical address in the form of a directory or page table word (without flags).
inline uint32_t encodeAddress(uint64_t pa) {
  return static_cast<uint32_t>(((pa & 0xfffff000ull) | ((pa & 0xf00000000ull) >> 24) | ((pa & 0xf000000000ull) >> 32)) & 0xfffffff0ull);
}
inline uint64_t decodeAddress(uint32_t word) {
  return (static_cast<uint64_t>(word & 0xf0) << 32) | (static_cast<uint64_t>(word & 0xf00) << 24) | (word & 0xfffff000u);
}
inline uint32_t makeDte(uint64_t ptBus) { return encodeAddress(ptBus) | 1u; }
inline uint32_t makePte(uint64_t pa, bool writable) { return encodeAddress(pa) | 2u | (writable ? 4u : 0u) | 1u; }   // readable always
inline bool isValid(uint32_t word) { return (word & 1u) != 0; }
inline bool isWritable(uint32_t word) { return (word & 4u) != 0; }

// The tables of one address space, in a window of memory both the CPU and the IOMMU can reach: `virt` is where the CPU sees it, `bus` what the IOMMU puts
// on the bus. The first page is the directory; page tables are taken from the rest as they are needed and stay (an unmapped range only clears its words).
class PageTables {
 public:
  bool init(uint8_t *virt, uint64_t bus, size_t bytes) {
    virt_ = nullptr;
    if (!virt || (bus & (kPageSize - 1)) || bytes < 2 * kPageSize || (reinterpret_cast<uintptr_t>(virt) & (kPageSize - 1))) return false;
    memset(virt, 0, bytes);
    virt_ = virt;
    bus_ = bus;
    pages_ = bytes / kPageSize;
    used_ = 1;   // the directory
    return true;
  }
  bool ready() const { return virt_ != nullptr; }
  uint64_t directoryBus() const { return bus_; }
  size_t pagesUsed() const { return used_; }
  size_t pagesTotal() const { return pages_; }

  // Maps [iova, iova + bytes) to the physical run starting at `phys`. Both addresses must be 4 KiB aligned; the length is rounded up to whole pages.
  // Words that are valid already are an error (a range is unmapped before it is mapped again). False when a page table cannot be had; what was written
  // stays and is the caller's to unmap.
  bool map(uint32_t iova, uint64_t phys, uint64_t bytes, bool writable) {
    if (!virt_ || (iova & (kPageSize - 1)) || (phys & (kPageSize - 1)) || (phys >> 40)) return false;
    const uint64_t pages = (bytes + kPageSize - 1) / kPageSize;
    if (!pages || static_cast<uint64_t>(iova) + pages * kPageSize > (1ull << 32) || (phys + pages * kPageSize - 1) >> 40) return false;
    for (uint64_t i = 0; i < pages; ++i) {
      uint32_t *pt = tableFor(iova + static_cast<uint32_t>(i * kPageSize), true);
      if (!pt) return false;
      uint32_t &word = pt[((iova >> 12) + i) & (kEntries - 1)];
      if (isValid(word)) return false;
      word = makePte(phys + i * kPageSize, writable);
    }
    return true;
  }

  // The same page for `pages` consecutive 4 KiB pages (the guard behind a mapped range).
  bool mapSame(uint32_t iova, uint64_t phys, uint32_t pages, bool writable) {
    for (uint32_t i = 0; i < pages; ++i)
      if (!map(iova + i * kPageSize, phys, kPageSize, writable)) return false;
    return true;
  }

  void unmap(uint32_t iova, uint64_t bytes) {
    if (!virt_) return;
    const uint64_t pages = (bytes + kPageSize - 1) / kPageSize;
    for (uint64_t i = 0; i < pages && static_cast<uint64_t>(iova) + i * kPageSize < (1ull << 32); ++i) {
      uint32_t *pt = tableFor(iova + static_cast<uint32_t>(i * kPageSize), false);
      if (pt) pt[((iova >> 12) + i) & (kEntries - 1)] = 0;
    }
  }

  // What the hardware would make of an address: the physical address, or false when the page is not mapped (a page fault).
  bool lookup(uint32_t iova, uint64_t *phys, bool *writable = nullptr) const {
    if (!virt_) return false;
    const uint32_t *pt = tableFor(iova, false);
    if (!pt) return false;
    const uint32_t word = pt[(iova >> 12) & (kEntries - 1)];
    if (!isValid(word)) return false;
    if (phys) *phys = decodeAddress(word) | (iova & (kPageSize - 1));
    if (writable) *writable = isWritable(word);
    return true;
  }

 private:
  uint32_t *dir() const { return reinterpret_cast<uint32_t *>(virt_); }
  uint32_t *tableFor(uint32_t iova, bool create) const {
    uint32_t &dte = dir()[iova >> 22];
    if (isValid(dte)) {
      const uint64_t at = decodeAddress(dte);
      return reinterpret_cast<uint32_t *>(virt_ + (at - bus_));
    }
    if (!create || used_ >= pages_) return nullptr;
    uint8_t *page = virt_ + used_ * kPageSize;   // zeroed by init: every word invalid
    dte = makeDte(bus_ + used_ * kPageSize);
    ++used_;
    return reinterpret_cast<uint32_t *>(page);
  }

  uint8_t *virt_ = nullptr;
  uint64_t bus_ = 0;
  size_t pages_ = 0;
  mutable size_t used_ = 0;
};

// The register file of one IOMMU as the driver reaches it.
struct Regs {
  void *ctx;
  uint32_t (*read)(void *ctx, uint32_t off);          // offsets of the enum above (relative to the IOMMU's base)
  void (*write)(void *ctx, uint32_t off, uint32_t value);
  void (*delayUs)(void *ctx, uint32_t us);
};

namespace detail {
template <typename Condition> bool waitFor(const Regs &r, unsigned limitUs, Condition done) {
  for (unsigned t = 0; t <= limitUs; t += 10) {
    if (done()) return true;
    r.delayUs(r.ctx, 10);
  }
  return false;
}
}  // namespace detail

inline void command(const Regs &r, uint32_t cmd) { r.write(r.ctx, kRegCommand, cmd); }

// The vendor driver's rk_iommu_enable: stall the unit (if it was translating), reset it, point it at the directory, shoot down the translation cache, enable
// the page fault and bus error interrupt bits (nothing listens: the driver polls the status), turn translation on, release the stall.
// The vendor's check that the directory register works (write a pattern, see it again) is done and what it read back is reported in `dummyReadback`
// instead of being demanded to equal a value; `dteReadback` receives what the register held after the real directory address was written.
inline bool enable(const Regs &r, uint64_t directoryBus, uint32_t *dteReadback = nullptr, uint32_t *dummyReadback = nullptr) {
  auto rd = [&](uint32_t off) { return r.read(r.ctx, off); };
  if (rd(kRegStatus) & kStatusPaging) {
    command(r, kCmdEnableStall);
    if (!detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusStallActive) != 0; })) return false;
  }
  r.write(r.ctx, kRegDteAddr, 0xcafebabeu);
  const uint32_t dummy = rd(kRegDteAddr);
  if (dummyReadback) *dummyReadback = dummy;
  command(r, kCmdForceReset);
  if (!detail::waitFor(r, 1000, [&] { return rd(kRegDteAddr) == 0; })) return false;
  r.write(r.ctx, kRegDteAddr, makeDte(directoryBus));   // the vendor writes mk_dtentries(): the valid bit is part of it
  if (dteReadback) *dteReadback = rd(kRegDteAddr);
  command(r, kCmdZapCache);
  r.write(r.ctx, kRegIntMask, kIntPageFault | kIntBusError);
  r.write(r.ctx, kRegAutoGating, rd(kRegAutoGating) | kAutoGatingNoFetchLimit);
  command(r, kCmdEnablePaging);
  if (!detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusPaging) != 0; })) return false;
  if (rd(kRegStatus) & kStatusStallActive) {   // the unit was stalled by the first step; now it translates
    command(r, kCmdDisableStall);
    return detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusStallActive) == 0; });
  }
  return true;
}

// Translation off (the unit passes addresses through).
inline bool disable(const Regs &r) {
  auto rd = [&](uint32_t off) { return r.read(r.ctx, off); };
  if (!(rd(kRegStatus) & kStatusPaging)) return true;
  command(r, kCmdEnableStall);
  if (!detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusStallActive) != 0; })) return false;
  command(r, kCmdDisablePaging);
  const bool off = detail::waitFor(r, 1000, [&] { return !(rd(kRegStatus) & kStatusPaging); });
  command(r, kCmdDisableStall);
  return off;
}

// After the IOMMU's page tables changed (a range unmapped, or mapped over a range that was): the translation cache must not hold the old words. While
// the unit translates it is stalled around the command, as the vendor driver does for its cache operations; the caller has no job running.
inline bool invalidate(const Regs &r) {
  auto rd = [&](uint32_t off) { return r.read(r.ctx, off); };
  if (!(rd(kRegStatus) & kStatusPaging)) {
    command(r, kCmdZapCache);
    return true;
  }
  command(r, kCmdEnableStall);
  if (!detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusStallActive) != 0; })) return false;
  command(r, kCmdZapCache);
  command(r, kCmdDisableStall);
  return detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusStallActive) == 0; });
}

}  // namespace rkiommu
