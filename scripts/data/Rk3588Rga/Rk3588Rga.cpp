// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Kernel glue of Rk3588Rga: the power domain, clock and reset of RGA3 core 0, its IOMMU's page tables and their mapping of memory, the copy call the video
// path uses, and the self-tests (a thread started by rk3588_rga_start). See Rk3588Rga.h; the register-level code is in Rga3.h and RkIommu.h.
//
// Power, clock and reset are the vendor kernel's (drivers/soc/rockchip/pm_domains.c: PD_RGA30 is a child of PD_VDPU, itself a child of PD_VCODEC;
// drivers/clk/rockchip/clk-rk3588.c: aclk_rga3_0 and hclk_rga3_0 under the VDPU roots, clk_rga3_0_core from CLKSEL_CON(100); include/dt-bindings/clock/
// rk3588-cru.h: SRST_H_RGA3_0 730, SRST_A_RGA3_0 731, SRST_RGA3_0_CORE 732). The same facts the video decoder driver (Rk3588Vdec) uses for its domains;
// that driver and this one do not share code, so the PMU/CRU helpers below are a small copy, and this driver waits for the decoder's thread to finish
// powering the shared parents before it touches the PMU (rk3588_vdec_power_pending).
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

#include "Rga3.h"
#include "Rk3588Rga.h"
#include "RkIommu.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" int rk3588_vdec_power_pending(void) __attribute__((weak));   // Rk3588Vdec, when it is in the kernel

namespace {

constexpr uint64_t kPmuPhys = 0xfd8d8000ull, kCruPhys = 0xfd7c0000ull, kRgaPhys = 0xfdb60000ull;
constexpr uint32_t kRgaMapSize = 0x1000;

// ---- PMU (vendor "rk3588_pmu"): the writable bits sit in the lower half-word, their write enables in the upper one.
enum : uint32_t { kPmuReq = 0x10c, kPmuAck = 0x118, kPmuIdle = 0x120, kPmuPwr = 0x14c, kPmuStatus = 0x180, kPmuMemPwr = 0x1a0, kPmuChain = 0x1f0, kPmuMem = 0x1f8, kPmuRepair = 0x290 };
struct Domain {
  const char *name;
  uint32_t pwr;      // bit in kPmuPwr (1 = off)
  uint32_t status;   // bit in kPmuStatus (1 = off), or 0
  uint32_t mem;      // bit in kPmuMem (1 = memory off) and kPmuChain, or 0
  uint32_t repair;   // bit in kPmuRepair (1 = on), or 0
  uint32_t req;      // bit in kPmuReq / kPmuAck / kPmuIdle, or 0
};
constexpr Domain kVcodec = {"vcodec", 1u << 2, 1u << 2, 0, 0, 0};
constexpr Domain kVdpu = {"vdpu", 1u << 10, 0, 1u << 18, 1u << 9, 1u << 8};
constexpr Domain kRga30 = {"rga30", 1u << 11, 0, 1u << 19, 1u << 10, 0};   // no idle request: the vendor table has req 0 for it
constexpr const Domain *kPowerOrder[] = {&kVcodec, &kVdpu, &kRga30};

// ---- CRU: CLKSEL_CON(n), CLKGATE_CON(n) (1 = gated), SOFTRST_CON(n) (1 = in reset); write enables in the upper half-word.
constexpr uint32_t sel(unsigned n) { return 0x300 + 4 * n; }
constexpr uint32_t gate(unsigned n) { return 0x800 + 4 * n; }
constexpr uint32_t soft(unsigned n) { return 0xa00 + 4 * n; }
// CLKGATE_CON(44): 0 aclk_vdpu_root, 1 aclk_vdpu_low_root, 2 hclk_vdpu_root
// CLKGATE_CON(45): 10 hclk_rga3_0, 11 aclk_rga3_0, 12 clk_rga3_0_core
// CLKSEL_CON(100): [15:13] mux (0 gpll, 1 cpll, 2 npll, 3 aupll, 4 spll) and [12:8] divider of clk_rga3_0_core
// SOFTRST_CON(45): 10 SRST_H_RGA3_0, 11 SRST_A_RGA3_0, 12 SRST_RGA3_0_CORE (ids 730..732)
constexpr uint32_t kGateVdpu = gate(44), kGateRga = gate(45), kSelRga = sel(100), kSoftRga = soft(45);
constexpr uint32_t kGateVdpuAll = 0x7, kGateRgaBuses = 3u << 10, kGateRgaCore = 1u << 12, kSoftRgaAll = 7u << 10;
struct Pll {
  const char *name;
  unsigned con;
};
constexpr Pll kPlls[] = {{"gpll", 112}, {"cpll", 104}};   // mux inputs 0 and 1

volatile uint32_t *gPmu, *gCru;
volatile uint8_t *gRga;
uint8_t *gWin;           // the DMA window, as the CPU sees it
uint64_t gWinBus;
uint32_t gMode;

constexpr uint32_t kTableBytes = 0x80000;   // the first 512 KiB of the window: the page tables (the directory, then page tables as needed)
constexpr uint32_t kScratchOff = 0x80000;   // one page every guard page maps
constexpr uint32_t kTestOff = 0x100000;     // the self-test's pictures: 3 MiB of source from here, 3 MiB of destination from kTestOff + 3 MiB
constexpr uint32_t kTestBytes = 0x300000;
constexpr uint32_t kMinWindow = 8u << 20;

uint32_t bootArg(const char *name, uint32_t fallback) {
  uint32_t value = fallback;
  if (!PE_parse_boot_argn(name, &value, sizeof(value))) return fallback;
  return value;
}

uint64_t nowUs() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000;
}

// The window as normal non-cacheable memory (the block's writes are what the CPU reads, and the other way round). Through IOKit and the kernel map, not
// ml_io_map: that window is 8 MiB for every driver together and a mapping that does not fit panics (Rk3588Vdec's first run died of it).
uint8_t *mapWindow(uint64_t phys, uint32_t bytes) {
  IOMemoryDescriptor *descriptor = IOMemoryDescriptor::withPhysicalAddress(phys, bytes, kIODirectionInOut);
  if (!descriptor) return nullptr;
  IOMemoryMap *map = descriptor->map(kIOMapWriteCombineCache);   // kept for good
  descriptor->release();
  return map ? reinterpret_cast<uint8_t *>(map->getVirtualAddress()) : nullptr;
}

inline void dsb() { __asm__ volatile("dsb sy" ::: "memory"); }
inline uint32_t pmu(uint32_t off) { return gPmu[off / 4]; }
inline void pmuWrite(uint32_t off, uint32_t value) {
  gPmu[off / 4] = value;
  dsb();
}
inline uint32_t cru(uint32_t off) { return gCru[off / 4]; }
inline void cruWrite(uint32_t off, uint32_t mask, uint32_t value) {   // the bits of `mask` get `value`
  gCru[off / 4] = (mask << 16) | (value & mask);
  dsb();
}
inline uint32_t rgaRead(uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gRga + off); }
inline void rgaWrite(uint32_t off, uint32_t value) {
  *reinterpret_cast<volatile uint32_t *>(gRga + off) = value;
  dsb();
}

template <typename Condition> bool waitUs(unsigned limit, Condition done) {
  for (unsigned t = 0; t <= limit; t += 10) {
    if (done()) return true;
    IODelay(10);
  }
  return false;
}

inline uint32_t mmuReg(uint32_t reg) { return rgaRead(static_cast<uint32_t>(rga3::kIommuBase) + reg); }   // a register of the IOMMU (it is in the block's page)

// The register files for Rga3.h / RkIommu.h.
uint32_t hwRead(void *, uint32_t off) { return rgaRead(off); }
void hwWrite(void *, uint32_t off, uint32_t value) { rgaWrite(off, value); }
void hwDelay(void *, uint32_t us) { IODelay(us); }
uint64_t hwNow(void *) { return nowUs(); }
uint32_t mmuRead(void *, uint32_t off) { return rgaRead(rga3::kIommuBase + off); }
void mmuWrite(void *, uint32_t off, uint32_t value) { rgaWrite(rga3::kIommuBase + off, value); }
const rga3::Regs kRgaRegs = {nullptr, hwRead, hwWrite, hwDelay, hwNow};
const rkiommu::Regs kMmuRegs = {nullptr, mmuRead, mmuWrite, hwDelay};

// ---- PLLs (the vendor driver's pll_rk3588): rate = 24 MHz * (m + k / 65536) / p / 2^s
uint64_t pllKHz(unsigned con) {
  const uint32_t c0 = gCru[con], c1 = gCru[con + 1], c2 = gCru[con + 2];
  const uint64_t m = c0 & 0x3ff, p = c1 & 0x3f, s = (c1 >> 6) & 7, k = c2 & 0xffff;
  if (!p || (c1 & (1u << 13))) return 0;
  return ((24000ull * (m * 65536 + k)) / 65536 / p) >> s;
}

// The highest rate not above `targetKHz` a divider of 1..32 gives from gpll or cpll; sets the mux and the divider field (divider minus one).
uint64_t pickClock(uint64_t targetKHz, uint32_t *mux, uint32_t *div) {
  uint64_t best = 0;
  for (unsigned s = 0; s < sizeof kPlls / sizeof kPlls[0]; ++s) {
    const uint64_t src = pllKHz(kPlls[s].con);
    if (!src) continue;
    uint64_t n = (src + targetKHz - 1) / targetKHz;
    if (n < 1) n = 1;
    if (n > 32) continue;
    const uint64_t rate = src / n;
    if (rate > best) {
      best = rate;
      *mux = s;
      *div = static_cast<uint32_t>(n - 1);
    }
  }
  return best;
}

// ---- power domains
bool domainIsOn(const Domain &d) {
  if (d.repair) return (pmu(kPmuRepair) & d.repair) != 0;
  if (d.status) return !(pmu(kPmuStatus) & d.status);
  return false;
}
bool domainMemOn(const Domain &d) { return d.mem && !(pmu(kPmuMem) & d.mem); }

void printDomain(const Domain &d) {
  printf("rk3588-rga: pd %s: %s (pwr %#x status %#x repair %#x mem %#x chain %#x req %#x ack %#x idle %#x)\n", d.name, domainIsOn(d) ? "on" : "off", pmu(kPmuPwr),
         pmu(kPmuStatus), pmu(kPmuRepair), pmu(kPmuMem), pmu(kPmuChain), pmu(kPmuReq), pmu(kPmuAck), pmu(kPmuIdle));
}

// The vendor driver's rockchip_pd_power(on): the domain's clocks are running (done by the caller), then the power bit, the memory power cycle when the
// memory was on, the idle request withdrawn.
bool domainOn(const Domain &d) {
  if (domainIsOn(d)) {
    printf("rk3588-rga: pd %s is on already\n", d.name);
    return true;
  }
  const bool memWasOn = domainMemOn(d);
  pmuWrite(kPmuPwr, d.pwr << 16);   // bit = 0: on
  if (memWasOn) {
    if (!waitUs(10000, [&] { return (pmu(kPmuChain) & d.mem) != 0; })) {
      printf("rk3588-rga: pd %s: the memory chain did not come up (chain %#x)\n", d.name, pmu(kPmuChain));
      return false;
    }
    IODelay(60);
    pmuWrite(kPmuMemPwr, d.pwr | (d.pwr << 16));   // memory off ...
    if (!waitUs(10000, [&] { return !domainMemOn(d); })) {
      printf("rk3588-rga: pd %s: the memory did not power down (mem %#x)\n", d.name, pmu(kPmuMem));
      return false;
    }
    pmuWrite(kPmuMemPwr, d.pwr << 16);             // ... and on again
    if (!waitUs(10000, [&] { return domainMemOn(d); })) {
      printf("rk3588-rga: pd %s: the memory did not power up (mem %#x)\n", d.name, pmu(kPmuMem));
      return false;
    }
  }
  if (!waitUs(10000, [&] { return domainIsOn(d); })) {
    printf("rk3588-rga: pd %s did not come on (status %#x repair %#x)\n", d.name, pmu(kPmuStatus), pmu(kPmuRepair));
    return false;
  }
  if (d.req) {
    pmuWrite(kPmuReq, d.req << 16);   // idle request = 0
    if (!waitUs(10000, [&] { return !(pmu(kPmuAck) & d.req) && !(pmu(kPmuIdle) & d.req); })) {
      printf("rk3588-rga: pd %s stayed idle (ack %#x idle %#x)\n", d.name, pmu(kPmuAck), pmu(kPmuIdle));
      return false;
    }
  }
  printf("rk3588-rga: pd %s is on (pwr %#x status %#x repair %#x)\n", d.name, pmu(kPmuPwr), pmu(kPmuStatus), pmu(kPmuRepair));
  return true;
}

void lookPower() {
  printf("rk3588-rga: cru: gate44 %#x gate45 %#x soft45 %#x sel98 %#x sel100 %#x\n", cru(kGateVdpu), cru(kGateRga), cru(kSoftRga), cru(sel(98)), cru(kSelRga));
  for (const Pll &p : kPlls) printf("rk3588-rga: %s %llu kHz\n", p.name, static_cast<unsigned long long>(pllKHz(p.con)));
  for (const Domain *d : kPowerOrder) printDomain(*d);
}

bool setClocks(uint32_t mhz) {
  uint32_t mux = 0, div = 0;
  // The roots and the two bus clocks must run before the domain is touched (its idle handshake needs them).
  cruWrite(kGateVdpu, kGateVdpuAll, 0);
  cruWrite(kGateRga, kGateRgaBuses, 0);
  const uint64_t core = pickClock(static_cast<uint64_t>(mhz) * 1000, &mux, &div);
  if (!core) return false;
  cruWrite(kSelRga, 7u << 13 | 0x1fu << 8, mux << 13 | div << 8);
  cruWrite(kGateRga, kGateRgaCore, 0);
  printf("rk3588-rga: clock: core %llu kHz (target %u MHz, mux %u divider %u)\n", static_cast<unsigned long long>(core), mhz, mux, div + 1);
  return true;
}

// Resets pulsed on, then off: the block starts from a known state.
void resetBlock() {
  cruWrite(kSoftRga, kSoftRgaAll, kSoftRgaAll);
  IODelay(20);
  cruWrite(kSoftRga, kSoftRgaAll, 0);
  IODelay(20);
}

bool powerUp(uint32_t mhz) {
  if (!setClocks(mhz)) {
    printf("rk3588-rga: no PLL gives the block's clock\n");
    return false;
  }
  for (const Domain *d : kPowerOrder)
    if (!domainOn(*d)) return false;
  resetBlock();
  printf("rk3588-rga: reset lines soft45 %#x, gate45 %#x\n", cru(kSoftRga), cru(kGateRga));
  return true;
}

void lookBlock() {
  printf("rk3588-rga: version %#x time %#x sys_ctrl %#x int_en %#x int_raw %#x ro_srst %#x status0 %#x cmd_state %#x\n", rgaRead(rga3::kVersion), rgaRead(rga3::kVersionTime),
         rgaRead(rga3::kSysCtrl), rgaRead(rga3::kIntEn), rgaRead(rga3::kIntRaw), rgaRead(rga3::kRoSrst), rgaRead(rga3::kStatus0), rgaRead(rga3::kCmdState));
  // Rockchip IOMMU v2: DTE_ADDR, STATUS, COMMAND, PAGE_FAULT_ADDR, ZAP_ONE_LINE, INT_RAWSTAT, INT_CLEAR, INT_MASK, INT_STATUS, AUTO_GATING; STATUS bit 0 is paging enabled
  printf("rk3588-rga: iommu:");
  for (unsigned i = 0; i < 10; ++i) printf(" %#x", rgaRead(rga3::kIommuBase + 4 * i));
  printf("\n");
}

// ---- the block's address space: the IOMMU's tables, the free list of IO addresses, the lock that serializes everything below
constexpr uint32_t kIovaBase = 0x20000000u, kIovaPages = 65536, kGuardPages = 4;

rkiommu::PageTables gTables;
uint64_t gScratchBus;
IOLock *gLock;
uint32_t gBits[kIovaPages / 32];   // IO pages in use (a mapped range and its guard)
volatile bool gReady;
bool gIommuUp;
unsigned gFailStreak;
uint64_t gJobs, gRunUs, gFailed;
unsigned gRejectLogs;

bool pageUsed(uint32_t i) { return (gBits[i / 32] >> (i % 32)) & 1; }
void markPages(uint32_t start, uint32_t count, bool used) {
  for (uint32_t i = start; i < start + count; ++i) {
    if (used) gBits[i / 32] |= 1u << (i % 32);
    else gBits[i / 32] &= ~(1u << (i % 32));
  }
}
// First fit; the index of the first page or ~0.
uint32_t reservePages(uint32_t count) {
  uint32_t run = 0, start = 0;
  for (uint32_t i = 0; i < kIovaPages; ++i) {
    if (pageUsed(i)) {
      run = 0;
      start = i + 1;
      continue;
    }
    if (++run == count) {
      markPages(start, count, true);
      return start;
    }
  }
  return ~0u;
}

// What the CPU wrote to the tables must be in memory before the IOMMU's walker looks, and its translation cache must forget the old words.
bool tablesChanged() {
  dsb();
  return rkiommu::invalidate(kMmuRegs);
}

bool bringUpIommu() {
  if (!gTables.init(gWin, gWinBus, kTableBytes)) {
    printf("rk3588-rga: the page table window is not usable (virt %p bus %#llx)\n", gWin, static_cast<unsigned long long>(gWinBus));
    return false;
  }
  memset(gWin + kScratchOff, 0, rkiommu::kPageSize);
  gScratchBus = gWinBus + kScratchOff;
  memset(gBits, 0, sizeof gBits);
  dsb();
  uint32_t dte = 0, dummy = 0;
  if (!rkiommu::enable(kMmuRegs, gTables.directoryBus(), &dte, &dummy)) {
    printf("rk3588-rga: the IOMMU did not come up (status %#x dte %#x)\n", mmuReg(rkiommu::kRegStatus), mmuReg(rkiommu::kRegDteAddr));
    return false;
  }
  printf("rk3588-rga: iommu up: directory at %#llx, dte register %#x (after the pattern %#x), status %#x\n", static_cast<unsigned long long>(gTables.directoryBus()), dte, dummy,
         mmuReg(rkiommu::kRegStatus));
  gIommuUp = true;
  return true;
}

// Maps the guard behind a range.
bool mapGuard(uint32_t iova) { return gTables.mapSame(iova, gScratchBus, kGuardPages, true); }

void unmapRange(uint32_t iova, uint32_t pages) {
  gTables.unmap(iova, static_cast<uint64_t>(pages + kGuardPages) * rkiommu::kPageSize);
  markPages((iova - kIovaBase) / rkiommu::kPageSize, pages + kGuardPages, false);
  tablesChanged();
}

// Walks a descriptor's physical segments into the tables from `iova` on.
bool mapSegments(uint32_t iova, IOMemoryDescriptor *memory, uint64_t length) {
  for (IOByteCount off = 0; off < length;) {
    IOByteCount seg = 0;
    const addr64_t pa = memory->getPhysicalSegment(off, &seg, kIOMemoryMapperNone);
    if (!pa || (pa & 0xfff) || !seg) return false;
    if (off + seg > length) seg = static_cast<IOByteCount>(length - off);
    if ((seg & 0xfff) && off + seg < length) return false;   // a segment ending inside a page with more behind it cannot be put in a page table
    if (!gTables.map(iova + static_cast<uint32_t>(off), pa, seg, true)) return false;
    off += seg;
  }
  return true;
}

uint32_t pagesOf(uint64_t bytes) { return static_cast<uint32_t>((bytes + rkiommu::kPageSize - 1) / rkiommu::kPageSize); }

// Reserves room for `bytes` and its guard; the IO address, with *firstPage the page index. 0 when there is none.
uint32_t reserveRange(uint64_t bytes, uint32_t *firstPage) {
  const uint32_t pages = pagesOf(bytes);
  if (!pages || pages > kIovaPages - kGuardPages) return 0;
  const uint32_t first = reservePages(pages + kGuardPages);
  if (first == ~0u) return 0;
  *firstPage = first;
  return kIovaBase + first * rkiommu::kPageSize;
}

uint32_t mapPhysicalLocked(uint64_t phys, uint64_t bytes, bool writable) {
  if ((phys & 0xfff) || (phys >> 40)) return 0;
  uint32_t first = 0;
  const uint32_t iova = reserveRange(bytes, &first);
  if (!iova) return 0;
  const uint32_t pages = pagesOf(bytes);
  if (!gTables.map(iova, phys, static_cast<uint64_t>(pages) * rkiommu::kPageSize, writable) || !mapGuard(iova + pages * rkiommu::kPageSize)) {
    unmapRange(iova, pages);
    return 0;
  }
  if (!tablesChanged()) {
    unmapRange(iova, pages);
    return 0;
  }
  return iova;
}

// Brings the block back after a failed job: its own reset, then the IOMMU (which is in the same reset domain) with the same tables.
void recover() {
  const bool done = rga3::softReset(kRgaRegs);
  const bool mmu = !gIommuUp || rkiommu::enable(kMmuRegs, gTables.directoryBus());   // the physical-address test has no IOMMU to bring back
  printf("rk3588-rga: reset after a failed job: block %s, iommu %s\n", done ? "ok" : "DID NOT FINISH", mmu ? "ok" : "DID NOT COME UP");
  if (!done || !mmu) gReady = false;
}

// One copy, with the lock held. Returns 0, -1 (not for the block) or -2 (the block failed).
int copyLocked(const rga3::Copy &c, uint32_t *blockUs) {
  if (const char *why = rga3::check(c)) {
    if (gRejectLogs < 8) {
      ++gRejectLogs;
      printf("rk3588-rga: a copy is left to the CPU: %s (%ux%u from (%u,%u), pitches %u and %u)\n", why, c.width, c.height, c.srcX, c.srcRow, c.srcStride, c.dstStride);
    }
    return -1;
  }
  uint32_t cmd[rga3::kCmdWords];
  rga3::build(c, cmd);
  rga3::Result res = {};
  const bool ok = rga3::run(kRgaRegs, cmd, 100000, &res);
  if (blockUs) *blockUs = res.us;
  if (ok) {
    ++gJobs;
    gRunUs += res.us;
    gFailStreak = 0;
    return 0;
  }
  ++gFailed;
  printf("rk3588-rga: job failed: %s, int_raw %#x status0 %#x cmd_state %#x after %u us; iommu status %#x int_raw %#x fault address %#x\n", (res.intRaw & rga3::kIntErrors) ? "an error bit" : "no end",
         res.intRaw, res.status0, res.cmdState, res.us, mmuReg(rkiommu::kRegStatus), mmuReg(rkiommu::kRegIntRaw),
         mmuReg(rkiommu::kRegPageFaultAddr));
  recover();
  if (++gFailStreak >= 3) {
    printf("rk3588-rga: three failures in a row: the block is given up\n");
    gReady = false;
  }
  return -2;
}

// ---- self-tests

// A pseudo-random but position-dependent byte: any copy that moves a byte to the wrong place changes the picture.
uint8_t patternByte(uint32_t seed, size_t i) {
  uint32_t x = seed + static_cast<uint32_t>(i) * 2654435761u;
  x ^= x >> 15;
  x *= 2246822519u;
  x ^= x >> 13;
  return static_cast<uint8_t>(x);
}

struct Case {
  const char *name;
  uint32_t w, h;                 // what is copied
  uint32_t srcStride, srcRows;   // the source picture's pitch and rows per plane
  uint32_t srcX, srcRow;         // where the visible part starts
  uint32_t dstStride;
};
const Case kCases[] = {
    {"320x240", 320, 240, 320, 240, 0, 0, 320},
    {"cropped, wider pitch", 304, 224, 384, 256, 16, 8, 512},
    {"720p", 1280, 720, 1280, 720, 0, 0, 1280},
    {"720p padded pitches", 1280, 720, 1344, 736, 0, 0, 1408},
    {"1080p", 1920, 1080, 1920, 1088, 0, 0, 1920},
    {"narrowest", 68, 64, 80, 64, 0, 0, 80},
};
constexpr unsigned kCaseCount = sizeof kCases / sizeof kCases[0];

// Where the destination's planes are inside its buffer, as an IOSurface lays them out: luma one page in, chroma on the next page after it.
struct DstLayout {
  uint32_t yOff, uvOff, total;   // total is a multiple of 64 KiB (the scatter buffer's chunk)
};
DstLayout dstLayout(const Case &c) {
  DstLayout l;
  l.yOff = 0x1000;
  l.uvOff = (l.yOff + c.dstStride * c.h + 0xfff) & ~0xfffu;
  l.total = (l.uvOff + c.dstStride * (c.h / 2) + 0x1000 + 0xffff) & ~0xffffu;
  return l;
}

constexpr uint8_t kSentinel = 0xa5;
constexpr uint32_t kChunk = 0x10000;
constexpr uint32_t kSrcSeed = 0x5eed1234;

// The source picture of a case: luma rows then chroma rows of srcStride bytes (the chroma plane starts srcStride * srcRows in).
size_t srcBytes(const Case &c) { return static_cast<size_t>(c.srcStride) * c.srcRows * 3 / 2; }

// What the destination buffer must hold after the copy: sentinel everywhere except the copied rectangle of each plane.
void expectedDestination(const Case &c, const DstLayout &l, uint8_t *expected) {
  memset(expected, kSentinel, l.total);
  const size_t chromaStart = static_cast<size_t>(c.srcStride) * c.srcRows;
  for (uint32_t y = 0; y < c.h; ++y)
    for (uint32_t x = 0; x < c.w; ++x) expected[l.yOff + y * c.dstStride + x] = patternByte(kSrcSeed, static_cast<size_t>(c.srcRow + y) * c.srcStride + c.srcX + x);
  for (uint32_t y = 0; y < c.h / 2; ++y)
    for (uint32_t x = 0; x < c.w; ++x)
      expected[l.uvOff + y * c.dstStride + x] = patternByte(kSrcSeed, chromaStart + static_cast<size_t>(c.srcRow / 2 + y) * c.srcStride + c.srcX + x);
}

// Compares what the block wrote with the model; prints the first difference. Returns the number of different bytes.
size_t compare(const Case &c, const DstLayout &l, const uint8_t *expected, const uint8_t *got) {
  size_t bad = 0, first = 0;
  for (size_t i = 0; i < l.total; ++i) {
    if (expected[i] == got[i]) continue;
    if (!bad) first = i;
    ++bad;
  }
  if (bad) {
    const char *plane = first >= l.uvOff ? "chroma" : "luma";
    const size_t rel = first >= l.uvOff ? first - l.uvOff : first - l.yOff;
    printf("rk3588-rga: %s: %zu bytes differ; the first at buffer offset %#zx (%s plane, row %zu column %zu): expected %#x got %#x\n", c.name, bad, first, plane, rel / c.dstStride,
           rel % c.dstStride, expected[first], got[first]);
  }
  return bad;
}

struct Scratch {
  uint8_t *source, *expected, *got;
  size_t sourceMax, destMax;
  bool alloc() {
    sourceMax = destMax = 0;
    for (const Case &c : kCases) {
      if (srcBytes(c) > sourceMax) sourceMax = srcBytes(c);
      if (dstLayout(c).total > destMax) destMax = dstLayout(c).total;
    }
    source = static_cast<uint8_t *>(IOMalloc(sourceMax));
    expected = static_cast<uint8_t *>(IOMalloc(destMax));
    got = static_cast<uint8_t *>(IOMalloc(destMax));
    return source && expected && got;
  }
  void release() {
    if (source) IOFree(source, sourceMax);
    if (expected) IOFree(expected, destMax);
    if (got) IOFree(got, destMax);
    source = expected = got = nullptr;
  }
};

struct CaseMeter {
  uint64_t blockUs, callUs, worstBlockUs;
  unsigned runs, bad;
};

void printMeters(const char *what, const CaseMeter *meters) {
  for (unsigned i = 0; i < kCaseCount; ++i) {
    const CaseMeter &m = meters[i];
    if (!m.runs) continue;
    printf("rk3588-rga: %s %s: %u runs, %u bad; block %llu us on average (%llu at worst), whole call %llu us on average\n", what, kCases[i].name, m.runs, m.bad,
           static_cast<unsigned long long>(m.blockUs / m.runs), static_cast<unsigned long long>(m.worstBlockUs), static_cast<unsigned long long>(m.callUs / m.runs));
  }
}

// Mode 3: copies inside the window, the IOMMU off, addresses physical.
bool physicalTest(unsigned passes) {
  if (rk3588_rga_config.dma_bytes < kMinWindow) {
    printf("rk3588-rga: the window is %u bytes; %u are needed for the pictures\n", rk3588_rga_config.dma_bytes, kMinWindow);
    return false;
  }
  if (!rkiommu::disable(kMmuRegs)) {
    printf("rk3588-rga: the IOMMU would not stop translating (status %#x)\n", mmuReg(rkiommu::kRegStatus));
    return false;
  }
  Scratch s = {};
  if (!s.alloc()) {
    printf("rk3588-rga: no memory for the test pictures\n");
    s.release();
    return false;
  }
  CaseMeter meters[kCaseCount] = {};
  const uint64_t srcBus = gWinBus + kTestOff, dstBus = gWinBus + kTestOff + kTestBytes;
  uint8_t *srcWin = gWin + kTestOff, *dstWin = gWin + kTestOff + kTestBytes;
  bool allOk = true;
  for (unsigned ci = 0; ci < kCaseCount && allOk; ++ci) {
    const Case &c = kCases[ci];
    const DstLayout l = dstLayout(c);
    for (size_t i = 0; i < srcBytes(c); ++i) s.source[i] = patternByte(kSrcSeed, i);
    memcpy(srcWin, s.source, srcBytes(c));
    expectedDestination(c, l, s.expected);
    for (unsigned pass = 0; pass < passes && allOk; ++pass) {
      memset(dstWin, kSentinel, l.total);
      dsb();
      rga3::Copy copy = {static_cast<uint32_t>(srcBus), static_cast<uint32_t>(srcBus + static_cast<uint64_t>(c.srcStride) * c.srcRows), c.srcStride, c.srcX, c.srcRow,
                         static_cast<uint32_t>(dstBus + l.yOff), static_cast<uint32_t>(dstBus + l.uvOff), c.dstStride, c.w, c.h};
      uint32_t blockUs = 0;
      const uint64_t t0 = nowUs();
      const int rc = copyLocked(copy, &blockUs);
      const uint64_t callUs = nowUs() - t0;
      if (rc) {
        printf("rk3588-rga: %s: the copy failed (%d)\n", c.name, rc);
        allOk = false;
        break;
      }
      memcpy(s.got, dstWin, l.total);
      CaseMeter &m = meters[ci];
      ++m.runs;
      m.blockUs += blockUs;
      m.callUs += callUs;
      if (blockUs > m.worstBlockUs) m.worstBlockUs = blockUs;
      if (compare(c, l, s.expected, s.got)) {
        ++m.bad;
        allOk = false;
      }
    }
  }
  printMeters("physical", meters);
  s.release();
  printf("rk3588-rga: physical-address test: %s\n", allOk ? "all copies match" : "FAILED");
  return allOk;
}

// The destination of the IOMMU test: separate 64 KiB allocations the CPU sees through the kernel map, mapped one after the other in the block's address space.
struct Scatter {
  IOBufferMemoryDescriptor *chunk[64];
  uint32_t count, iova, pages;
  bool open(uint32_t bytes) {
    count = bytes / kChunk;
    if (count > 64) return false;
    for (uint32_t i = 0; i < count; ++i) {
      chunk[i] = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, static_cast<IOOptionBits>(kIODirectionInOut), kChunk, rkiommu::kPageSize);
      if (!chunk[i] || chunk[i]->prepare() != kIOReturnSuccess) {
        close();
        return false;
      }
    }
    uint32_t first = 0;
    iova = reserveRange(bytes, &first);
    pages = pagesOf(bytes);
    if (!iova) {
      close();
      return false;
    }
    for (uint32_t i = 0; i < count; ++i)
      if (!mapSegments(iova + i * kChunk, chunk[i], kChunk)) {
        unmapRange(iova, pages);
        iova = 0;
        close();
        return false;
      }
    if (!mapGuard(iova + pages * rkiommu::kPageSize) || !tablesChanged()) {
      unmapRange(iova, pages);
      iova = 0;
      close();
      return false;
    }
    return true;
  }
  void close() {
    for (uint32_t i = 0; i < count; ++i)
      if (chunk[i]) {
        (void)chunk[i]->complete();
        chunk[i]->release();
        chunk[i] = nullptr;
      }
    count = 0;
  }
  void release() {
    if (iova) unmapRange(iova, pages);
    iova = 0;
    close();
  }
  // How many physically separate pieces the buffer is made of.
  unsigned segments() const {
    unsigned n = 0;
    uint64_t next = 0;
    for (uint32_t i = 0; i < count; ++i)
      for (IOByteCount off = 0; off < kChunk;) {
        IOByteCount seg = 0;
        const addr64_t pa = chunk[i]->getPhysicalSegment(off, &seg, kIOMemoryMapperNone);
        if (!pa || !seg) return n;
        if (pa != next) ++n;
        next = pa + seg;
        off += seg;
      }
    return n;
  }
  void fill(uint8_t value) {
    for (uint32_t i = 0; i < count; ++i) {
      memset(chunk[i]->getBytesNoCopy(), value, kChunk);
      (void)chunk[i]->performOperation(kIOMemoryIncoherentIOFlush, 0, kChunk);
    }
  }
  void read(uint8_t *out) {
    for (uint32_t i = 0; i < count; ++i) {
      (void)chunk[i]->performOperation(kIOMemoryIncoherentIOFlush, 0, kChunk);   // drops what the CPU may have cached; the block wrote behind its back
      memcpy(out + static_cast<size_t>(i) * kChunk, chunk[i]->getBytesNoCopy(), kChunk);
    }
  }
};

// Mode 4 and 6: the source is the window (mapped read-only), the destination is scattered kernel memory, both reached through the IOMMU.
bool iommuTest(unsigned passes, bool fault) {
  if (rk3588_rga_config.dma_bytes < kMinWindow) {
    printf("rk3588-rga: the window is %u bytes; %u are needed for the pictures\n", rk3588_rga_config.dma_bytes, kMinWindow);
    return false;
  }
  Scratch s = {};
  if (!s.alloc()) {
    printf("rk3588-rga: no memory for the test pictures\n");
    s.release();
    return false;
  }
  CaseMeter meters[kCaseCount] = {};
  uint8_t *srcWin = gWin + kTestOff;
  IOLockLock(gLock);
  const uint32_t srcIova = mapPhysicalLocked(gWinBus + kTestOff, kTestBytes, false);
  IOLockUnlock(gLock);
  if (!srcIova) {
    printf("rk3588-rga: the source window could not be mapped\n");
    s.release();
    return false;
  }
  printf("rk3588-rga: source window mapped read-only at IO address %#x (%u pages used of the tables' %zu)\n", srcIova, static_cast<unsigned>(gTables.pagesUsed()), gTables.pagesTotal());
  bool allOk = true;
  for (unsigned ci = 0; ci < kCaseCount && allOk; ++ci) {
    const Case &c = kCases[ci];
    const DstLayout l = dstLayout(c);
    for (size_t i = 0; i < srcBytes(c); ++i) s.source[i] = patternByte(kSrcSeed, i);
    memcpy(srcWin, s.source, srcBytes(c));
    dsb();
    expectedDestination(c, l, s.expected);
    Scatter dst = {};
    IOLockLock(gLock);
    const bool opened = dst.open(l.total);
    IOLockUnlock(gLock);
    if (!opened) {
      printf("rk3588-rga: %s: the destination could not be allocated and mapped\n", c.name);
      allOk = false;
      break;
    }
    if (ci == 0) printf("rk3588-rga: destination of %u KiB in %u physically separate pieces, mapped at IO address %#x\n", l.total >> 10, dst.segments(), dst.iova);
    for (unsigned pass = 0; pass < passes && allOk; ++pass) {
      dst.fill(kSentinel);
      rga3::Copy copy = {srcIova, srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, dst.iova + l.yOff, dst.iova + l.uvOff, c.dstStride, c.w, c.h};
      uint32_t blockUs = 0;
      const uint64_t t0 = nowUs();
      IOLockLock(gLock);
      const int rc = copyLocked(copy, &blockUs);
      IOLockUnlock(gLock);
      const uint64_t callUs = nowUs() - t0;
      if (rc) {
        printf("rk3588-rga: %s: the copy failed (%d)\n", c.name, rc);
        allOk = false;
        break;
      }
      dst.read(s.got);
      CaseMeter &m = meters[ci];
      ++m.runs;
      m.blockUs += blockUs;
      m.callUs += callUs;
      if (blockUs > m.worstBlockUs) m.worstBlockUs = blockUs;
      if (compare(c, l, s.expected, s.got)) {
        ++m.bad;
        allOk = false;
      }
    }
    if (allOk && fault && ci == 1) {
      // The error path: a destination nothing maps. The block must stop with an error (or not at all), the driver must reset it, and the next copy must work.
      const uint32_t unmapped = kIovaBase + (kIovaPages - 64) * rkiommu::kPageSize;
      rga3::Copy bad = {srcIova, srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, unmapped, unmapped + 0x40000, c.dstStride, c.w, c.h};
      IOLockLock(gLock);
      const int rc = copyLocked(bad, nullptr);
      IOLockUnlock(gLock);
      printf("rk3588-rga: copy to an unmapped address: %s (%d)\n", rc == -2 ? "failed as it should" : "DID NOT FAIL", rc);
      dst.fill(kSentinel);
      rga3::Copy good = {srcIova, srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, dst.iova + l.yOff, dst.iova + l.uvOff, c.dstStride, c.w, c.h};
      IOLockLock(gLock);
      const int rc2 = copyLocked(good, nullptr);
      IOLockUnlock(gLock);
      dst.read(s.got);
      const bool after = rc2 == 0 && compare(c, l, s.expected, s.got) == 0;
      printf("rk3588-rga: the copy after the failure: %s\n", after ? "matches" : "FAILED");
      if (rc != -2 || !after) allOk = false;
      gFailStreak = 0;
    }
    IOLockLock(gLock);
    dst.release();
    IOLockUnlock(gLock);
  }
  IOLockLock(gLock);
  unmapRange(srcIova, pagesOf(kTestBytes));
  IOLockUnlock(gLock);
  printMeters("iommu", meters);
  s.release();
  printf("rk3588-rga: iommu test: %s; the block did %llu jobs in %llu us, %llu failed\n", allOk ? "all copies match" : "FAILED", static_cast<unsigned long long>(gJobs),
         static_cast<unsigned long long>(gRunUs), static_cast<unsigned long long>(gFailed));
  return allOk;
}

void rgaThread(void *, wait_result_t) {
  // The decoder driver may be powering the domains this one hangs under: wait for it (it clears the flag when done, whatever came of it).
  if (rk3588_vdec_power_pending)
    for (unsigned i = 0; i < 600 && rk3588_vdec_power_pending(); ++i) IOSleep(100);
  lookPower();
  if (gMode < 2) return;
  if (!powerUp(bootArg("rk3588-rga-mhz", 750))) return;
  lookBlock();
  if (gMode == 2) return;
  const unsigned passes = bootArg("rk3588-rga-passes", 10);
  if (gMode == 3) {
    physicalTest(passes);
    lookBlock();
    return;
  }
  if (!bringUpIommu()) return;
  if (gMode == 4 || gMode == 6) iommuTest(gMode == 4 ? passes : 1, bootArg("rk3588-rga-fault", 0) != 0);
  lookBlock();
  if (gMode >= 5) {
    printf("rk3588-rga: ready for clients\n");
    gFailStreak = 0;
    gReady = true;
  }
}

}  // namespace

extern "C" void rk3588_rga_start(void) {
  gMode = bootArg("rk3588-rga", 0);
  if (!gMode) return;
  printf("rk3588-rga: mode %u\n", gMode);
  vm_offset_t pmuMap = ml_io_map(static_cast<vm_offset_t>(kPmuPhys), 0x400), cruMap = ml_io_map(static_cast<vm_offset_t>(kCruPhys), 0x1000);
  vm_offset_t rgaMap = ml_io_map(static_cast<vm_offset_t>(kRgaPhys), kRgaMapSize);
  uint8_t *window = mapWindow(rk3588_rga_config.dma_phys, rk3588_rga_config.dma_bytes);
  if (!pmuMap || !cruMap || !rgaMap || !window) {
    printf("rk3588-rga: cannot map PMU (%d) CRU (%d) the block (%d) or the DMA window (%d)\n", pmuMap != 0, cruMap != 0, rgaMap != 0, window != nullptr);
    return;
  }
  gPmu = reinterpret_cast<volatile uint32_t *>(pmuMap);
  gCru = reinterpret_cast<volatile uint32_t *>(cruMap);
  gRga = reinterpret_cast<volatile uint8_t *>(rgaMap);
  gWin = window;
  gWinBus = rk3588_rga_config.dma_phys;
  gLock = IOLockAlloc();
  if (!gLock) return;
  thread_t thread = nullptr;
  if (kernel_thread_start(rgaThread, nullptr, &thread) == KERN_SUCCESS && thread) thread_deallocate(thread);
}

extern "C" int rk3588_rga_ready(void) { return gReady ? 1 : 0; }

extern "C" uint32_t rk3588_rga_map_physical(uint64_t phys, uint64_t bytes, int writable) {
  if (!gReady || !bytes) return 0;
  IOLockLock(gLock);
  const uint32_t iova = mapPhysicalLocked(phys, bytes, writable != 0);
  IOLockUnlock(gLock);
  return iova;
}

uint32_t rk3588_rga_map_memory(IOMemoryDescriptor *memory, uint64_t length) {
  if (!gReady || !memory || !length) return 0;
  IOLockLock(gLock);
  uint32_t first = 0;
  uint32_t iova = reserveRange(length, &first);
  if (iova) {
    const uint32_t pages = pagesOf(length);
    if (!mapSegments(iova, memory, length) || !mapGuard(iova + pages * rkiommu::kPageSize) || !tablesChanged()) {
      unmapRange(iova, pages);
      iova = 0;
    }
  }
  IOLockUnlock(gLock);
  return iova;
}

extern "C" void rk3588_rga_unmap(uint32_t iova, uint64_t bytes) {
  if (!gIommuUp || iova < kIovaBase || !bytes) return;
  IOLockLock(gLock);
  unmapRange(iova, pagesOf(bytes));
  IOLockUnlock(gLock);
}

extern "C" int rk3588_rga_copy_nv12(const struct Rk3588RgaPicture *src, uint32_t srcX, uint32_t srcRow, const struct Rk3588RgaPicture *dst, uint32_t width, uint32_t height,
                                    uint32_t *us) {
  if (!gReady) return -1;
  const rga3::Copy copy = {src->y, src->uv, src->stride, srcX, srcRow, dst->y, dst->uv, dst->stride, width, height};
  IOLockLock(gLock);
  const int rc = gReady ? copyLocked(copy, us) : -1;
  IOLockUnlock(gLock);
  return rc;
}

extern "C" void rk3588_rga_stats(uint64_t *jobs, uint64_t *runUs, uint64_t *failed) {
  if (jobs) *jobs = gJobs;
  if (runUs) *runUs = gRunUs;
  if (failed) *failed = gFailed;
}
