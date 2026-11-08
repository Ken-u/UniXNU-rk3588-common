// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Kernel glue of Rk3588Venc: the PMU and CRU access that powers and clocks the RKVENC0 block, the register mapping and the first look at the block. See
// Rk3588Venc.h.
//
// Everything here is derived from the vendor kernel's open sources: drivers/soc/rockchip/pm_domains.c (how the RK3588 turns a power domain on; the venc0
// entry is `DOMAIN_RK3588("venc0", 0x0, BIT(6), 0, 0x0, BIT(14), BIT(5), 0x0, BIT(4), BIT(4), false)`), drivers/clk/rockchip/clk-rk3588.c (the encoder's
// clock tree: `/* venc */`), include/dt-bindings/clock/rk3588-cru.h (reset numbers, "SOFTRST_CON47") and drivers/video/rockchip/mpp/mpp_rkvenc2.c (the
// register file, rkvenc_soft_reset). Nothing is copied; the register numbers are facts.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <stdint.h>

#include "Rk3588Venc.h"
#include "core/VencCore.h"
#include "VencTrace.h"
#include "VencTrace.inc"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

namespace {

constexpr uint64_t kPmuPhys = 0xfd8d8000ull, kCruPhys = 0xfd7c0000ull;
constexpr uint64_t kVencPhys = 0xfdbd0000ull;   // rkvenc0: registers at +0, its two IOMMUs at +0xf000 and +0xf040
constexpr uint32_t kVencMapSize = 0x10000, kMmuOff = 0xf000;

// ---- PMU, vendor "rk3588_pmu". The writable bits of the power and request registers sit in the lower half-word, their write enables in the upper one.
enum : uint32_t {
  kPmuReq = 0x10c, kPmuAck = 0x118, kPmuIdle = 0x120, kPmuPwr = 0x14c, kPmuStatus = 0x180, kPmuMemPwr = 0x1a0, kPmuChain = 0x1f0, kPmuMem = 0x1f8,
  kPmuRepair = 0x290,
};
struct Domain {
  const char *name;
  uint32_t pwr;      // bit in kPmuPwr (1 = off)
  uint32_t status;   // bit in kPmuStatus (1 = off), or 0
  uint32_t mem;      // bit in kPmuMem (1 = memory off) and kPmuChain, or 0
  uint32_t repair;   // bit in kPmuRepair (1 = on), or 0
  uint32_t req;      // bit in kPmuReq / kPmuAck / kPmuIdle, or 0
};
constexpr Domain kVcodec = {"vcodec", 1u << 2, 1u << 2, 0, 0, 0};
constexpr Domain kVenc0 = {"venc0", 1u << 6, 0, 1u << 14, 1u << 5, 1u << 4};
constexpr Domain kVenc1 = {"venc1", 1u << 7, 0, 1u << 15, 1u << 6, 1u << 5};   // only printed
constexpr Domain kRkvdec0 = {"rkvdec0", 1u << 8, 0, 1u << 16, 1u << 7, 1u << 6};   // only printed
constexpr const Domain *kLook[] = {&kVcodec, &kVenc0, &kVenc1, &kRkvdec0};
constexpr const Domain *kPowerOrder[] = {&kVcodec, &kVenc0};

// ---- CRU: CLKSEL_CON(n), CLKGATE_CON(n) (1 = gated) and SOFTRST_CON(n) (1 = in reset) with write enables in the upper half-word; PLL CON0 at 4 * con.
constexpr uint32_t sel(unsigned n) { return 0x300 + 4 * n; }
constexpr uint32_t gate(unsigned n) { return 0x800 + 4 * n; }
constexpr uint32_t soft(unsigned n) { return 0xa00 + 4 * n; }
// CLKSEL_CON(102): [1:0] hclk_rkvenc0_root (200/100/50/24 MHz), [8:7] mux and [6:2] divider of aclk_rkvenc0_root (gpll, cpll, npll),
//                  [15:14] mux and [13:9] divider of clk_rkvenc0_core (gpll, cpll, aupll, npll)
// CLKGATE_CON(47): 0 hclk_rkvenc0_root, 1 aclk_rkvenc0_root, 4 hclk_rkvenc0, 5 aclk_rkvenc0, 6 clk_rkvenc0_core (the bits 2 and 3 have no clock of the vendor tree)
// SOFTRST_CON(47) (ids 754..758 = 16 * 47 + 2..6, "Name=SOFTRST_CON47,Offset=0xABC" in the vendor header): 2 H_BIU, 3 A_BIU, 4 H, 5 A, 6 CORE
constexpr uint32_t kGateEnc = gate(47), kSoftEnc = soft(47);
constexpr uint32_t kGateEncAll = 1u << 0 | 1u << 1 | 1u << 4 | 1u << 5 | 1u << 6;   // must be 0
constexpr uint32_t kSoftEncAll = 0x7c;                                                // everything
constexpr uint32_t kSoftEncCore = 1u << 4 | 1u << 5 | 1u << 6;                       // what the vendor driver pulses (video_h, video_a, video_core)
struct Pll {
  const char *name;
  unsigned con;
};
constexpr Pll kPlls[] = {{"gpll", 112}, {"cpll", 104}, {"aupll", 96}, {"npll", 120}, {"v0pll", 88}};   // mux inputs 0 and 1 of the encoder clocks are gpll, cpll

// ---- the block's registers (mpp_rkvenc2.c, rkvenc_v2_hw_info of VEPU580)
enum : uint32_t {
  kRegId = 0x00, kRegEnable = 0x04, kRegStart = 0x10, kRegClear = 0x14, kRegIntEn = 0x20, kRegIntMask = 0x24, kRegIntClr = 0x28, kRegIntSta = 0x2c, kRegWdg = 0x38,
};
constexpr uint32_t kSclrDone = 1u << 2;   // INT_STA_SCLR_DONE_STA

volatile uint32_t *gPmu, *gCru;
volatile uint8_t *gVenc;
uint8_t *gDma;
uint32_t gMode;
bool gReady;           // powered, reset, window mapped (the template encoder may use the block)
bool gStreamOpen;      // one stream at a time

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

// The DMA window as normal non-cacheable memory (what the block writes is what the CPU reads, and the other way round, without cache maintenance). It goes
// through IOKit and the kernel map, not ml_io_map: the kernel's I/O window is 8 MiB for every ml_io_map of every driver together, permanent, and a mapping
// that does not fit panics (the decoder's first run died on a 16 MiB window).
uint8_t *mapWindow(uint64_t phys, uint32_t bytes) {
  IOMemoryDescriptor *descriptor = IOMemoryDescriptor::withPhysicalAddress(phys, bytes, kIODirectionInOut);
  if (!descriptor) return nullptr;
  IOMemoryMap *map = descriptor->map(kIOMapWriteCombineCache);   // kept for good: the window is never unmapped
  descriptor->release();
  return map ? reinterpret_cast<uint8_t *>(map->getVirtualAddress()) : nullptr;
}

inline uint32_t pmu(uint32_t off) { return gPmu[off / 4]; }
inline void pmuWrite(uint32_t off, uint32_t value) {
  gPmu[off / 4] = value;
  __asm__ volatile("dsb sy" ::: "memory");
}
inline uint32_t cru(uint32_t off) { return gCru[off / 4]; }
// A write-masked CRU register: the bits of `mask` get the value `value`.
inline void cruWrite(uint32_t off, uint32_t mask, uint32_t value) {
  gCru[off / 4] = (mask << 16) | (value & mask);
  __asm__ volatile("dsb sy" ::: "memory");
}
inline uint32_t vencRead(uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gVenc + off); }
inline void vencWrite(uint32_t off, uint32_t value) {
  *reinterpret_cast<volatile uint32_t *>(gVenc + off) = value;
  __asm__ volatile("dsb sy" ::: "memory");
}

// Poll with a time limit (microseconds); true when the condition came true.
template <typename Condition> bool waitUs(unsigned limit, Condition done) {
  for (unsigned t = 0; t <= limit; t += 10) {
    if (done()) return true;
    IODelay(10);
  }
  return false;
}

// ---- PLLs (the vendor driver's pll_rk3588): rate = 24 MHz * (m + k / 65536) / p / 2^s
uint64_t pllKHz(unsigned con) {
  const uint32_t c0 = gCru[con], c1 = gCru[con + 1], c2 = gCru[con + 2];
  const uint64_t m = c0 & 0x3ff, p = c1 & 0x3f, s = (c1 >> 6) & 7, k = c2 & 0xffff;
  if (!p || (c1 & (1u << 13))) return 0;   // no divider, or powered down
  return ((24000ull * (m * 65536 + k)) / 65536 / p) >> s;
}

// The highest rate not above `targetKHz` that a divider of 1..32 gives from one of the first `sources` PLLs of kPlls; sets *mux and *div (register value, divider minus one).
uint64_t pickClock(uint64_t targetKHz, unsigned sources, uint32_t *mux, uint32_t *div) {
  uint64_t best = 0;
  for (unsigned s = 0; s < sources; ++s) {
    const uint64_t src = pllKHz(kPlls[s].con);
    if (!src) continue;
    uint64_t n = (src + targetKHz - 1) / targetKHz;   // rounds up: the result is not above the target
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
  printf("rk3588-venc: pd %s: %s (pwr %#x status %#x repair %#x mem %#x chain %#x req %#x ack %#x idle %#x)\n", d.name, domainIsOn(d) ? "on" : "off",
         pmu(kPmuPwr), pmu(kPmuStatus), pmu(kPmuRepair), pmu(kPmuMem), pmu(kPmuChain), pmu(kPmuReq), pmu(kPmuAck), pmu(kPmuIdle));
}

// The vendor driver's rockchip_pd_power(on): the domain's clocks are running (done by the caller), then the power bit, the memory power cycle when the
// memory was on, the idle request withdrawn.
bool domainOn(const Domain &d) {
  if (domainIsOn(d)) {
    printf("rk3588-venc: pd %s is on already\n", d.name);
    return true;
  }
  const bool memWasOn = domainMemOn(d);
  pmuWrite(kPmuPwr, d.pwr << 16);   // bit = 0: on
  if (memWasOn) {
    if (!waitUs(10000, [&] { return (pmu(kPmuChain) & d.mem) != 0; })) {
      printf("rk3588-venc: pd %s: the memory chain did not come up (chain %#x)\n", d.name, pmu(kPmuChain));
      return false;
    }
    IODelay(60);
    pmuWrite(kPmuMemPwr, d.pwr | (d.pwr << 16));   // memory off ...
    if (!waitUs(10000, [&] { return !domainMemOn(d); })) {
      printf("rk3588-venc: pd %s: the memory did not power down (mem %#x)\n", d.name, pmu(kPmuMem));
      return false;
    }
    pmuWrite(kPmuMemPwr, d.pwr << 16);             // ... and on again
    if (!waitUs(10000, [&] { return domainMemOn(d); })) {
      printf("rk3588-venc: pd %s: the memory did not power up (mem %#x)\n", d.name, pmu(kPmuMem));
      return false;
    }
  }
  if (!waitUs(10000, [&] { return domainIsOn(d); })) {
    printf("rk3588-venc: pd %s did not come on (status %#x repair %#x)\n", d.name, pmu(kPmuStatus), pmu(kPmuRepair));
    return false;
  }
  if (d.req) {
    pmuWrite(kPmuReq, d.req << 16);   // idle request = 0
    if (!waitUs(10000, [&] { return !(pmu(kPmuAck) & d.req) && !(pmu(kPmuIdle) & d.req); })) {
      printf("rk3588-venc: pd %s stayed idle (ack %#x idle %#x)\n", d.name, pmu(kPmuAck), pmu(kPmuIdle));
      return false;
    }
  }
  printf("rk3588-venc: pd %s is on (pwr %#x status %#x repair %#x)\n", d.name, pmu(kPmuPwr), pmu(kPmuStatus), pmu(kPmuRepair));
  return true;
}

void lookPower() {
  printf("rk3588-venc: cru: gate47 %#x soft47 %#x sel102 %#x gate48 %#x soft48 %#x sel104 %#x\n", cru(kGateEnc), cru(kSoftEnc), cru(sel(102)), cru(gate(48)),
         cru(soft(48)), cru(sel(104)));
  for (const Pll &p : kPlls) printf("rk3588-venc: %s %llu kHz%s\n", p.name, static_cast<unsigned long long>(pllKHz(p.con)), (cru(4 * p.con + 24) & (1u << 15)) ? "" : " (not locked)");
  for (const Domain *d : kLook) printDomain(*d);
}

// Dividers and gates of the encoder's clock tree; the targets are what the vendor tree assigns (assigned-clock-rates: aclk 500, core 800 MHz), reached as
// closely as the PLLs of the firmware allow without going above them. The hclk root keeps the setting it has.
bool setClocks() {
  uint32_t mux = 0, div = 0;
  // The roots and their gates must be running before the power domain is touched (the NIU needs them to answer the idle request).
  cruWrite(kGateEnc, 1u << 0 | 1u << 1 | 1u << 4 | 1u << 5, 0);
  const uint64_t aclk = pickClock(500000, 2, &mux, &div);
  if (!aclk) return false;
  cruWrite(sel(102), 3u << 7 | 0x1fu << 2, mux << 7 | div << 2);               // aclk_rkvenc0_root
  const uint64_t core = pickClock(800000, 2, &mux, &div);
  if (!core) return false;
  cruWrite(sel(102), 3u << 14 | 0x1fu << 9, mux << 14 | div << 9);             // clk_rkvenc0_core
  printf("rk3588-venc: clocks: aclk %llu core %llu kHz (sel102 %#x)\n", static_cast<unsigned long long>(aclk), static_cast<unsigned long long>(core), cru(sel(102)));
  cruWrite(kGateEnc, kGateEncAll, 0);
  return true;
}

bool powerUp() {
  if (!setClocks()) {
    printf("rk3588-venc: no PLL gives the encoder clocks\n");
    return false;
  }
  for (const Domain *d : kPowerOrder)
    if (!domainOn(*d)) return false;
  printf("rk3588-venc: reset lines before: soft47 %#x\n", cru(kSoftEnc));
  cruWrite(kSoftEnc, kSoftEncAll, 0);   // everything out of reset
  IODelay(20);
  printf("rk3588-venc: reset lines after: soft47 %#x, gate47 %#x\n", cru(kSoftEnc), cru(kGateEnc));
  return true;
}

// ---- the block
void lookBlock() {
  printf("rk3588-venc: id %#x enable %#x\n", vencRead(kRegId), vencRead(kRegEnable));
  printf("rk3588-venc: regs 0..23:");
  for (unsigned i = 0; i < 24; ++i) printf(" %#x", vencRead(4 * i));
  printf("\n");
  printf("rk3588-venc: irq: en %#x mask %#x status %#x wdg %#x\n", vencRead(kRegIntEn), vencRead(kRegIntMask), vencRead(kRegIntSta), vencRead(kRegWdg));
  // Rockchip IOMMU v2: DTE_ADDR, STATUS, COMMAND, PAGE_FAULT_ADDR, ZAP_ONE_LINE, INT_RAWSTAT, INT_CLEAR, INT_MASK, INT_STATUS, AUTO_GATING; STATUS bit 0 is paging enabled
  for (unsigned mmu = 0; mmu < 2; ++mmu) {
    printf("rk3588-venc: iommu %u:", mmu);
    for (unsigned i = 0; i < 10; ++i) printf(" %#x", vencRead(kMmuOff + 0x40 * mmu + 4 * i));
    printf("\n");
  }
}

// rkvenc_soft_reset of the vendor driver: mask the interrupts, ask for a safe stop, wait for "safe reset done", clear, release.
bool softReset() {
  vencWrite(kRegIntMask, 0x3ff);
  vencWrite(kRegClear, 0x1);
  const bool done = waitUs(1000, [&] { return (vencRead(kRegIntSta) & kSclrDone) != 0; });
  const uint32_t status = vencRead(kRegIntSta);
  vencWrite(kRegClear, 0x2);
  IODelay(5);
  vencWrite(kRegClear, 0);
  vencWrite(kRegIntClr, 0xffffffffu);
  vencWrite(kRegIntSta, 0);
  printf("rk3588-venc: safe reset %s (status %#x, afterwards %#x)\n", done ? "done" : "TIMED OUT", status, vencRead(kRegIntSta));
  return done;
}

// What the vendor driver does when the safe reset times out: the three reset lines of the block (video_a, video_h, video_core) asserted, 5 us, released.
// (It asks the PMU for an idle bus interface first only where the device tree does not say `rockchip,skip-pmu-idle-request`, and the RK3588's does.)
void resetLines() {
  cruWrite(kSoftEnc, kSoftEncCore, kSoftEncCore);
  IODelay(5);
  cruWrite(kSoftEnc, kSoftEncCore, 0);
  IODelay(20);
}

// ---- the self-test: replay recorded encode jobs and print what the block wrote
uint32_t hwRead(void *, uint32_t off) { return vencRead(off); }
void hwWrite(void *, uint32_t off, uint32_t value) { vencWrite(off, value); }
void hwBarrier(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
uint64_t hwNow(void *) { return nowUs(); }
void hwDelay(void *, uint32_t us) {
  if (us >= 1000) IOSleep((us + 999) / 1000);
  else IODelay(us);
}

venc::Hw blockHw() {
  venc::Hw hw = {};
  hw.read32 = hwRead;
  hw.write32 = hwWrite;
  hw.barrier = hwBarrier;
  hw.nowUs = hwNow;
  hw.delayUs = hwDelay;
  hw.dmaVirt = gDma;
  hw.dmaBus = rk3588_venc_config.dma_phys;
  hw.dmaBytes = rk3588_venc_config.dma_bytes;
  return hw;
}

// The bytes of the stream buffer, 32 to a line; tools/check-enc-output.py puts them back together.
void dumpStream(const char *name, unsigned job, const uint8_t *p, uint32_t len) {
  static const char hex[] = "0123456789abcdef";
  for (uint32_t off = 0; off < len; off += 32) {
    char line[80];
    unsigned n = 0;
    for (uint32_t i = 0; i < 32 && off + i < len; ++i) {
      line[n++] = hex[p[off + i] >> 4];
      line[n++] = hex[p[off + i] & 15];
    }
    line[n] = 0;
    printf("rk3588-venc: stream %s job %u %06x %s\n", name, job, off, line);
  }
}

void lookStatus() {
  printf("rk3588-venc: irq: en %#x mask %#x status %#x; status class:", vencRead(kRegIntEn), vencRead(kRegIntMask), vencRead(kRegIntSta));
  for (unsigned i = 0; i < 16; ++i) printf(" %#x", vencRead(venc::kRegStStreamLen + 4 * i));
  printf("\n");
}

// One trace, `passes` times over. Returns false on the first job that fails (the jobs depend on each other: the second reads what the first reconstructed).
// With `dump`, the bit stream of every job is printed on the first pass; on every pass its CRC is compared with the first pass's (the block is deterministic).
bool runTrace(const char *name, const unsigned char *data, size_t size, unsigned passes, bool dump) {
  venc::Trace *trace = static_cast<venc::Trace *>(IOMallocZero(sizeof(venc::Trace)));
  venc::Player *player = static_cast<venc::Player *>(IOMallocZero(sizeof(venc::Player)));
  venc::Job *job = static_cast<venc::Job *>(IOMallocZero(sizeof(venc::Job)));
  uint32_t *crcs = static_cast<uint32_t *>(IOMallocZero(4 * venc::kMaxBuffers * 64));
  uint32_t *lens = static_cast<uint32_t *>(IOMallocZero(4 * venc::kMaxBuffers * 64));
  bool ok = false;
  if (trace && player && job && crcs && lens && trace->parse(data, size)) {
    printf("rk3588-venc: %s: %ux%u, %u jobs, %u buffers, %u bytes of window needed (%u given)\n", name, trace->width(), trace->height(), trace->jobCount(),
           trace->bufCount(), trace->windowBytes(), rk3588_venc_config.dma_bytes);
    player->init(blockHw(), trace);
    ok = trace->jobCount() <= venc::kMaxBuffers * 64;
    uint64_t totalUs = 0, worstUs = 0, jobs = 0;
    for (unsigned pass = 0; pass < passes && ok; ++pass) {
      if (!player->prepare()) {
        printf("rk3588-venc: %s: the DMA window (%#llx, %u bytes) cannot hold the trace\n", name, static_cast<unsigned long long>(rk3588_venc_config.dma_phys),
               rk3588_venc_config.dma_bytes);
        ok = false;
        break;
      }
      size_t cursor = 0;
      for (unsigned n = 0; n < trace->jobCount() && ok; ++n) {
        if (!trace->job(&cursor, job)) {
          printf("rk3588-venc: %s: job %u: %s\n", name, n, trace->error());
          ok = false;
          break;
        }
        venc::JobResult r;
        player->run(*job, 500000, &r);
        const uint32_t length = r.streamLen + 64 < job->outLen ? r.streamLen + 64 : job->outLen;   // the stream and what comes just after it
        const uint8_t *stream = player->stream(*job);
        const uint32_t crc = venc::crc32(stream, length);
        if (pass == 0 || !r.ok)
          printf("rk3588-venc: %s: job %u: %s status %#x stream %u bytes in %llu us, crc %#x of %u bytes\n", name, n, r.ok ? "ok" : "FAILED", r.status, r.streamLen,
                 static_cast<unsigned long long>(r.runUs), crc, length);
        if (!r.ok) {
          lookStatus();
          ok = false;
          break;
        }
        if (pass == 0) {
          crcs[n] = crc;
          lens[n] = r.streamLen;
          if (dump) dumpStream(name, n, stream, length);
        } else if (crc != crcs[n] || r.streamLen != lens[n]) {
          printf("rk3588-venc: %s: pass %u job %u: stream differs from the first pass (crc %#x, %#x; %u bytes, %u)\n", name, pass, n, crc, crcs[n], r.streamLen,
                 lens[n]);
          ok = false;
        }
        totalUs += r.runUs;
        if (r.runUs > worstUs) worstUs = r.runUs;
        ++jobs;
      }
    }
    if (jobs && ok)
      printf("rk3588-venc: %s: %llu jobs, %llu us per job on the block on average, %llu us at worst: ok\n", name, static_cast<unsigned long long>(jobs),
             static_cast<unsigned long long>(totalUs / jobs), static_cast<unsigned long long>(worstUs));
  } else if (trace && trace->error()[0]) {
    printf("rk3588-venc: %s: the trace is not usable: %s\n", name, trace->error());
  }
  if (lens) IOFree(lens, 4 * venc::kMaxBuffers * 64);
  if (crcs) IOFree(crcs, 4 * venc::kMaxBuffers * 64);
  if (job) IOFree(job, sizeof(venc::Job));
  if (player) IOFree(player, sizeof(venc::Player));
  if (trace) IOFree(trace, sizeof(venc::Trace));
  return ok;
}

bool selfTest(unsigned passes, bool dump) {
  unsigned failed = 0;
  for (const auto &t : kVencTraces)
    if (!runTrace(t.name, t.data, t.size, passes, dump)) {
      ++failed;
      break;   // the block may be wedged: the next trace would only add noise
    }
  printf("rk3588-venc: self-test: %s\n", failed ? "FAILED" : "all traces ok");
  return failed == 0;
}

// ---- the trace-template encoder (see Rk3588Venc.h)
IOLock *gStreamLock;

struct StreamImpl {
  venc::Trace *trace;
  venc::Player *player;
  venc::Job *job;
  const char *name;
  size_t cursor;
  uint32_t index;
  uint32_t srcBuf;
  uint8_t header[512];
  uint32_t headerLength;
};

bool nameHas(const char *name, const char *part) {
  for (; *name; ++name) {
    const char *a = name, *b = part;
    while (*a && *b && *a == *b) ++a, ++b;
    if (!*b) return true;
  }
  return false;
}

// Where the source picture of the first job is: the three address registers of the encoder's input (adr_src0..2 = registers 160..162).
bool findPicture(const venc::Job &job, uint32_t *buffer, uint32_t offset[3]) {
  unsigned found = 0;
  for (uint32_t i = 0; i < job.relocCount; ++i) {
    const venc::Reloc &r = job.relocs[i];
    if (r.reg < 160 || r.reg > 162) continue;
    if (found && r.buf != *buffer) return false;
    *buffer = r.buf;
    offset[r.reg - 160] = r.offset;
    ++found;
  }
  return found == 3;
}

}  // namespace

extern "C" int rk3588_venc_ready(void) { return gReady; }

extern "C" struct Rk3588VencStream *rk3588_venc_stream_open(uint32_t width, uint32_t height, int hevc, struct Rk3588VencPicture *picture, const char **name) {
  if (!gReady || !gDma || !picture) return nullptr;
  if (!gStreamLock) gStreamLock = IOLockAlloc();
  if (!gStreamLock) return nullptr;
  IOLockLock(gStreamLock);
  if (gStreamOpen) {
    IOLockUnlock(gStreamLock);
    return nullptr;
  }
  StreamImpl *s = static_cast<StreamImpl *>(IOMallocZero(sizeof(StreamImpl)));
  venc::Trace *trace = static_cast<venc::Trace *>(IOMallocZero(sizeof(venc::Trace)));
  venc::Player *player = static_cast<venc::Player *>(IOMallocZero(sizeof(venc::Player)));
  venc::Job *job = static_cast<venc::Job *>(IOMallocZero(sizeof(venc::Job)));
  bool ok = false;
  if (s && trace && player && job) {
    // the preferred trace of a size: the one recorded with a closed GOP of several pictures ("gop"), else the first
    const unsigned char *best = nullptr;
    size_t bestSize = 0;
    const char *bestName = nullptr;
    for (const auto &t : kVencTraces) {
      if (nameHas(t.name, "hevc") != (hevc != 0)) continue;
      if (!trace->parse(t.data, t.size)) continue;
      if (trace->width() != width || trace->height() != height) continue;
      if (!best || nameHas(t.name, "gop")) {
        best = t.data;
        bestSize = t.size;
        bestName = t.name;
      }
    }
    if (best && trace->parse(best, bestSize)) {
      uint32_t buffer = 0, offset[3] = {};
      size_t cursor = 0;
      if (trace->job(&cursor, job) && findPicture(*job, &buffer, offset)) {
        const uint32_t rows = trace->height();
        const uint32_t yStride = rows ? (offset[1] - offset[0]) / rows : 0, uStride = rows >= 2 ? (offset[2] - offset[1]) / (rows / 2) : 0;
        if (yStride >= trace->width() && uStride >= trace->width() / 2 && offset[2] + uint64_t(uStride) * (rows / 2) <= trace->bufSize(buffer)) {
          player->init(blockHw(), trace);
          if (player->prepare()) {
            // the parameter sets: what the CPU wrote at the head of the stream buffer before the first job
            uint32_t extent = 0;
            if (job->outOffset == 0) {
              for (uint32_t i = 0; i < job->uploadCount; ++i) {
                const venc::Upload &u = job->uploads[i];
                if (u.buf != job->outBuf || u.offset + u.len > sizeof s->header) continue;
                memcpy(s->header + u.offset, u.data, u.len);
                if (u.offset + u.len > extent) extent = u.offset + u.len;
              }
            }
            s->trace = trace;
            s->player = player;
            s->job = job;
            s->name = bestName;
            s->cursor = 0;
            s->index = 0;
            s->srcBuf = buffer;
            s->headerLength = extent;
            uint8_t *base = gDma + trace->bufOffset(buffer);
            picture->y = base + offset[0];
            picture->u = base + offset[1];
            picture->v = base + offset[2];
            picture->width = trace->width();
            picture->height = trace->height();
            picture->yStride = yStride;
            picture->uStride = uStride;
            if (name) *name = bestName;
            gStreamOpen = true;
            ok = true;
          }
        }
      }
    }
  }
  IOLockUnlock(gStreamLock);
  if (!ok) {
    if (job) IOFree(job, sizeof(venc::Job));
    if (player) IOFree(player, sizeof(venc::Player));
    if (trace) IOFree(trace, sizeof(venc::Trace));
    if (s) IOFree(s, sizeof(StreamImpl));
    return nullptr;
  }
  return reinterpret_cast<Rk3588VencStream *>(s);
}

extern "C" void rk3588_venc_stream_close(struct Rk3588VencStream *stream) {
  StreamImpl *s = reinterpret_cast<StreamImpl *>(stream);
  if (!s) return;
  IOLockLock(gStreamLock);
  gStreamOpen = false;
  IOLockUnlock(gStreamLock);
  IOFree(s->job, sizeof(venc::Job));
  IOFree(s->player, sizeof(venc::Player));
  IOFree(s->trace, sizeof(venc::Trace));
  IOFree(s, sizeof(StreamImpl));
}

extern "C" int rk3588_venc_stream_encode(struct Rk3588VencStream *stream, struct Rk3588VencResult *result) {
  StreamImpl *s = reinterpret_cast<StreamImpl *>(stream);
  if (!s || !result) return 0;
  *result = {};
  IOLockLock(gStreamLock);
  bool ok = false;
  if (s->trace->job(&s->cursor, s->job)) {
    venc::Job &job = *s->job;
    for (uint32_t i = 0; i < job.uploadCount; ++i)
      if (job.uploads[i].buf == s->srcBuf) job.uploads[i].len = 0;   // the picture is the caller's, not the recording's
    venc::JobResult r;
    ok = s->player->run(job, 500000, &r);
    result->job = s->index;
    result->runUs = static_cast<uint32_t>(r.runUs);
    if (!ok) printf("rk3588-venc: stream %s job %u: FAILED status %#x after %llu us\n", s->name, s->index, r.status, static_cast<unsigned long long>(r.runUs));
    const uint32_t buffer = s->trace->bufSize(job.outBuf);
    const uint32_t length = job.outOffset + r.streamLen;
    result->stream = gDma + s->trace->bufOffset(job.outBuf);
    result->length = length <= buffer ? length : buffer;
    result->hwOffset = job.outOffset;
    result->headers = s->headerLength ? s->header : nullptr;
    result->headerLength = s->headerLength;
    if (++s->index >= s->trace->jobCount()) {
      s->index = 0;
      s->cursor = 0;
    }
  }
  IOLockUnlock(gStreamLock);
  return ok;
}

// ---- the block for the encoder core (core/VencCore.h, Rk3588VencCore.cpp): the jobs MPP's register generator makes, run as the template encoder runs a recording

extern "C" int rk3588_venc_core_run(void *, const uint32_t *regs, const VencRun *runs, unsigned nRuns, uint32_t *status) {
  __asm__ volatile("dsb sy" ::: "memory");
  // rkvenc_run: "add force clear to avoid pagefault" (VEPU580), clear the hardware counter, the registers, the start
  vencWrite(venc::kRegClear, 0x2);
  IODelay(5);
  vencWrite(venc::kRegClear, 0);
  vencWrite(venc::kRegHwCounter, 0x2);
  for (unsigned r = 0; r < nRuns; ++r)
    for (unsigned w = 0; w < runs[r].count; ++w) {
      const unsigned index = runs[r].first + w;
      if (index != venc::kRegStart) vencWrite(4 * index, regs[index]);
    }
  __asm__ volatile("dsb sy" ::: "memory");
  const uint64_t start = nowUs();
  vencWrite(venc::kRegStartOff, regs[venc::kRegStart]);
  uint32_t st = 0;
  bool done = false;
  for (;;) {
    st = vencRead(venc::kRegIntSta);
    if (st & (venc::kStaEncDone | venc::kStaErrorMask)) {
      done = (st & venc::kStaEncDone) != 0;
      break;
    }
    if (nowUs() - start > 500000) break;
    IODelay(20);
  }
  *status = st;
  if (st) vencWrite(venc::kRegIntClr, st);   // rkvenc_irq: clear what was seen
  __asm__ volatile("dsb sy" ::: "memory");
  return done && !(st & venc::kStaErrorMask) ? 0 : -1;
}

extern "C" void rk3588_venc_core_read_regs(void *, uint32_t byteOffset, uint32_t *dst, unsigned words) {
  for (unsigned i = 0; i < words; ++i) dst[i] = vencRead(byteOffset + 4 * i);
}

extern "C" void rk3588_venc_core_window(uint8_t **virt, uint64_t *bus, uint32_t *bytes) {
  *virt = gDma;
  *bus = rk3588_venc_config.dma_phys;
  *bytes = rk3588_venc_config.dma_bytes;
}

namespace {

void vencThread(void *, wait_result_t) {
  lookPower();
  if (gMode < 2) return;
  if (!powerUp()) return;
  lookBlock();
  if (!softReset()) {
    printf("rk3588-venc: the safe reset failed: pulsing the reset lines\n");
    resetLines();
  }
  lookBlock();
  if (gMode < 3) return;
  gDma = mapWindow(rk3588_venc_config.dma_phys, rk3588_venc_config.dma_bytes);
  if (!gDma) {
    printf("rk3588-venc: cannot map the DMA window (%#llx, %u bytes)\n", static_cast<unsigned long long>(rk3588_venc_config.dma_phys),
           rk3588_venc_config.dma_bytes);
    return;
  }
  printf("rk3588-venc: DMA window %#llx, %u bytes, %u traces built in\n", static_cast<unsigned long long>(rk3588_venc_config.dma_phys),
         rk3588_venc_config.dma_bytes, static_cast<unsigned>(sizeof kVencTraces / sizeof kVencTraces[0]));
  if (gMode == 5) {   // only the AppleAVE2 service uses the block (rk3588-venc-ave2=2): no self-test
    gReady = true;
    return;
  }
  if (gMode == 6) {   // the encoder core made of MPP's code (no traces): Rk3588VencCore.cpp
    rk3588_venc_core_selftest();
    lookBlock();
    return;
  }
  selfTest(gMode == 4 ? bootArg("rk3588-venc-passes", 20) : 1, gMode == 3 || bootArg("rk3588-venc-dump", 0));
  lookBlock();
}

}  // namespace

extern "C" void rk3588_venc_start(void) {
  rk3588_venc_ave2_serve();   // its own boot argument (rk3588-venc-ave2): no hardware is touched
  gMode = bootArg("rk3588-venc", 0);
  if (!gMode && bootArg("rk3588-venc-ave2-encode", 0)) gMode = 5;   // the block for the AppleAVE2 service: powered and mapped, no self-test
  if (!gMode) return;
  printf("rk3588-venc: mode %u\n", gMode);
  vm_offset_t pmuMap = ml_io_map(static_cast<vm_offset_t>(kPmuPhys), 0x400), cruMap = ml_io_map(static_cast<vm_offset_t>(kCruPhys), 0x1000);
  vm_offset_t vencMap = ml_io_map(static_cast<vm_offset_t>(kVencPhys), kVencMapSize);
  if (!pmuMap || !cruMap || !vencMap) {
    printf("rk3588-venc: cannot map PMU (%d) CRU (%d) or the encoder (%d)\n", pmuMap != 0, cruMap != 0, vencMap != 0);
    return;
  }
  gPmu = reinterpret_cast<volatile uint32_t *>(pmuMap);
  gCru = reinterpret_cast<volatile uint32_t *>(cruMap);
  gVenc = reinterpret_cast<volatile uint8_t *>(vencMap);
  thread_t thread = nullptr;
  if (kernel_thread_start(vencThread, nullptr, &thread) == KERN_SUCCESS && thread) thread_deallocate(thread);
}
