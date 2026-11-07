// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Host test of RkIommu.h and Rga3.h: the real driver code (page tables, the IOMMU's register sequences, the command register image of a copy, the job runner,
// the reset) against a strict model of the hardware written from the vendor driver's description of it, not from the driver's own header:
//   * memory: a low window (the page tables, the scratch page, the source) and high regions above 4 GiB with address bits 32..39 set (the destination pages),
//   * the IOMMU: registers, the stall rules (the directory address and the translation cache may only be touched while stalled), the walk of the two-level
//     table with its own decoder of the word format, a translation cache that only a ZAP command empties, read-only pages, the page fault,
//   * RGA3: slave mode, the command registers decoded strictly (every field the copy depends on, and a check that the ones it does not use hold the values
//     the vendor driver writes), reads and writes through the IOMMU row by row, the done and error bits, the soft reset (which also resets the IOMMU).
// The copies are compared byte by byte with a software model of the crop; pages the block touches that were not mapped for it, or writes into read-only pages,
// are errors. tests/mutate.sh breaks the driver in several ways and requires this test to fail each time.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <map>
#include <string>
#include <vector>

#include "../Rga3.h"
#include "../RkIommu.h"

namespace {

int gChecks, gFailed;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++gChecks;                                                                   \
    if (!(cond)) {                                                               \
      ++gFailed;                                                                 \
      if (gFailed < 40) printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
    }                                                                            \
  } while (0)

// ---- memory
struct Region {
  uint64_t base;
  size_t size;
  uint8_t *data;   // page aligned, like the mapped window the driver puts its tables in
  Region(uint64_t b, size_t n) : base(b), size(n), data(static_cast<uint8_t *>(aligned_alloc(4096, n))) { memset(data, 0, n); }
  Region(Region &&o) noexcept : base(o.base), size(o.size), data(o.data) { o.data = nullptr; }
  Region(const Region &) = delete;
  ~Region() { free(data); }
};

struct Memory {
  std::vector<Region> regions;
  uint8_t *at(uint64_t phys, size_t len) {
    for (Region &r : regions)
      if (phys >= r.base && phys + len <= r.base + r.size) return r.data + (phys - r.base);
    return nullptr;
  }
  uint32_t read32(uint64_t phys) {
    uint8_t *p = at(phys, 4);
    uint32_t v = 0;
    if (p) memcpy(&v, p, 4);
    else ++gFailed, printf("FAIL: the walker read outside memory at %#llx\n", (unsigned long long)phys);
    return v;
  }
};

// ---- the IOMMU model
struct MmuModel {
  Memory *mem;
  uint32_t dte = 0, intRaw = 0, intMask = 0, autoGating = 3, pageFaultAddr = 0;   // reset value 3 (read on the board): gating on, the workaround bit (31) clear
  bool paging = false, stalled = false, hadReset = false;
  struct TlbEntry {
    uint64_t phys;
    bool writable;
  };
  std::map<uint32_t, TlbEntry> tlb;   // by IO page number; only a ZAP or a reset empties it
  int errors = 0;
  std::string last;
  unsigned zaps = 0;

  void error(const char *what) {
    ++errors;
    last = what;
    ++gFailed;
    if (gFailed < 40) printf("FAIL: iommu model: %s\n", what);
  }
  // the vendor's format, decoded independently of RkIommu.h
  static uint64_t wordAddress(uint32_t w) {
    uint64_t a = w & 0xfffff000u;
    a |= (uint64_t)((w >> 8) & 0xf) << 32;
    a |= (uint64_t)((w >> 4) & 0xf) << 36;
    return a;
  }
  uint32_t status() const {
    uint32_t s = 0;
    if (paging) s |= 1;
    if (stalled) s |= 4;
    else s |= 1u << 31;
    s |= 8;   // idle
    return s;
  }
  uint32_t read(uint32_t off) {
    switch (off) {
      case rkiommu::kRegDteAddr: return dte;
      case rkiommu::kRegStatus: return status();
      case rkiommu::kRegPageFaultAddr: return pageFaultAddr;
      case rkiommu::kRegIntRaw: return intRaw;
      case rkiommu::kRegIntMask: return intMask;
      case rkiommu::kRegIntStatus: return intRaw & intMask;
      case rkiommu::kRegAutoGating: return autoGating;
      default: return 0;
    }
  }
  void reset() {
    dte = 0;
    paging = false;
    stalled = false;
    intRaw = 0;
    intMask = 0;
    autoGating = 3;
    tlb.clear();
    hadReset = true;
  }
  void write(uint32_t off, uint32_t v) {
    switch (off) {
      case rkiommu::kRegDteAddr:
        if (paging && !stalled) error("directory address written while translating and not stalled");
        dte = v;
        break;
      case rkiommu::kRegCommand:
        switch (v) {
          case rkiommu::kCmdEnableStall:
            if (paging) stalled = true;   // "stall can only be enabled if paging is enabled": otherwise ignored
            break;
          case rkiommu::kCmdDisableStall: stalled = false; break;
          case rkiommu::kCmdZapCache:
            if (paging && !stalled) error("translation cache zapped while translating and not stalled");
            tlb.clear();
            ++zaps;
            break;
          case rkiommu::kCmdEnablePaging:
            if (!dte) error("paging enabled with no directory");
            paging = true;
            break;
          case rkiommu::kCmdDisablePaging:
            if (!stalled) error("paging disabled while not stalled");
            paging = false;
            break;
          case rkiommu::kCmdForceReset:
            if (paging && !stalled) error("force reset while translating and not stalled");
            reset();
            break;
          case rkiommu::kCmdPageFaultDone: intRaw &= ~1u; break;
          default: error("unknown IOMMU command");
        }
        break;
      case rkiommu::kRegIntMask: intMask = v; break;
      case rkiommu::kRegIntClear: intRaw &= ~v; break;
      case rkiommu::kRegAutoGating: autoGating = v; break;
      default: break;
    }
  }
  // Address translation for one access of the block; false on a fault (and the fault is latched).
  bool translate(uint32_t iova, bool write, uint64_t *phys) {
    if (!paging) {
      *phys = iova;
      return true;
    }
    const uint32_t page = iova >> 12;
    auto it = tlb.find(page);
    if (it == tlb.end()) {
      const uint32_t dteWord = mem->read32(wordAddress(dte) + 4ull * (iova >> 22));
      if (!(dteWord & 1)) return fault(iova);
      const uint32_t pteWord = mem->read32(wordAddress(dteWord) + 4ull * ((iova >> 12) & 0x3ff));
      if (!(pteWord & 1)) return fault(iova);
      it = tlb.emplace(page, TlbEntry{wordAddress(pteWord), (pteWord & 4) != 0}).first;
      if (!(pteWord & 2)) return fault(iova);   // not readable: the model never allows it
    }
    if (write && !it->second.writable) return fault(iova);
    *phys = it->second.phys | (iova & 0xfff);
    return true;
  }
  bool fault(uint32_t iova) {
    pageFaultAddr = iova;
    intRaw |= 1;
    return false;
  }
};

// ---- the RGA3 model
struct RgaModel {
  Memory *mem;
  MmuModel *mmu;
  uint32_t sys = 0, cmdAddr = 0, cmdCtrl = 0, intEn = 0, intRaw = 0;
  uint32_t cmd[rga3::kCmdWords] = {};
  bool hang = false;      // never finishes
  bool errorWithDone = false;   // finishes (frame done up) but with a window error bit up as well
  unsigned jobs = 0, resets = 0;
  std::map<uint32_t, int> touchedPages;   // IO pages the block read (1) or wrote (2) with the IOMMU on
  std::string error;      // the first thing the model did not like

  void fail(const std::string &what) {
    if (error.empty()) error = what;
    ++gFailed;
    if (gFailed < 40) printf("FAIL: rga model: %s\n", what.c_str());
  }
  uint32_t read(uint32_t off) {
    if (off >= rga3::kIommuBase) return mmu->read(off - rga3::kIommuBase);
    if (off >= rga3::kCmdBase && off < rga3::kCmdBase + 4 * rga3::kCmdWords) return cmd[(off - rga3::kCmdBase) / 4];
    switch (off) {
      case rga3::kSysCtrl: return sys;
      case rga3::kIntEn: return intEn;
      case rga3::kIntRaw: return intRaw;
      case rga3::kRoSrst: return (sys & (rga3::kSysCclkReset | rga3::kSysAclkReset)) ? 0x3f : 0;
      case rga3::kVersion: return 0x03000000;
      default: return 0;
    }
  }
  void write(uint32_t off, uint32_t v) {
    if (off >= rga3::kIommuBase) {
      mmu->write(off - rga3::kIommuBase, v);
      return;
    }
    if (off >= rga3::kCmdBase && off < rga3::kCmdBase + 4 * rga3::kCmdWords) {
      cmd[(off - rga3::kCmdBase) / 4] = v;
      return;
    }
    switch (off) {
      case rga3::kSysCtrl: {
        const bool wasReset = (sys & (rga3::kSysCclkReset | rga3::kSysAclkReset)) != 0;
        sys = v;
        if ((v & (rga3::kSysCclkReset | rga3::kSysAclkReset)) && !wasReset) {
          ++resets;
          mmu->reset();   // the IOMMU is in the block's reset domain
          intRaw = 0;
        }
        if (v & rga3::kSysStart) start();
        break;
      }
      case rga3::kCmdAddr: cmdAddr = v; break;
      case rga3::kCmdCtrl: cmdCtrl = v; break;
      case rga3::kIntEn: intEn = v; break;
      case rga3::kIntClear: intRaw &= ~v; break;
      default: break;
    }
  }

  uint32_t reg(uint32_t off) const { return cmd[off / 4]; }

  // A row of `len` bytes at IO address `iova`, read or written page by page through the IOMMU.
  bool access(uint32_t iova, uint32_t len, bool write, uint8_t *buf) {
    uint32_t done = 0;
    while (done < len) {
      const uint32_t at = iova + done, room = 0x1000 - (at & 0xfff), n = len - done < room ? len - done : room;
      uint64_t phys;
      if (!mmu->translate(at, write, &phys)) return false;
      if (mmu->paging) touchedPages[at >> 12] |= write ? 2 : 1;
      uint8_t *p = mem->at(phys, n);
      if (!p) {
        fail("access outside memory");
        return false;
      }
      if (write) memcpy(p, buf + done, n);
      else memcpy(buf + done, p, n);
      done += n;
    }
    return true;
  }

  void start() {
    ++jobs;
    if (hang) return;
    if (!(sys & rga3::kSysLgcClkOn)) fail("started without the logic clock on");
    if (sys & rga3::kSysCmdMode) fail("master mode: the copy is meant to run in slave mode");
    if (!(sys & rga3::kSysFrmendAutoReset)) fail("frame-end auto reset not set (the vendor driver sets it)");
    if (cmdAddr || cmdCtrl) fail("command address or control not zero in slave mode");
    if (!(intEn & rga3::kIntFrameDone)) fail("frame-done interrupt not enabled");
    // window 0 as the vendor driver writes it for a 1:1 NV12 raster read
    const uint32_t rd = reg(rga3::kWin0RdCtrl);
    if (!(rd & 1)) fail("window 0 not enabled");
    if (((rd >> 1) & 3) != 0) fail("window 0 not in raster mode");
    if (((rd >> 4) & 0xf) != 0) fail("window 0 format is not YUV 4:2:0 8 bit");
    if (((rd >> 8) & 3) != 1) fail("window 0 is not semi-planar");
    if (rd & ((1u << 12) | (1u << 13) | (1u << 16) | (1u << 17) | (1u << 18))) fail("window 0 swaps, rotates or mirrors");
    if (!(rd & (1u << 20)) || !(rd & (1u << 22)) || (rd & ((1u << 21) | (1u << 23)))) fail("window 0 scales");
    if (rd & ((1u << 24) | (1u << 25))) fail("window 0 converts colours");
    if (reg(rga3::kWin0SclFac) != 0) fail("scale factors not zero");
    if (reg(rga3::kWin0VBase) != 0) fail("a V plane for NV12");
    const uint32_t y0 = reg(rga3::kWin0YBase), uv0 = reg(rga3::kWin0UBase), stride = reg(rga3::kWin0VirStride) * 4, uvStride = reg(rga3::kWin0UvVirStride) * 4;
    if (stride != uvStride) fail("the two source pitches differ");
    if (stride & 15) fail("source pitch not a multiple of 16");
    const uint32_t x = reg(rga3::kWin0ActOff) & 0xffff, y = reg(rga3::kWin0ActOff) >> 16;
    const uint32_t w = reg(rga3::kWin0ActSize) & 0xffff, h = reg(rga3::kWin0ActSize) >> 16;
    if (reg(rga3::kWin0DstSize) != reg(rga3::kWin0ActSize)) fail("window 0 destination size differs from its source size");
    if (reg(rga3::kWin0SrcSize) != (rga3::align16(w + x) | (rga3::align16(h + y) << 16))) fail("source size is not the aligned extent of the crop");
    if ((x | y | w | h) & 1) fail("odd crop for a 4:2:0 picture");
    if (w < rga3::kMinWidth || h < rga3::kMinHeight || w > rga3::kMaxOutput || h > rga3::kMaxOutput) fail("size outside the block's range");
    // window 1 off, in the state of an unused window
    if (reg(rga3::kWin1RdCtrl) & 1) fail("window 1 enabled");
    if (reg(rga3::kWin1RdCtrl) != ((1u << 8) | (1u << 10) | (1u << 20) | (1u << 22))) fail("window 1 register image differs from the vendor's unused window");
    for (uint32_t off = rga3::kWin1RdCtrl + 4; off < rga3::kOvlpCtrl; off += 4)
      if (reg(off)) fail("a window 1 register is not zero");
    // overlap: a straight copy
    if (reg(rga3::kOvlpCtrl) != (1u << 2) || reg(rga3::kOvlpBotCtrl) != (0xffu << 16) || reg(rga3::kOvlpOff) || reg(rga3::kOvlpTopCtrl) || reg(rga3::kOvlpTopAlpha) ||
        reg(rga3::kOvlpBotAlpha))
      fail("overlap registers are not those of a straight copy");
    // write-back
    const uint32_t wr = reg(rga3::kWrCtrl);
    if (((wr >> 4) & 0xf) != 0 || ((wr >> 8) & 3) != 1 || (wr & 3) != 0 || (wr & ((1u << 12) | (1u << 20)))) fail("write-back is not raster NV12");
    if (((wr >> 13) & 0x3f) != 0xf) fail("outstanding limit is not the vendor's 15");
    const uint32_t wstride = reg(rga3::kWrVirStride) * 4, wuvStride = reg(rga3::kWrPlVirStride) * 4;
    if (wstride != wuvStride || (wstride & 15)) fail("write-back pitches");
    if (reg(rga3::kWrVBase) != 0) fail("a V plane written for NV12");
    const uint32_t wy = reg(rga3::kWrYBase), wuv = reg(rga3::kWrUBase);
    if (!error.empty()) {
      intRaw |= 1u << 7;   // a window error bit: the block would not like it either
      return;
    }
    // the copy
    std::vector<uint8_t> row(w);
    bool ok = true;
    for (uint32_t r = 0; r < h && ok; ++r) {
      ok = access(y0 + (y + r) * stride + x, w, false, row.data());
      if (ok) ok = access(wy + r * wstride, w, true, row.data());
    }
    for (uint32_t r = 0; r < h / 2 && ok; ++r) {
      ok = access(uv0 + (y / 2 + r) * stride + x, w, false, row.data());
      if (ok) ok = access(wuv + r * wstride, w, true, row.data());
    }
    if (!ok) {
      intRaw |= rga3::kIntMmu;
      return;
    }
    intRaw |= rga3::kIntFrameDone | rga3::kIntCmdLineFinish;
    if (errorWithDone) intRaw |= 1u << 7;
  }
};

// ---- glue between the model and the drivers' accessors
uint64_t gNow;
struct Bench {
  Memory mem;
  MmuModel mmu;
  RgaModel rga;
  rkiommu::PageTables tables;
  Bench() {
    mem.regions.emplace_back(0x66000000ull, 8u << 20);
    mem.regions.emplace_back(0x100000000ull, 4u << 20);     // address bit 32
    mem.regions.emplace_back(0x800000000ull, 4u << 20);     // bit 35
    mem.regions.emplace_back(0x1000000000ull, 4u << 20);    // bit 36
    mem.regions.emplace_back(0xf000000000ull, 4u << 20);    // bits 39..36
    mmu.mem = &mem;
    rga.mem = &mem;
    rga.mmu = &mmu;
  }
};
Bench *gBench;

uint32_t rgaRead(void *, uint32_t off) { return gBench->rga.read(off); }
void rgaWrite(void *, uint32_t off, uint32_t v) { gBench->rga.write(off, v); }
uint32_t mmuRead(void *, uint32_t off) { return gBench->mmu.read(off); }
void mmuWrite(void *, uint32_t off, uint32_t v) { gBench->mmu.write(off, v); }
// The model's clock only moves when a driver waits, so a wait that never ends would loop for ever: stop it (a driver that cannot give up is a failure).
void runaway() {
  if (gNow > 100000000ull) {
    printf("FAIL: a wait did not end within 100 s of model time\n");
    exit(1);
  }
}
void delayUs(void *, uint32_t us) { gNow += us; runaway(); }
uint64_t nowUs(void *) { return ++gNow; }
const rga3::Regs kRga = {nullptr, rgaRead, rgaWrite, delayUs, nowUs};
const rkiommu::Regs kMmu = {nullptr, mmuRead, mmuWrite, delayUs};

uint8_t pattern(uint32_t seed, size_t i) {
  uint32_t x = seed + (uint32_t)i * 2654435761u;
  x ^= x >> 15;
  x *= 2246822519u;
  x ^= x >> 13;
  return (uint8_t)x;
}

// ---- tests of the page tables on their own
void testEncoding() {
  // a few addresses, against the vendor format
  CHECK(rkiommu::makeDte(0x66000000ull) == 0x66000001u);
  CHECK(rkiommu::makePte(0x66001000ull, true) == (0x66001000u | 7));
  CHECK(rkiommu::makePte(0x66001000ull, false) == (0x66001000u | 3));
  CHECK(rkiommu::makePte(0x100000000ull, true) == (0x00000100u | 7));       // bit 32 lands in bit 8
  CHECK(rkiommu::makePte(0x800000000ull, true) == (0x00000800u | 7));       // bit 35 in bit 11
  CHECK(rkiommu::makePte(0x1000000000ull, true) == (0x00000010u | 7));      // bit 36 in bit 4
  CHECK(rkiommu::makePte(0xf000000000ull, true) == (0x000000f0u | 7));      // bits 39..36 in 7..4
  for (uint64_t pa : {0x1000ull, 0x66abc000ull, 0x123456000ull, 0x7fffff000ull, 0xfeedbeef000ull & ((1ull << 40) - 1), 0xffffffffff000ull & ((1ull << 40) - 1)}) {
    const uint32_t w = rkiommu::makePte(pa, true);
    CHECK(rkiommu::decodeAddress(w) == pa);
    CHECK(MmuModel::wordAddress(w) == pa);   // the model's independent decoder agrees
    CHECK(rkiommu::isValid(w) && rkiommu::isWritable(w));
  }
}

void testPageTables() {
  std::vector<uint8_t> raw(64 * 4096 + 4096);
  uint8_t *aligned = (uint8_t *)(((uintptr_t)raw.data() + 4095) & ~(uintptr_t)4095);
  rkiommu::PageTables t;
  CHECK(!t.init(aligned + 1, 0x66000000, 16 * 4096));        // virt not aligned
  CHECK(!t.init(aligned, 0x66000800, 16 * 4096));            // bus not aligned
  CHECK(!t.init(aligned, 0x66000000, 4096));                 // no room for a table
  CHECK(t.init(aligned, 0x66000000, 8 * 4096));
  CHECK(t.pagesUsed() == 1);
  uint64_t phys = 0;
  bool w = false;
  CHECK(!t.lookup(0x20000000, &phys));
  CHECK(t.map(0x20000000, 0x100000000ull, 3 * 4096, true));
  CHECK(t.pagesUsed() == 2);                                  // directory + one table
  CHECK(t.lookup(0x20000000, &phys, &w) && phys == 0x100000000ull && w);
  CHECK(t.lookup(0x20002fff, &phys, &w) && phys == 0x100002fffull);
  CHECK(!t.lookup(0x20003000, &phys));
  CHECK(!t.map(0x20001000, 0x66000000, 4096, true));          // already mapped
  CHECK(t.map(0x20003000, 0x66002000, 4096, false));
  CHECK(t.lookup(0x20003000, &phys, &w) && phys == 0x66002000 && !w);
  t.unmap(0x20001000, 2 * 4096);
  CHECK(t.lookup(0x20000000, &phys) && !t.lookup(0x20001000, &phys) && !t.lookup(0x20002000, &phys) && t.lookup(0x20003000, &phys));
  CHECK(t.map(0x20001000, 0x800000000ull, 2 * 4096, true));  // reusable after the unmap
  CHECK(t.lookup(0x20002000, &phys) && phys == 0x800001000ull);
  CHECK(t.map(0x30000000, 0x1000000000ull, 4096, true) && t.pagesUsed() == 3);   // another 4 MiB: another table
  CHECK(t.mapSame(0x30001000, 0x66001000, 4, true));
  for (uint32_t i = 0; i < 4; ++i) CHECK(t.lookup(0x30001000 + i * 4096, &phys) && phys == 0x66001000);
  CHECK(!t.map(0x30010000, 0x1ull << 40, 4096, true));        // beyond 40 bits
  CHECK(!t.map(0xfffff000u, 0x66000000, 2 * 4096, true));     // beyond the 32-bit space
  CHECK(!t.map(0x20000000 + 1, 0x66000000, 4096, true));      // not aligned
  // out of tables: 8 pages, 3 used so far; four more 4 MiB areas take four tables (7 used), a fifth the last page, a sixth finds none
  CHECK(t.map(0x40000000, 0x66000000, 4096, true));
  CHECK(t.map(0x40400000, 0x66000000, 4096, true));
  CHECK(t.map(0x40800000, 0x66000000, 4096, true));
  CHECK(t.map(0x40c00000, 0x66000000, 4096, true));
  CHECK(t.pagesUsed() == 7);
  CHECK(t.map(0x41000000, 0x66000000, 4096, true) && t.pagesUsed() == 8);
  CHECK(!t.map(0x41400000, 0x66000000, 4096, true));
  CHECK(!t.lookup(0x41400000, &phys));
}

// ---- tests with the model
struct Picture {
  uint32_t seed;
  size_t bytes;
};

struct CaseDef {
  const char *name;
  uint32_t w, h, srcStride, srcRows, srcX, srcRow, dstStride;
};
const CaseDef kCases[] = {
    {"320x240", 320, 240, 320, 240, 0, 0, 320},
    {"cropped, wider pitch", 304, 224, 384, 256, 16, 8, 512},
    {"720p", 1280, 720, 1280, 720, 0, 0, 1280},
    {"720p padded", 1280, 720, 1344, 736, 0, 0, 1408},
    {"narrowest", 68, 64, 80, 64, 0, 0, 80},
    {"even crop offsets", 200, 100, 256, 128, 22, 14, 256},
};

// Source in the low window (read-only IO mapping), destination scattered: page i of the IO range -> a page of a high region, in a shuffled order.
struct Mapped {
  uint32_t srcIova, dstIova;
  std::vector<uint64_t> dstPages;
};

Mapped mapCase(Bench &b, const CaseDef &c, uint32_t dstBytes, uint32_t srcIovaBase, uint32_t dstIovaBase) {
  Mapped m = {};
  const uint32_t srcBytes = c.srcStride * c.srcRows * 3 / 2;
  CHECK(b.tables.map(srcIovaBase, 0x66100000ull, srcBytes, false));
  m.srcIova = srcIovaBase;
  m.dstIova = dstIovaBase;
  const uint64_t bases[] = {0x100000000ull, 0x800000000ull, 0x1000000000ull, 0xf000000000ull};
  const uint32_t pages = (dstBytes + 4095) / 4096;
  for (uint32_t i = 0; i < pages; ++i) {
    // a different region for neighbouring pages, and a scrambled slot inside the region (4 MiB = 1024 pages; i * 389 mod 1024 is a permutation)
    const uint64_t pa = bases[i % 4] + (uint64_t)((i * 389u) % 1024) * 4096;
    m.dstPages.push_back(pa);
    CHECK(b.tables.map(dstIovaBase + i * 4096, pa, 4096, true));
  }
  return m;
}

// What the destination buffer must hold, byte for byte, read through the page list.
bool compareDestination(Bench &b, const CaseDef &c, const Mapped &m, uint32_t yOff, uint32_t uvOff, uint32_t seed, uint32_t dstBytes) {
  const size_t chromaStart = (size_t)c.srcStride * c.srcRows;
  auto byteAt = [&](uint32_t off) {
    uint8_t *p = b.mem.at(m.dstPages[off / 4096] + (off & 0xfff), 1);
    return p ? *p : (uint8_t)0x00;
  };
  size_t bad = 0;
  for (uint32_t off = 0; off < dstBytes; ++off) {
    uint8_t expected = 0xa5;
    if (off >= yOff && off < yOff + c.dstStride * c.h) {
      const uint32_t r = (off - yOff) / c.dstStride, x = (off - yOff) % c.dstStride;
      if (x < c.w) expected = pattern(seed, (size_t)(c.srcRow + r) * c.srcStride + c.srcX + x);
    } else if (off >= uvOff && off < uvOff + c.dstStride * (c.h / 2)) {
      const uint32_t r = (off - uvOff) / c.dstStride, x = (off - uvOff) % c.dstStride;
      if (x < c.w) expected = pattern(seed, chromaStart + (size_t)(c.srcRow / 2 + r) * c.srcStride + c.srcX + x);
    }
    if (byteAt(off) != expected) ++bad;
  }
  return bad == 0;
}

void fillDestination(Bench &b, const Mapped &m, uint32_t bytes) {
  for (uint32_t off = 0; off < bytes; ++off) *b.mem.at(m.dstPages[off / 4096] + (off & 0xfff), 1) = 0xa5;
}

void setUpIommu(Bench &b) {
  CHECK(b.tables.init(b.mem.at(0x66000000ull, 0x80000), 0x66000000ull, 0x80000));
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  CHECK(b.mmu.paging && !b.mmu.stalled);
  CHECK(b.mmu.dte == rkiommu::makeDte(0x66000000ull));
}

void testCopiesThroughIommu() {
  for (const CaseDef &c : kCases) {
    Bench b;
    gBench = &b;
    setUpIommu(b);
    const uint32_t srcBytes = c.srcStride * c.srcRows * 3 / 2;
    // the source picture in the low window, 1 MiB in
    for (uint32_t i = 0; i < srcBytes; ++i) *b.mem.at(0x66100000ull + i, 1) = pattern(0x77aa, i);
    // layout of the destination like an IOSurface: luma a page in, chroma on the next page boundary after it
    const uint32_t yOff = 0x1000, uvOff = (yOff + c.dstStride * c.h + 0xfff) & ~0xfffu, dstBytes = (uvOff + c.dstStride * (c.h / 2) + 0x1000 + 0xffff) & ~0xffffu;
    Mapped m = mapCase(b, c, dstBytes, 0x20000000, 0x20400000);
    CHECK(rkiommu::invalidate(kMmu));
    fillDestination(b, m, dstBytes);
    rga3::Copy copy = {m.srcIova, m.srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, m.dstIova + yOff, m.dstIova + uvOff, c.dstStride, c.w, c.h};
    CHECK(rga3::check(copy) == nullptr);
    uint32_t cmd[rga3::kCmdWords];
    rga3::build(copy, cmd);
    rga3::Result res = {};
    CHECK(rga3::run(kRga, cmd, 1000, &res));
    CHECK(res.done);
    CHECK(b.rga.error.empty());
    CHECK(compareDestination(b, c, m, yOff, uvOff, 0x77aa, dstBytes));
    // a second run into the same buffers (a new picture in the same surface)
    for (uint32_t i = 0; i < srcBytes; ++i) *b.mem.at(0x66100000ull + i, 1) = pattern(0x1234, i);
    fillDestination(b, m, dstBytes);
    CHECK(rga3::run(kRga, cmd, 1000, &res));
    CHECK(compareDestination(b, c, m, yOff, uvOff, 0x1234, dstBytes));
    // the block only touched what was mapped for it: pages of the source it read (1), of the destination it wrote (2), nothing else
    for (auto &t : b.rga.touchedPages) {
      const uint32_t iova = t.first << 12;
      const bool inSrc = iova >= m.srcIova && iova < m.srcIova + ((srcBytes + 4095) & ~4095u);
      const bool inDst = iova >= m.dstIova && iova < m.dstIova + dstBytes;
      CHECK(inSrc || inDst);
      if (inSrc) CHECK(t.second == 1);
      if (inDst) CHECK(t.second == 2);
    }
    CHECK(b.mmu.errors == 0);
  }
}

// Unmapping and mapping the same IO addresses to other pages: a stale translation would send the copy to the old pages. The driver zaps the cache after each
// change (rkiommu::invalidate); the model's translation cache only forgets on a zap, so skipping it shows.
void testRemapNeedsZap() {
  const CaseDef &c = kCases[0];
  Bench b;
  gBench = &b;
  setUpIommu(b);
  const uint32_t srcBytes = c.srcStride * c.srcRows * 3 / 2;
  for (uint32_t i = 0; i < srcBytes; ++i) *b.mem.at(0x66100000ull + i, 1) = pattern(0x99, i);
  const uint32_t yOff = 0x1000, uvOff = (yOff + c.dstStride * c.h + 0xfff) & ~0xfffu, dstBytes = (uvOff + c.dstStride * (c.h / 2) + 0x1000 + 0xffff) & ~0xffffu;
  Mapped first = mapCase(b, c, dstBytes, 0x20000000, 0x20400000);
  CHECK(rkiommu::invalidate(kMmu));
  rga3::Copy copy = {first.srcIova, first.srcIova + c.srcStride * c.srcRows, c.srcStride, 0, 0, first.dstIova + yOff, first.dstIova + uvOff, c.dstStride, c.w, c.h};
  uint32_t cmd[rga3::kCmdWords];
  rga3::build(copy, cmd);
  fillDestination(b, first, dstBytes);
  CHECK(rga3::run(kRga, cmd, 1000, nullptr));
  CHECK(compareDestination(b, c, first, yOff, uvOff, 0x99, dstBytes));
  // unmap the destination, map the same IO range to a different set of pages (shifted slots), and copy again
  b.tables.unmap(first.dstIova, dstBytes);
  CHECK(rkiommu::invalidate(kMmu));
  Mapped second = {};
  second.srcIova = first.srcIova;
  second.dstIova = first.dstIova;
  for (uint32_t i = 0; i < dstBytes / 4096; ++i) {
    const uint64_t pa = 0x800000000ull + (uint64_t)(((i * 389u) + 17) % 1024) * 4096;
    second.dstPages.push_back(pa);
    CHECK(b.tables.map(second.dstIova + i * 4096, pa, 4096, true));
  }
  CHECK(rkiommu::invalidate(kMmu));
  fillDestination(b, second, dstBytes);
  CHECK(rga3::run(kRga, cmd, 1000, nullptr));
  CHECK(compareDestination(b, c, second, yOff, uvOff, 0x99, dstBytes));
}

void testPhysicalWithoutIommu() {
  const CaseDef &c = kCases[1];
  Bench b;
  gBench = &b;
  CHECK(rkiommu::disable(kMmu));   // already off: nothing to do
  CHECK(!b.mmu.paging);
  const uint32_t srcBytes = c.srcStride * c.srcRows * 3 / 2;
  for (uint32_t i = 0; i < srcBytes; ++i) *b.mem.at(0x66100000ull + i, 1) = pattern(0x31, i);
  const uint32_t yOff = 0x1000, uvOff = (yOff + c.dstStride * c.h + 0xfff) & ~0xfffu;
  rga3::Copy copy = {0x66100000u, 0x66100000u + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, 0x66400000u + yOff, 0x66400000u + uvOff, c.dstStride, c.w, c.h};
  CHECK(rga3::check(copy) == nullptr);
  uint32_t cmd[rga3::kCmdWords];
  rga3::build(copy, cmd);
  memset(b.mem.at(0x66400000ull, 0x100000), 0xa5, 0x100000);
  CHECK(rga3::run(kRga, cmd, 1000, nullptr));
  const uint8_t *dst = b.mem.at(0x66400000ull, 0x100000);
  size_t bad = 0;
  const size_t chromaStart = (size_t)c.srcStride * c.srcRows;
  for (uint32_t r = 0; r < c.h; ++r)
    for (uint32_t x = 0; x < c.w; ++x)
      if (dst[yOff + r * c.dstStride + x] != pattern(0x31, (size_t)(c.srcRow + r) * c.srcStride + c.srcX + x)) ++bad;
  for (uint32_t r = 0; r < c.h / 2; ++r)
    for (uint32_t x = 0; x < c.w; ++x)
      if (dst[uvOff + r * c.dstStride + x] != pattern(0x31, chromaStart + (size_t)(c.srcRow / 2 + r) * c.srcStride + c.srcX + x)) ++bad;
  CHECK(bad == 0);
  // the pad bytes of the rows and the page before the luma plane stayed as they were
  CHECK(dst[0] == 0xa5 && dst[yOff - 1] == 0xa5 && dst[yOff + c.w] == 0xa5 && dst[yOff + c.dstStride * c.h - 1] == 0xa5 && dst[uvOff + c.dstStride * (c.h / 2)] == 0xa5);
}

void testRejects() {
  rga3::Copy ok = {0x20000000, 0x20100000, 1280, 0, 0, 0x20400000, 0x20500000, 1280, 1280, 720};
  CHECK(rga3::check(ok) == nullptr);
  auto bad = [&](rga3::Copy c) { return rga3::check(c) != nullptr; };
  rga3::Copy c = ok;
  c.srcY = 0;
  CHECK(bad(c));
  c = ok;
  c.dstUv = 0;
  CHECK(bad(c));
  c = ok;
  c.srcY += 8;
  CHECK(bad(c));                    // not 16-byte aligned
  c = ok;
  c.dstStride = 1288;
  CHECK(bad(c));                    // pitch not a multiple of 16
  c = ok;
  c.srcStride = 0;
  CHECK(bad(c));
  c = ok;
  c.dstStride = 32768 + 16;
  c.width = 1280;
  CHECK(bad(c));
  c = ok;
  c.width = 1279;
  CHECK(bad(c));                    // odd
  c = ok;
  c.height = 721;
  CHECK(bad(c));
  c = ok;
  c.srcX = 1;
  CHECK(bad(c));
  c = ok;
  c.srcRow = 3;
  CHECK(bad(c));
  c = ok;
  c.width = 66;
  CHECK(bad(c));                    // narrower than 68
  c = ok;
  c.height = 0;
  CHECK(bad(c));
  c = ok;
  c.width = 8130;
  c.srcStride = c.dstStride = 8192;
  CHECK(bad(c));                    // above the output limit
  c = ok;
  c.srcX = 16;
  CHECK(bad(c));                    // the crop does not fit in the source pitch
  c = ok;
  c.width = 1300;
  c.dstStride = 1296;
  c.srcStride = 1312;
  CHECK(bad(c));                    // a row wider than the destination pitch
  // the limit values themselves pass
  c = ok;
  c.width = 68;
  c.height = 2;
  c.srcStride = c.dstStride = 80;
  CHECK(rga3::check(c) == nullptr);
  c = ok;
  c.width = 8128;
  c.height = 8128;
  c.srcStride = c.dstStride = 8128;
  CHECK(rga3::check(c) == nullptr);
}

void testFaultAndRecovery() {
  const CaseDef &c = kCases[1];
  Bench b;
  gBench = &b;
  setUpIommu(b);
  const uint32_t srcBytes = c.srcStride * c.srcRows * 3 / 2;
  for (uint32_t i = 0; i < srcBytes; ++i) *b.mem.at(0x66100000ull + i, 1) = pattern(0x55, i);
  const uint32_t yOff = 0x1000, uvOff = (yOff + c.dstStride * c.h + 0xfff) & ~0xfffu, dstBytes = (uvOff + c.dstStride * (c.h / 2) + 0x1000 + 0xffff) & ~0xffffu;
  Mapped m = mapCase(b, c, dstBytes, 0x20000000, 0x20400000);
  CHECK(rkiommu::invalidate(kMmu));
  // a destination that is not mapped: a page fault
  rga3::Copy bad = {m.srcIova, m.srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, 0x2ffc0000u, 0x30000000u, c.dstStride, c.w, c.h};
  uint32_t cmd[rga3::kCmdWords];
  rga3::build(bad, cmd);
  rga3::Result res = {};
  CHECK(!rga3::run(kRga, cmd, 1000, &res));
  CHECK(res.intRaw & rga3::kIntMmu);
  CHECK(b.mmu.pageFaultAddr == 0x2ffc0000u);
  // the driver's recovery: reset the block, bring the IOMMU up again with the same tables
  CHECK(rga3::softReset(kRga));
  CHECK(b.rga.resets == 1 && !b.mmu.paging);       // the reset took the IOMMU with it
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  CHECK(b.mmu.paging);
  // the next copy works
  rga3::Copy good = {m.srcIova, m.srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, m.dstIova + yOff, m.dstIova + uvOff, c.dstStride, c.w, c.h};
  rga3::build(good, cmd);
  fillDestination(b, m, dstBytes);
  CHECK(rga3::run(kRga, cmd, 1000, &res));
  CHECK(compareDestination(b, c, m, yOff, uvOff, 0x55, dstBytes));
  // a write into a read-only page faults too: point the destination at the source mapping
  rga3::Copy readOnly = {m.srcIova, m.srcIova + c.srcStride * c.srcRows, c.srcStride, c.srcX, c.srcRow, m.srcIova, m.srcIova + 0x8000, c.dstStride, c.w, c.h};
  rga3::build(readOnly, cmd);
  CHECK(!rga3::run(kRga, cmd, 1000, &res));
  CHECK(res.intRaw & rga3::kIntMmu);
  CHECK(rga3::softReset(kRga));
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  // a block that never finishes: the runner gives up at the limit, with no frame-done bit
  b.rga.errorWithDone = true;
  rga3::build(good, cmd);
  CHECK(!rga3::run(kRga, cmd, 1000, &res));   // frame done and an error bit together is a failed job
  CHECK(res.done && (res.intRaw & rga3::kIntErrors));
  b.rga.errorWithDone = false;
  CHECK(rga3::softReset(kRga));
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  b.rga.hang = true;
  rga3::build(good, cmd);
  const uint64_t t0 = gNow;
  CHECK(!rga3::run(kRga, cmd, 500, &res));
  CHECK(!res.done && gNow - t0 >= 500 && gNow - t0 < 1000);
  b.rga.hang = false;
  CHECK(rga3::softReset(kRga));
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  CHECK(rga3::run(kRga, cmd, 1000, &res));
}

void testEnableDisable() {
  Bench b;
  gBench = &b;
  // from cold, with paging off
  setUpIommu(b);
  CHECK(b.mmu.hadReset && b.mmu.errors == 0);
  // enabling again while translating (the stall rules)
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  CHECK(b.mmu.paging && !b.mmu.stalled && b.mmu.errors == 0);
  CHECK(rkiommu::invalidate(kMmu));
  CHECK(b.mmu.errors == 0);
  CHECK(rkiommu::disable(kMmu));
  CHECK(!b.mmu.paging && !b.mmu.stalled && b.mmu.errors == 0);
  CHECK(rkiommu::invalidate(kMmu));   // paging off: no stall needed
  CHECK(b.mmu.errors == 0);
  // the interrupt mask the vendor driver sets and the auto-gating workaround bit
  CHECK(rkiommu::enable(kMmu, b.tables.directoryBus()));
  CHECK(b.mmu.intMask == 3 && (b.mmu.autoGating & 0x80000000u) && (b.mmu.autoGating & 3) == 3);   // set, and the bits already there kept
}

void testBuiltRegisters() {
  // The image of one copy, register by register (the numbers the vendor driver writes for a plain NV12 copy).
  rga3::Copy c = {0x20001000, 0x20100000, 1280, 16, 8, 0x30002000, 0x30102000, 1408, 1200, 700};
  uint32_t cmd[rga3::kCmdWords];
  rga3::build(c, cmd);
  auto reg = [&](uint32_t off) { return cmd[off / 4]; };
  CHECK(reg(0x00) == 0x00500501);
  CHECK(reg(0x10) == 0x20001000 && reg(0x14) == 0x20100000 && reg(0x18) == 0);
  CHECK(reg(0x1c) == 320 && reg(0x38) == 320);                              // 1280 / 4
  CHECK(reg(0x24) == (1216u | (720u << 16)));                                // 16-aligned 1200 + 16, 700 + 8
  CHECK(reg(0x28) == (16u | (8u << 16)));
  CHECK(reg(0x2c) == (1200u | (700u << 16)) && reg(0x30) == reg(0x2c));
  CHECK(reg(0x34) == 0);
  CHECK(reg(0x40) == 0x00500500);
  CHECK(reg(0x80) == 4 && reg(0x94) == 0x00ff0000);
  CHECK(reg(0xa0) == ((1u << 8) | (1u << 2) | (1u << 10) | (0xfu << 13)));
  CHECK(reg(0xa8) == 352 && reg(0xac) == 352);                              // 1408 / 4
  CHECK(reg(0xb0) == 0x30002000 && reg(0xb4) == 0x30102000 && reg(0xb8) == 0);
  CHECK(rga3::kCmdBase + 4 * rga3::kCmdWords == 0x1c0);                      // 0x100 .. 0x1bc
  // every register not named above is zero
  for (uint32_t off = 0; off < 4 * rga3::kCmdWords; off += 4) {
    switch (off) {
      case 0x00: case 0x10: case 0x14: case 0x1c: case 0x24: case 0x28: case 0x2c: case 0x30: case 0x38: case 0x40: case 0x80: case 0x94: case 0xa0: case 0xa8: case 0xac:
      case 0xb0: case 0xb4: break;
      default: CHECK(reg(off) == 0);
    }
  }
  CHECK(rga3::kIntErrors == 0x3f3fcee6u);
}

}  // namespace

int main() {
  testEncoding();
  testPageTables();
  testBuiltRegisters();
  testRejects();
  testEnableDisable();
  testPhysicalWithoutIommu();
  testCopiesThroughIommu();
  testRemapNeedsZap();
  testFaultAndRecovery();
  printf("%d checks, %d failed\n", gChecks, gFailed);
  return gFailed ? 1 : 0;
}
