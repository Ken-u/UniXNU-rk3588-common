// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VdecTrace.h -- the decode-job trace the rkvdec2 self-test replays, and the code that replays it. A pure computation over a register accessor and a DMA
// window (like XhciCore.h), so tests/test-vdec.cpp runs it against a model of the block; Rk3588Vdec.cpp supplies the real ones.
//
// A trace is what Rockchip's unmodified user-space library (MPP: the H.264 parser and the vdpu34x register generator) asks the kernel driver to run for
// a stream, recorded on a host with tools/trace-shim.c and packed by tools/make-trace.py: for every decode job the register image, the registers that hold
// buffer addresses (which buffer, which offset), the pages of the CPU-written buffers (stream, tables, parameter sets) that changed, and the CRC-32 of the
// picture the job must produce (taken from a bit-exact software decoder, libavc). Replaying it is what vendor kernel's rkvdec2_run() does for one task:
// the same register writes in the same order, start last, then wait for the interrupt status. If the pictures come out with the right CRC the block, its
// power, clocks, resets and the bus path to memory work, and the register generator of MPP is a correct description of the job.
//
// Binary layout (little endian words; make-trace.py writes it):
//   'VDT1', width, height, nBuf, nJob, then nBuf words: size of buffer 1..nBuf (0 = unused)
//   per job:
//     regs[360]                                  the register image (address registers hold garbage: the relocations below replace them)
//     nRun, nRun x { first, count }              the register ranges to write, in this order (the start register, index 10, is written last)
//     nReloc, nReloc x { reg, buf, offset }      register `reg` = bus address of buffer `buf` + offset
//     nUpload, nUpload x { buf, offset, len } and the bytes (padded to four)   CPU-written content to copy into the buffers before the run
//     outBuf, outOffset, outLen, outCrc          the picture (NV12) and the CRC-32 of it
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace vdec {

constexpr unsigned kRegCount = 360;      // RKVDEC_REG_NUM
constexpr unsigned kRegStart = 10;       // RKVDEC_REG_START_EN_BASE 0x28 / 4: bit 0 starts the job
constexpr unsigned kMaxBuffers = 32, kMaxRuns = 16, kMaxRelocs = 128, kMaxUploads = 32;
constexpr uint32_t kPageSize = 4096;

// Byte offsets from the register base (0xfdc38100 on the RK3588: the vendor driver's mpp->reg_base)
enum : uint32_t {
  kRegIrq = 0x380,                                    // RKVDEC_REG_INT_EN: the interrupt enable bits and, read back, the status
  kRegCache0Size = 0x51c, kRegCache1Size = 0x55c, kRegCache2Size = 0x59c,
  kRegClrCache0 = 0x510, kRegClrCache1 = 0x550, kRegClrCache2 = 0x590,
};
constexpr uint32_t kCachePermit = 1u << 0 | 1u << 1 | 1u << 4;   // cacheable access, read allocate, 64 byte lines (rkvdec2_run)
// Interrupt status (RKVDEC_*_STA)
enum : uint32_t {
  kStaIrq = 1u << 0, kStaIrqRaw = 1u << 1, kStaReady = 1u << 2, kStaBus = 1u << 3, kStaError = 1u << 4, kStaTimeout = 1u << 5, kStaBufEmpty = 1u << 6,
  kStaColmvRefError = 1u << 7, kStaCabacEnd = 1u << 8, kStaSoftResetReady = 1u << 9,
};
constexpr uint32_t kStaErrorMask = kStaError | kStaTimeout | kStaBufEmpty | kStaColmvRefError;

inline uint32_t crc32(const uint8_t *p, size_t n, uint32_t crc = 0) {
  static uint32_t table[256];
  static bool ready = false;
  if (!ready) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = c & 1 ? 0xedb88320u ^ (c >> 1) : c >> 1;
      table[i] = c;
    }
    ready = true;
  }
  crc = ~crc;
  for (size_t i = 0; i < n; ++i) crc = table[(crc ^ p[i]) & 0xff] ^ (crc >> 8);
  return ~crc;
}

struct Hw {
  void *ctx;
  uint32_t (*read32)(void *ctx, uint32_t off);
  void (*write32)(void *ctx, uint32_t off, uint32_t value);
  void (*barrier)(void *ctx);
  uint64_t (*nowUs)(void *ctx);
  void (*delayUs)(void *ctx, uint32_t us);
  uint8_t *dmaVirt;      // the DMA window as the CPU sees it (normal non-cacheable memory: what the CPU writes is what the block reads)
  uint64_t dmaBus;       // its address on the block's bus (below 4 GiB; the block is in IOMMU bypass)
  uint32_t dmaBytes;
};

struct Upload {
  uint32_t buf, offset, len;
  const uint8_t *data;
};
struct Reloc {
  uint32_t reg, buf, offset;
};
struct Job {
  uint32_t regs[kRegCount];
  uint32_t runCount, runs[kMaxRuns][2];
  uint32_t relocCount;
  Reloc relocs[kMaxRelocs];
  uint32_t uploadCount;
  Upload uploads[kMaxUploads];
  uint32_t outBuf, outOffset, outLen, outCrc;
};

class Trace {
 public:
  // Checks the whole trace once (every length, every index), so a job that was accepted can be run without further bounds worries.
  bool parse(const uint8_t *data, size_t size) {
    data_ = data;
    size_ = size;
    jobsAt_ = 0;
    jobCount_ = 0;
    if (size < 20 || memcmp(data, "VDT1", 4) != 0) return fail("not a VDT1 trace");
    width_ = word(4);
    height_ = word(8);
    bufCount_ = word(12);
    jobCount_ = word(16);
    if (bufCount_ == 0 || bufCount_ > kMaxBuffers) return fail("bad buffer count");
    if (size < 20 + 4ull * bufCount_) return fail("truncated buffer table");
    uint64_t offset = 0;
    for (uint32_t i = 0; i < bufCount_; ++i) {
      size_t s = word(20 + 4 * i);
      bufSize_[i] = static_cast<uint32_t>(s);
      bufOffset_[i] = static_cast<uint32_t>(offset);
      offset += (s + kPageSize - 1) & ~(kPageSize - 1ull);
      if (offset > 0x40000000ull) return fail("buffers too large");
    }
    windowBytes_ = static_cast<uint32_t>(offset);
    jobsAt_ = 20 + 4 * bufCount_;
    size_t at = jobsAt_;
    Job scratch;
    for (uint32_t j = 0; j < jobCount_; ++j) {
      size_t next;
      if (!readJob(at, &scratch, &next)) return false;
      at = next;
    }
    if (at != size) return fail("bytes after the last job");
    return true;
  }

  // The jobs are read one after the other: `cursor` starts at 0 and advances.
  bool job(size_t *cursor, Job *out) const {
    if (*cursor == 0) *cursor = jobsAt_;
    size_t next;
    if (!readJob(*cursor, out, &next)) return false;
    *cursor = next;
    return true;
  }

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  uint32_t jobCount() const { return jobCount_; }
  uint32_t bufCount() const { return bufCount_; }
  uint32_t bufSize(uint32_t id) const { return id >= 1 && id <= bufCount_ ? bufSize_[id - 1] : 0; }
  uint32_t bufOffset(uint32_t id) const { return bufOffset_[id - 1]; }   // inside the window
  uint32_t windowBytes() const { return windowBytes_; }
  const char *error() const { return error_; }

 private:
  uint32_t word(size_t at) const {
    uint32_t v;
    memcpy(&v, data_ + at, 4);
    return v;
  }
  bool fail(const char *why) const {
    error_ = why;
    return false;
  }
  // One job at `at`; `next` is where the following one starts.
  bool readJob(size_t at, Job *j, size_t *next) const {
    auto have = [&](size_t n) { return at <= size_ && size_ - at >= n; };
    if (!have(4ull * kRegCount + 4)) return fail("truncated registers");
    memcpy(j->regs, data_ + at, 4 * kRegCount);
    at += 4 * kRegCount;
    j->runCount = word(at);
    at += 4;
    if (j->runCount > kMaxRuns || !have(8ull * j->runCount + 4)) return fail("bad register runs");
    for (uint32_t i = 0; i < j->runCount; ++i, at += 8) {
      j->runs[i][0] = word(at);
      j->runs[i][1] = word(at + 4);
      if (j->runs[i][0] >= kRegCount || j->runs[i][1] > kRegCount - j->runs[i][0]) return fail("register run out of range");
    }
    j->relocCount = word(at);
    at += 4;
    if (j->relocCount > kMaxRelocs || !have(12ull * j->relocCount + 4)) return fail("bad relocations");
    for (uint32_t i = 0; i < j->relocCount; ++i, at += 12) {
      j->relocs[i] = {word(at), word(at + 4), word(at + 8)};
      const Reloc &r = j->relocs[i];
      if (r.reg >= kRegCount || r.reg == kRegStart || r.buf < 1 || r.buf > bufCount_ || bufSize_[r.buf - 1] == 0 || r.offset >= bufSize_[r.buf - 1])
        return fail("relocation out of range");
    }
    j->uploadCount = word(at);
    at += 4;
    if (j->uploadCount > kMaxUploads) return fail("too many uploads");
    for (uint32_t i = 0; i < j->uploadCount; ++i) {
      if (!have(12)) return fail("truncated upload header");
      Upload &u = j->uploads[i];
      u.buf = word(at);
      u.offset = word(at + 4);
      u.len = word(at + 8);
      at += 12;
      const uint64_t padded = (u.len + 3ull) & ~3ull;
      if (!have(padded)) return fail("truncated upload data");
      if (u.buf < 1 || u.buf > bufCount_ || u.offset > bufSize_[u.buf - 1] || u.len > bufSize_[u.buf - 1] - u.offset) return fail("upload out of range");
      u.data = data_ + at;
      at += padded;
    }
    if (!have(16)) return fail("truncated result");
    j->outBuf = word(at);
    j->outOffset = word(at + 4);
    j->outLen = word(at + 8);
    j->outCrc = word(at + 12);
    at += 16;
    if (j->outBuf < 1 || j->outBuf > bufCount_ || j->outOffset > bufSize_[j->outBuf - 1] || j->outLen > bufSize_[j->outBuf - 1] - j->outOffset)
      return fail("result out of range");
    *next = at;
    return true;
  }

  const uint8_t *data_ = nullptr;
  size_t size_ = 0, jobsAt_ = 0;
  uint32_t width_ = 0, height_ = 0, bufCount_ = 0, jobCount_ = 0, windowBytes_ = 0;
  uint32_t bufSize_[kMaxBuffers] = {}, bufOffset_[kMaxBuffers] = {};
  mutable const char *error_ = "";
};

struct JobResult {
  bool done;            // the block raised its interrupt
  uint32_t status;      // the status read back (reg 224)
  uint64_t runUs;       // start to interrupt
  uint32_t crc;         // CRC-32 of the picture the block wrote
  bool crcOk;           // the block finished without an error bit and the picture has the trace's CRC
};

// The hardware half of a job: the cache permissions, the register runs (`runAt(i, &first, &count)` gives run i; register 10, the start, is written last, with
// bit 0 set) and the wait for the interrupt (at most `timeoutUs`). True when the block raised it; *status is what register 224 held, *runUs the time from the
// start to the interrupt. The interrupt is cleared afterwards, as rkvdec2_irq() does, and the status is the caller's to judge.
template <typename RunAt>
inline bool executeJob(const Hw &hw, const uint32_t regs[kRegCount], unsigned runCount, RunAt runAt, uint32_t timeoutUs, uint64_t *runUs, uint32_t *status) {
  hw.barrier(hw.ctx);
  hw.write32(hw.ctx, kRegCache0Size, kCachePermit);
  hw.write32(hw.ctx, kRegCache1Size, kCachePermit);
  hw.write32(hw.ctx, kRegCache2Size, kCachePermit);
  hw.write32(hw.ctx, kRegClrCache0, 1);
  hw.write32(hw.ctx, kRegClrCache1, 1);
  hw.write32(hw.ctx, kRegClrCache2, 1);
  for (unsigned i = 0; i < runCount; ++i) {
    uint32_t first = 0, count = 0;
    runAt(i, &first, &count);
    for (uint32_t r = first; r < first + count && r < kRegCount; ++r)
      if (r != kRegStart) hw.write32(hw.ctx, 4 * r, regs[r]);
  }
  hw.barrier(hw.ctx);
  const uint64_t start = hw.nowUs(hw.ctx);
  hw.write32(hw.ctx, 4 * kRegStart, regs[kRegStart] | 1);
  uint32_t st = 0;
  bool done = false;
  while (true) {
    st = hw.read32(hw.ctx, kRegIrq);
    if (st & kStaIrqRaw) {
      done = true;
      break;
    }
    if (hw.nowUs(hw.ctx) - start > timeoutUs) break;
    hw.delayUs(hw.ctx, 20);
  }
  *runUs = hw.nowUs(hw.ctx) - start;
  *status = st;
  if (done) hw.write32(hw.ctx, kRegIrq, 0);
  hw.barrier(hw.ctx);
  return done;
}

// Runs the jobs of a trace on the block behind `hw`. The block must be powered, clocked and out of reset, and its IOMMU must not translate.
class Player {
 public:
  Player(const Hw &hw, const Trace &trace) : hw_(hw), trace_(trace) {}

  // The window must hold every buffer. The buffers start out zero (the trace's model of them assumes that).
  bool prepare() {
    if (trace_.windowBytes() > hw_.dmaBytes) return false;
    if (hw_.dmaBus + trace_.windowBytes() > (1ull << 32)) return false;   // address registers are 32 bit
    memset(hw_.dmaVirt, 0, trace_.windowBytes());
    return true;
  }

  // Puts the job's address registers and CPU-written pages in place and starts it; waits at most `timeoutUs` for the interrupt. The picture is checked
  // against the trace's CRC.
  bool run(const Job &job, uint32_t timeoutUs, JobResult *result) {
    *result = {};
    for (uint32_t i = 0; i < job.uploadCount; ++i) {
      const Upload &u = job.uploads[i];
      memcpy(hw_.dmaVirt + trace_.bufOffset(u.buf) + u.offset, u.data, u.len);
    }
    uint32_t regs[kRegCount];
    memcpy(regs, job.regs, sizeof regs);
    for (uint32_t i = 0; i < job.relocCount; ++i) {
      const Reloc &r = job.relocs[i];
      regs[r.reg] = static_cast<uint32_t>(hw_.dmaBus + trace_.bufOffset(r.buf) + r.offset);
    }
    // The picture is overwritten by the block; a stale one from a previous pass must not make a dead block look alive.
    memset(hw_.dmaVirt + trace_.bufOffset(job.outBuf) + job.outOffset, 0, job.outLen);
    uint64_t runUs = 0;
    uint32_t status = 0;
    const bool done = executeJob(hw_, regs, job.runCount, [&](unsigned i, uint32_t *first, uint32_t *count) {
      *first = job.runs[i][0];
      *count = job.runs[i][1];
    }, timeoutUs, &runUs, &status);
    result->runUs = runUs;
    result->done = done;
    result->status = status;
    result->crc = crc32(hw_.dmaVirt + trace_.bufOffset(job.outBuf) + job.outOffset, job.outLen);
    result->crcOk = done && !(status & kStaErrorMask) && result->crc == job.outCrc;
    return result->crcOk;
  }

 private:
  Hw hw_;
  const Trace &trace_;
};

}  // namespace vdec
