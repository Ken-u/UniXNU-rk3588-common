// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VencTrace.h -- the encode-job trace the RKVENC0 self-test replays, and the code that replays it. A pure computation over a register accessor and a DMA
// window (like VdecTrace.h), so a host test can run it against a model of the block; Rk3588Venc.cpp supplies the real ones.
//
// A trace is what Rockchip's unmodified user-space library (MPP: the H.264 / H.265 encoder front end and the vepu580 register generator) asks the kernel
// driver to run for a few pictures, recorded on a host with tools/trace-shim-enc.c and packed by tools/make-enc-trace.py: for every encode job the register
// image, the registers that hold buffer addresses (which buffer, which offset), and the pages of the CPU-written buffers (the input picture, the parameter
// sets in front of the stream) that changed. Replaying it is what the vendor kernel's rkvenc_run() does for one task: a forced clear, the counter clear, the
// same register writes in the same order, start last, then wait for the interrupt status. What the block wrote is checked outside: the bit stream is
// printed and a software decoder on the host decodes it (tools/check-enc-output.py).
//
// Binary layout (little endian words; make-enc-trace.py writes it):
//   'VET1', width, height, nBuf, nJob, then nBuf words: size of buffer 1..nBuf (0 = unused)
//   per job:
//     nReg, nReg x { index, value }              the registers MPP wrote, ascending (the start register, index 4, is written last)
//     nReloc, nReloc x { reg, buf, offset }      register `reg` = bus address of buffer `buf` + offset
//     nUpload, nUpload x { buf, offset, len } and the bytes (padded to four)   CPU-written content to copy into the buffers before the run
//     outBuf, outOffset, outLen                  the bit stream buffer, where MPP's address registers say the block starts writing, and its size
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace venc {

constexpr unsigned kRegCount = 4096;     // the register file MPP writes: words 0 .. 3583 (osd class ends there); the status class starts at 4096
constexpr unsigned kRegStart = 4;        // 0x10: the start register
constexpr unsigned kMaxBuffers = 32, kMaxRegs = 4096, kMaxRelocs = 64, kMaxUploads = 256;
constexpr uint32_t kPageSize = 4096;

// Byte offsets from the register base (0xfdbd0000 on the RK3588)
enum : uint32_t {
  kRegEnable = 0x04, kRegStartOff = 0x10, kRegClear = 0x14, kRegIntEn = 0x20, kRegIntMask = 0x24, kRegIntClr = 0x28, kRegIntSta = 0x2c, kRegWdg = 0x38,
  kRegHwCounter = 0x5300,      // written 2 before the registers ("clear hardware counter" in rkvenc_run)
  kRegStStreamLen = 0x4000,    // status class: bs_lgth_l32, the bit stream length in bytes
  kRegStQpSum = 0x400c,
};

// Interrupt status (INT_STA_*)
enum : uint32_t {
  kStaEncDone = 1u << 0, kStaSclrDone = 1u << 2, kStaSliceDone = 1u << 3, kStaBsOverflow = 1u << 4, kStaBrspOtsd = 1u << 5, kStaWbusErr = 1u << 6,
  kStaRbusErr = 1u << 7, kStaWdg = 1u << 8,
};
constexpr uint32_t kStaErrorMask = kStaBrspOtsd | kStaWbusErr | kStaRbusErr | kStaWdg;   // INT_STA_ERROR of the vendor driver

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
struct RegWrite {
  uint32_t index, value;
};
struct Job {
  uint32_t regCount;
  RegWrite regs[kMaxRegs];
  uint32_t relocCount;
  Reloc relocs[kMaxRelocs];
  uint32_t uploadCount;
  Upload uploads[kMaxUploads];
  uint32_t outBuf, outOffset, outLen;
};

class Trace {
 public:
  // Checks the whole trace once (every length, every index), so a job that was accepted can be run without further bounds worries.
  bool parse(const uint8_t *data, size_t size) {
    error_ = "";
    data_ = data;
    size_ = size;
    jobsAt_ = 0;
    jobCount_ = 0;
    if (size < 20 || memcmp(data, "VET1", 4) != 0) return fail("not a VET1 trace");
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
    Job *scratch = &scratch_;
    for (uint32_t j = 0; j < jobCount_; ++j) {
      size_t next;
      if (!readJob(at, scratch, &next)) return false;
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
    if (!have(4)) return fail("truncated registers");
    j->regCount = word(at);
    at += 4;
    if (j->regCount > kMaxRegs || !have(8ull * j->regCount + 4)) return fail("bad register list");
    uint32_t previous = 0;
    for (uint32_t i = 0; i < j->regCount; ++i, at += 8) {
      j->regs[i] = {word(at), word(at + 4)};
      if (j->regs[i].index >= kRegCount || (i && j->regs[i].index <= previous)) return fail("register index out of order or out of range");
      previous = j->regs[i].index;
    }
    j->relocCount = word(at);
    at += 4;
    if (j->relocCount > kMaxRelocs || !have(12ull * j->relocCount + 4)) return fail("bad relocations");
    for (uint32_t i = 0; i < j->relocCount; ++i, at += 12) {
      j->relocs[i] = {word(at), word(at + 4), word(at + 8)};
      const Reloc &r = j->relocs[i];
      if (r.reg >= kRegCount || r.reg == kRegStart || r.buf < 1 || r.buf > bufCount_ || bufSize_[r.buf - 1] == 0 || r.offset > bufSize_[r.buf - 1])   // (one past the end is legal: H.265's stream top address)
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
    if (!have(12)) return fail("truncated result");
    j->outBuf = word(at);
    j->outOffset = word(at + 4);
    j->outLen = word(at + 8);
    at += 12;
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
  mutable Job scratch_;
};

struct JobResult {
  bool done;            // the block raised its interrupt (encode done) before the time limit
  uint32_t status;      // the status read back (int_sta)
  uint64_t runUs;       // start to interrupt
  uint32_t streamLen;   // bs_lgth_l32 after the job
  bool ok;              // done and no error bit
};

// The vendor driver's rkvenc_soft_reset (the safe reset): mask the interrupts, ask the block to stop safely, wait for "safe reset done", clear, release.
// True when the block said done.
inline bool softReset(const Hw &hw) {
  hw.write32(hw.ctx, kRegIntMask, 0x3ff);
  hw.write32(hw.ctx, kRegClear, 0x1);
  bool done = false;
  for (unsigned t = 0; t < 1000 && !done; t += 5) {
    done = (hw.read32(hw.ctx, kRegIntSta) & kStaSclrDone) != 0;
    if (!done) hw.delayUs(hw.ctx, 5);
  }
  hw.write32(hw.ctx, kRegClear, 0x2);
  hw.delayUs(hw.ctx, 5);
  hw.write32(hw.ctx, kRegClear, 0);
  hw.write32(hw.ctx, kRegIntClr, 0xffffffffu);
  hw.write32(hw.ctx, kRegIntSta, 0);
  return done;
}

// Runs the jobs of a trace on the block behind `hw`. The block must be powered, clocked and out of reset, and its IOMMU must not translate. A Player and a
// Trace are big (a Job holds up to 4096 registers): allocate them on the heap, not on the stack of a kernel thread.
class Player {
 public:
  // No constructor: the kernel has no placement new, and a Player lives in heap memory that IOMallocZero cleared. init() is the constructor.
  void init(const Hw &hw, const Trace *trace) {
    hw_ = hw;
    trace_ = trace;
  }

  // The window must hold every buffer. The buffers start out zero (the trace's model of them assumes that).
  bool prepare() {
    if (trace_->windowBytes() > hw_.dmaBytes) return false;
    if (hw_.dmaBus + trace_->windowBytes() > (1ull << 32)) return false;   // address registers are 32 bit
    memset(hw_.dmaVirt, 0, trace_->windowBytes());
    return true;
  }

  // Puts the job's address registers and CPU-written pages in place and starts it; waits at most `timeoutUs` for the interrupt.
  bool run(const Job &job, uint32_t timeoutUs, JobResult *result) {
    *result = {};
    for (uint32_t i = 0; i < job.uploadCount; ++i) {
      const Upload &u = job.uploads[i];
      memcpy(hw_.dmaVirt + trace_->bufOffset(u.buf) + u.offset, u.data, u.len);
    }
    // the bit stream is overwritten by the block; stale bytes of a previous pass must not make a dead block look alive (the CPU-written head of the
    // buffer, the parameter sets, is an upload and comes back with it)
    uint32_t *regs = regs_;
    bool *set = set_;
    memset(regs, 0, sizeof regs_);
    memset(set, 0, sizeof set_);
    for (uint32_t i = 0; i < job.regCount; ++i) {
      regs[job.regs[i].index] = job.regs[i].value;
      set[job.regs[i].index] = true;
    }
    for (uint32_t i = 0; i < job.relocCount; ++i) {
      const Reloc &r = job.relocs[i];
      regs[r.reg] = static_cast<uint32_t>(hw_.dmaBus + trace_->bufOffset(r.buf) + r.offset);
    }
    hw_.barrier(hw_.ctx);
    // rkvenc_run: "add force clear to avoid pagefault" (VEPU580), clear the hardware counter, the registers, the start
    hw_.write32(hw_.ctx, kRegClear, 0x2);
    hw_.delayUs(hw_.ctx, 5);
    hw_.write32(hw_.ctx, kRegClear, 0);
    hw_.write32(hw_.ctx, kRegHwCounter, 0x2);
    for (uint32_t i = 0; i < job.regCount; ++i) {
      const uint32_t index = job.regs[i].index;
      if (index != kRegStart) hw_.write32(hw_.ctx, 4 * index, regs[index]);
    }
    hw_.barrier(hw_.ctx);
    const uint64_t start = hw_.nowUs(hw_.ctx);
    hw_.write32(hw_.ctx, kRegStartOff, set[kRegStart] ? regs[kRegStart] : 0);
    uint32_t st = 0;
    bool done = false;
    while (true) {
      st = hw_.read32(hw_.ctx, kRegIntSta);
      if (st & (kStaEncDone | kStaErrorMask)) {
        done = (st & kStaEncDone) != 0;
        break;
      }
      if (hw_.nowUs(hw_.ctx) - start > timeoutUs) break;
      hw_.delayUs(hw_.ctx, 20);
    }
    result->runUs = hw_.nowUs(hw_.ctx) - start;
    result->status = st;
    result->done = done;
    result->streamLen = hw_.read32(hw_.ctx, kRegStStreamLen);
    if (st) hw_.write32(hw_.ctx, kRegIntClr, st);   // rkvenc_irq: clear what was seen
    hw_.barrier(hw_.ctx);
    result->ok = done && !(st & kStaErrorMask);
    return result->ok;
  }

  // Where the bit stream of the job starts and how many bytes of it the CPU can look at (the window as the CPU sees it).
  const uint8_t *stream(const Job &job) const { return hw_.dmaVirt + trace_->bufOffset(job.outBuf) + job.outOffset; }

 private:
  Hw hw_;
  const Trace *trace_;
  uint32_t regs_[kRegCount];   // members, not locals: a player is allocated on the heap (these are 20 KiB, the kernel stack 16 KiB)
  bool set_[kRegCount];
};

}  // namespace venc
