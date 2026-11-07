// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Host test of VencTrace.h: the parser and the player, against a model of the RKVENC0 register interface that is strict about what the vendor driver does:
// the forced clear and the counter clear before the first register write, every register of the trace written exactly once with its value (the address
// registers with the bus address of their buffer), the CPU-written pages in the window by the time the block is started, the start register last, the status
// read and cleared afterwards. The "block" then writes a bit stream where the stream address register says and raises the done bit.
//
// usage: test-venc TRACE.vet...
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <map>
#include <vector>

#include "VencTrace.h"

namespace {

int gChecks = 0, gFailures = 0;
#define CHECK(cond)                                                      \
  do {                                                                   \
    ++gChecks;                                                           \
    if (!(cond)) {                                                       \
      ++gFailures;                                                       \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
    }                                                                    \
  } while (0)

std::vector<uint8_t> readFile(const char *path) {
  FILE *f = fopen(path, "rb");
  std::vector<uint8_t> data;
  if (!f) return data;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) data.insert(data.end(), buf, buf + n);
  fclose(f);
  return data;
}

constexpr uint64_t kBus = 0x6a000000ull;
constexpr uint32_t kWindow = 32u << 20;   // the size of the board's window

enum class Fault { None, ErrorStatus, NeverDone };

// The block as the vendor driver sees it.
struct Model {
  std::vector<uint8_t> window = std::vector<uint8_t>(kWindow);
  std::map<uint32_t, uint32_t> written;   // register byte offset -> value, this job
  std::vector<uint32_t> order;            // register write order, this job
  bool forcedClear = false, clearDone = false, counterClear = false, started = false;
  uint32_t status = 0, streamLen = 0, intClr = 0;
  uint64_t clock = 0;
  Fault fault = Fault::None;
  // what the test expects of the job about to start
  const venc::Job *job = nullptr;
  const venc::Trace *trace = nullptr;
  int jobs = 0, errors = 0;
  std::vector<uint32_t> streamLens;

  uint32_t expectedValue(uint32_t index) const {
    for (uint32_t i = 0; i < job->relocCount; ++i)
      if (job->relocs[i].reg == index) return static_cast<uint32_t>(kBus + trace->bufOffset(job->relocs[i].buf) + job->relocs[i].offset);
    for (uint32_t i = 0; i < job->regCount; ++i)
      if (job->regs[i].index == index) return job->regs[i].value;
    return 0;
  }

  void write(uint32_t off, uint32_t value) {
    const bool imagePhase = forcedClear && clearDone && counterClear;   // between the clears and the start, the register image goes out (0x14 and 0x28 are in it)
    if (off == venc::kRegClear && !imagePhase) {
      if (value == 2) forcedClear = true;
      if (value == 0 && forcedClear) clearDone = true;
      return;
    }
    if (off == venc::kRegHwCounter) {
      if (value == 2 && clearDone) counterClear = true;
      return;
    }
    if (off == venc::kRegIntClr && !imagePhase) {
      intClr = value;
      status &= ~value;
      return;
    }
    if (off == venc::kRegStartOff) {
      errors += !(forcedClear && clearDone && counterClear) ? 1 : 0;   // the clears came first
      start(value);
      return;
    }
    // everything else is a register of the image: only ascending indexes, only once, only after the clears
    errors += !(forcedClear && clearDone && counterClear) ? 1 : 0;
    errors += written.count(off) ? 1 : 0;
    errors += !order.empty() && order.back() >= off ? 1 : 0;
    written[off] = value;
    order.push_back(off);
  }

  void start(uint32_t value) {
    started = true;
    // start is last: every register of the image has been written, none twice, each with its value
    uint32_t expected = 0;
    for (uint32_t i = 0; i < job->regCount; ++i) {
      const uint32_t index = job->regs[i].index;
      if (index == venc::kRegStart) {
        errors += value != job->regs[i].value;
        continue;
      }
      ++expected;
      auto it = written.find(4 * index);
      errors += (it == written.end() || it->second != expectedValue(index)) ? 1 : 0;
    }
    errors += written.size() != expected;
    // the pages the CPU wrote are in the window now
    for (uint32_t i = 0; i < job->uploadCount; ++i) {
      const venc::Upload &u = job->uploads[i];
      errors += memcmp(window.data() + trace->bufOffset(u.buf) + u.offset, u.data, u.len) != 0;
    }
    // the block writes the stream where the start address register says
    const uint32_t bsbs = written.count(4 * 175) ? written[4 * 175] : 0;
    const uint64_t at = bsbs - kBus;
    streamLen = 600 + 37 * jobs;
    if (fault == Fault::None && at + streamLen <= window.size())
      for (uint32_t i = 0; i < streamLen; ++i) window[at + i] = static_cast<uint8_t>(0x80 + i * 7 + jobs);
    status = fault == Fault::NeverDone ? 0u : fault == Fault::ErrorStatus ? uint32_t(venc::kStaWbusErr) : uint32_t(venc::kStaEncDone);
    ++jobs;
    streamLens.push_back(streamLen);
    written.clear();
    order.clear();
    forcedClear = clearDone = counterClear = false;
  }

  uint32_t read(uint32_t off) {
    if (off == venc::kRegIntSta) return status;
    if (off == venc::kRegStStreamLen) return streamLen;
    return 0;
  }
};

uint32_t mRead(void *c, uint32_t off) { return static_cast<Model *>(c)->read(off); }
void mWrite(void *c, uint32_t off, uint32_t v) { static_cast<Model *>(c)->write(off, v); }
void mBarrier(void *) {}
uint64_t mNow(void *c) { return static_cast<Model *>(c)->clock; }
void mDelay(void *c, uint32_t us) { static_cast<Model *>(c)->clock += us; }

venc::Hw hwFor(Model &m) {
  venc::Hw hw = {};
  hw.ctx = &m;
  hw.read32 = mRead;
  hw.write32 = mWrite;
  hw.barrier = mBarrier;
  hw.nowUs = mNow;
  hw.delayUs = mDelay;
  hw.dmaVirt = m.window.data();
  hw.dmaBus = kBus;
  hw.dmaBytes = kWindow;
  return hw;
}

void testTrace(const char *path) {
  printf("%s\n", path);
  const std::vector<uint8_t> data = readFile(path);
  CHECK(!data.empty());
  venc::Trace *trace = new venc::Trace();
  CHECK(trace->parse(data.data(), data.size()));
  CHECK(trace->jobCount() >= 1);
  CHECK(trace->width() > 0 && trace->height() > 0);

  // ---- a good run, twice (the window is cleared between passes)
  Model m;
  m.trace = trace;
  venc::Player *player = new venc::Player();
  player->init(hwFor(m), trace);
  venc::Job *job = new venc::Job();
  for (int pass = 0; pass < 2; ++pass) {
    CHECK(player->prepare());
    m.jobs = 0;
    size_t cursor = 0;
    for (uint32_t n = 0; n < trace->jobCount(); ++n) {
      CHECK(trace->job(&cursor, job));
      m.job = job;
      venc::JobResult r;
      CHECK(player->run(*job, 100000, &r));
      CHECK(r.ok && r.done && r.status == venc::kStaEncDone);
      CHECK(r.streamLen == m.streamLen);
      CHECK(m.intClr == venc::kStaEncDone);   // the status that was seen was cleared
      CHECK(m.status == 0);
      // the bytes the block wrote are where the player says the stream is
      const uint8_t *s = player->stream(*job);
      bool same = true;
      for (uint32_t i = 0; i < m.streamLen; ++i) same = same && s[i] == static_cast<uint8_t>(0x80 + i * 7 + n);
      CHECK(same);
      CHECK(m.errors == 0);
    }
  }
  CHECK(m.errors == 0);

  // ---- an error status ends the wait at once and is reported
  {
    Model e;
    e.trace = trace;
    e.fault = Fault::ErrorStatus;
    venc::Player *p = new venc::Player();
    p->init(hwFor(e), trace);
    CHECK(p->prepare());
    size_t cursor = 0;
    CHECK(trace->job(&cursor, job));
    e.job = job;
    venc::JobResult r;
    CHECK(!p->run(*job, 100000, &r));
    CHECK(!r.done && !r.ok && (r.status & venc::kStaWbusErr));
    CHECK(r.runUs < 1000);
    delete p;
  }
  // ---- a block that never answers: the wait ends at the limit, no more, no less
  {
    Model e;
    e.trace = trace;
    e.fault = Fault::NeverDone;
    venc::Player *p = new venc::Player();
    p->init(hwFor(e), trace);
    CHECK(p->prepare());
    size_t cursor = 0;
    CHECK(trace->job(&cursor, job));
    e.job = job;
    venc::JobResult r;
    CHECK(!p->run(*job, 5000, &r));
    CHECK(!r.done && !r.ok && r.status == 0);
    CHECK(r.runUs >= 5000 && r.runUs < 5100);
    delete p;
  }
  // ---- a window that is too small, or does not fit below 4 GiB
  {
    Model e;
    venc::Hw hw = hwFor(e);
    hw.dmaBytes = trace->windowBytes() - 1;
    venc::Player *p = new venc::Player();
    p->init(hw, trace);
    CHECK(!p->prepare());
    hw = hwFor(e);
    hw.dmaBus = (1ull << 32) - 4096;
    p->init(hw, trace);
    CHECK(!p->prepare());
    delete p;
  }
  // ---- the parser refuses a truncated trace at every length that was tried, and bytes after the end
  {
    venc::Trace *t = new venc::Trace();
    unsigned accepted = 0, tried = 0;
    for (size_t n = 0; n < data.size(); n += (n < 2048 || n + 2048 > data.size()) ? 1 : 997) {
      ++tried;
      accepted += t->parse(data.data(), n) ? 1 : 0;
    }
    CHECK(accepted == 0 && tried > 3000);
    std::vector<uint8_t> longer = data;
    longer.push_back(0);
    CHECK(!t->parse(longer.data(), longer.size()));
    // bad magic, absurd counts, a register out of order, a relocation into nowhere
    std::vector<uint8_t> bad = data;
    bad[0] ^= 1;
    CHECK(!t->parse(bad.data(), bad.size()));
    bad = data;
    uint32_t huge = 0x7fffffff;
    memcpy(&bad[12], &huge, 4);   // buffer count
    CHECK(!t->parse(bad.data(), bad.size()));
    bad = data;
    const size_t firstJob = 20 + 4 * trace->bufCount();
    uint32_t zero = 0;
    memcpy(&bad[firstJob + 4 + 8], &zero, 4);   // index of the second register: 0, not above the first
    CHECK(!t->parse(bad.data(), bad.size()));
    bad = data;
    uint32_t regs;
    memcpy(&regs, &data[firstJob], 4);
    const size_t relocAt = firstJob + 4 + 8ull * regs + 4;   // first relocation: reg, buf, offset
    uint32_t nowhere = 99;
    memcpy(&bad[relocAt + 4], &nowhere, 4);
    CHECK(!t->parse(bad.data(), bad.size()));
    bad = data;
    uint32_t past = 0x7fffffff;
    memcpy(&bad[relocAt + 8], &past, 4);
    CHECK(!t->parse(bad.data(), bad.size()));
    delete t;
  }
  delete job;
  delete player;
  delete trace;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("usage: test-venc TRACE.vet...\n");
    return 2;
  }
  for (int i = 1; i < argc; ++i) testTrace(argv[i]);
  printf("%d checks, %d failures\n", gChecks, gFailures);
  return gFailures ? 1 : 0;
}
