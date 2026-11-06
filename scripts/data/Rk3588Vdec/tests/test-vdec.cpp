// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// test-vdec.cpp -- host test of VdecTrace.h: the trace parser (against traces written here in the layout of tools/make-trace.py and against the committed
// traces/*.vdt) and the job player (against a model of the decoder's register file that is strict about the order and the content of what is written).
//
//   c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I.. test-vdec.cpp -o /tmp/test-vdec && /tmp/test-vdec ../traces/t320_cabac.vdt
//
// A passing run means the player does what the vendor kernel's rkvdec2_run() does with a job (cache setup, only the recorded register ranges in
// ascending order, the start register last with bit 0 set, the address registers replaced by bus addresses inside the window, the CPU-written pages
// in place when the block starts) and that a bad trace is refused before anything is written. It does not mean the block on the board agrees.
#include <stdio.h>

#include <map>
#include <string>
#include <vector>

#include "../VdecTrace.h"

using namespace vdec;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond)                                                                     \
  do {                                                                                  \
    ++g_checks;                                                                         \
    if (!(cond)) {                                                                      \
      ++g_fail;                                                                         \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                            \
    }                                                                                   \
  } while (0)

// ---- the trace writer (what make-trace.py does)
struct TraceBuilder {
  std::vector<uint8_t> out;
  void w(uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(v >> (8 * i) & 0xff);
  }
  void header(uint32_t width, uint32_t height, const std::vector<uint32_t> &sizes, uint32_t jobs) {
    out.insert(out.end(), {'V', 'D', 'T', '1'});
    w(width);
    w(height);
    w(static_cast<uint32_t>(sizes.size()));
    w(jobs);
    for (uint32_t s : sizes) w(s);
  }
  struct J {
    uint32_t regs[kRegCount] = {};
    std::vector<std::pair<uint32_t, uint32_t>> runs;
    std::vector<Reloc> relocs;
    struct U {
      uint32_t buf, off;
      std::vector<uint8_t> data;
    };
    std::vector<U> uploads;
    uint32_t outBuf = 0, outOff = 0, outLen = 0, outCrc = 0;
  };
  void job(const J &j) {
    for (uint32_t r : j.regs) w(r);
    w(static_cast<uint32_t>(j.runs.size()));
    for (auto &r : j.runs) {
      w(r.first);
      w(r.second);
    }
    w(static_cast<uint32_t>(j.relocs.size()));
    for (auto &r : j.relocs) {
      w(r.reg);
      w(r.buf);
      w(r.offset);
    }
    w(static_cast<uint32_t>(j.uploads.size()));
    for (auto &u : j.uploads) {
      w(u.buf);
      w(u.off);
      w(static_cast<uint32_t>(u.data.size()));
      out.insert(out.end(), u.data.begin(), u.data.end());
      out.insert(out.end(), (4 - u.data.size() % 4) % 4, 0);
    }
    w(j.outBuf);
    w(j.outOff);
    w(j.outLen);
    w(j.outCrc);
  }
};

// ---- the model of the block
// It decodes nothing: when started it writes a picture that is a function of the stream it finds where reg 128 points and of the job number, to where
// reg 130 points, then raises the interrupt after `polls` reads of the status.
struct Model {
  static constexpr uint64_t kBus = 0x65000000ull;
  std::vector<uint8_t> mem;
  std::map<uint32_t, uint32_t> reg;       // what the driver wrote, by byte offset
  std::vector<uint32_t> order;            // the offsets in write order
  uint32_t statusReg = 0, polls = 0, pollsLeft = 0;
  unsigned starts = 0;
  bool started = false;
  uint32_t errorBits = 0;                 // set into the status when the job "ends"
  bool neverFinish = false;
  uint64_t clock = 0;
  std::vector<std::string> violations;
  uint32_t expectedStreamWord = 0;        // the first word the stream buffer must hold when the job starts
  uint32_t streamReg = 128 * 4, outReg = 130 * 4;
  std::vector<uint8_t> lastPicture;

  explicit Model(size_t bytes) : mem(bytes, 0xAA) {}

  static uint32_t pattern(uint32_t stream, uint32_t index) { return (stream * 2654435761u + index * 40503u) >> 8 & 0xff; }

  void write(uint32_t off, uint32_t v) {
    order.push_back(off);
    reg[off] = v;
    if (started && off != kRegIrq) violations.push_back("register write after the start");
    if (off == 4 * kRegStart) {
      if (!(v & 1)) violations.push_back("start without bit 0");
      start();
    }
    if (off == kRegIrq) statusReg = v;   // writing 0 clears
  }
  uint32_t read(uint32_t off) {
    if (off == kRegIrq) {
      if (!started) return statusReg;
      if (neverFinish) return 0;
      if (pollsLeft && --pollsLeft) return 0;
      started = false;
      statusReg = kStaIrqRaw | kStaReady | errorBits;
      return statusReg;
    }
    return reg.count(off) ? reg[off] : 0;
  }
  bool inWindow(uint32_t addr) const { return addr >= kBus && addr < kBus + mem.size(); }
  void start() {
    ++starts;
    started = true;
    pollsLeft = polls;
    // cache setup first
    for (uint32_t off : {kRegCache0Size, kRegCache1Size, kRegCache2Size})
      if (!reg.count(off) || reg[off] != kCachePermit) violations.push_back("cache size register not set");
    for (uint32_t off : {kRegClrCache0, kRegClrCache1, kRegClrCache2})
      if (!reg.count(off) || reg[off] != 1) violations.push_back("cache clear register not set");
    const uint32_t stream = reg[streamReg], out = reg[outReg];
    if (!inWindow(stream) || !inWindow(out)) {
      violations.push_back("stream or output address outside the window");
      return;
    }
    uint32_t first;
    memcpy(&first, &mem[stream - kBus], 4);
    if (first != expectedStreamWord) violations.push_back("stream page not in place at the start");
    lastPicture.clear();
    for (uint32_t i = 0; i < pictureLen; ++i) lastPicture.push_back(pattern(first, i));
    if (!lastPicture.empty()) memcpy(&mem[out - kBus], lastPicture.data(), lastPicture.size());
  }
  uint32_t pictureLen = 0;
};

struct Fixture {
  // two buffers (stream, picture) and a third with a table, three jobs; the second job uploads a new stream page
  TraceBuilder b;
  uint32_t outLen = 4096 + 100;   // a picture that is not a whole number of pages
  std::vector<uint32_t> streamWords = {0x11111111, 0x22222222, 0x33333333};
  uint32_t crcOf(uint32_t streamWord) {
    std::vector<uint8_t> pic;
    for (uint32_t i = 0; i < outLen; ++i) pic.push_back(Model::pattern(streamWord, i));
    return crc32(pic.data(), pic.size());
  }
  std::vector<uint8_t> build() {
    b = TraceBuilder();
    b.header(64, 48, {8192, 4 * 4096, 4096}, 3);
    for (unsigned k = 0; k < 3; ++k) {
      TraceBuilder::J j;
      for (unsigned i = 0; i < kRegCount; ++i) j.regs[i] = 0xdead0000 + i;   // the garbage MPP leaves in address registers
      j.regs[10] = 0x100;
      j.regs[11] = 0x2c0de;
      j.runs = {{8, 25}, {64, 5}, {128, 4}};
      j.relocs = {{128, 1, 0}, {129, 1, 0}, {130, 2, k * 4096u}, {131, 3, 64}};
      j.uploads.push_back({1, 0, {static_cast<uint8_t>(streamWords[k]), static_cast<uint8_t>(streamWords[k] >> 8), static_cast<uint8_t>(streamWords[k] >> 16),
                                  static_cast<uint8_t>(streamWords[k] >> 24), 1, 2, 3}});   // seven bytes: the padding is exercised
      j.outBuf = 2;
      j.outOff = k * 4096u;
      j.outLen = outLen;
      j.outCrc = crcOf(streamWords[k]);
      b.job(j);
    }
    return b.out;
  }
};

static Hw hwFor(Model *m) {
  Hw hw = {};
  hw.ctx = m;
  hw.read32 = [](void *c, uint32_t off) { return static_cast<Model *>(c)->read(off); };
  hw.write32 = [](void *c, uint32_t off, uint32_t v) { static_cast<Model *>(c)->write(off, v); };
  hw.barrier = [](void *) {};
  hw.nowUs = [](void *c) { return static_cast<Model *>(c)->clock += 7; };
  hw.delayUs = [](void *c, uint32_t us) { static_cast<Model *>(c)->clock += us; };
  hw.dmaVirt = m->mem.data();
  hw.dmaBus = Model::kBus;
  hw.dmaBytes = static_cast<uint32_t>(m->mem.size());
  return hw;
}

static void testParse() {
  Fixture f;
  std::vector<uint8_t> t = f.build();
  Trace trace;
  CHECK(trace.parse(t.data(), t.size()));
  CHECK(trace.width() == 64 && trace.height() == 48 && trace.jobCount() == 3 && trace.bufCount() == 3);
  CHECK(trace.bufSize(1) == 8192 && trace.bufSize(3) == 4096 && trace.bufSize(4) == 0 && trace.bufSize(0) == 0);
  CHECK(trace.bufOffset(1) == 0 && trace.bufOffset(2) == 8192 && trace.bufOffset(3) == 8192 + 16384);
  CHECK(trace.windowBytes() == 8192 + 16384 + 4096);
  size_t cursor = 0;
  Job j;
  unsigned n = 0;
  while (n < 3 && trace.job(&cursor, &j)) {
    CHECK(j.runCount == 3 && j.relocCount == 4 && j.uploadCount == 1 && j.uploads[0].len == 7);
    CHECK(j.outBuf == 2 && j.outLen == f.outLen && j.outOffset == n * 4096u);
    ++n;
  }
  CHECK(n == 3);
  CHECK(cursor == t.size());

  // every cut of the trace is refused, none crashes (the sanitizers are watching)
  for (size_t cut = 0; cut < t.size(); ++cut) {
    Trace tr;
    CHECK(!tr.parse(t.data(), cut));
  }
  // trailing bytes
  std::vector<uint8_t> extra = t;
  extra.push_back(0);
  Trace tr;
  CHECK(!tr.parse(extra.data(), extra.size()));
  // wrong magic
  std::vector<uint8_t> bad = t;
  bad[3] = '2';
  CHECK(!tr.parse(bad.data(), bad.size()));

  // fields out of range: a relocation to a buffer that does not exist, past the end of a buffer, in the start register, in an unused register; a run past
  // the register file; an upload past its buffer; an output past its buffer
  struct Case {
    const char *what;
    void (*mutate)(TraceBuilder::J *);
  } cases[] = {
      {"reloc buffer 9", [](TraceBuilder::J *j) { j->relocs[0].buf = 9; }},
      {"reloc buffer 0", [](TraceBuilder::J *j) { j->relocs[0].buf = 0; }},
      {"reloc offset past the buffer", [](TraceBuilder::J *j) { j->relocs[0].offset = 8192; }},
      {"reloc into the start register", [](TraceBuilder::J *j) { j->relocs[0].reg = kRegStart; }},
      {"reloc register 360", [](TraceBuilder::J *j) { j->relocs[0].reg = 360; }},
      {"run past the registers", [](TraceBuilder::J *j) { j->runs[2] = {350, 11}; }},
      {"run start past the registers", [](TraceBuilder::J *j) { j->runs[2] = {360, 0}; }},
      {"upload past the buffer", [](TraceBuilder::J *j) { j->uploads[0].off = 8190; }},
      {"upload to buffer 0", [](TraceBuilder::J *j) { j->uploads[0].buf = 0; }},
      {"output past the buffer", [](TraceBuilder::J *j) { j->outOff = 12200; }},
      {"output buffer 4", [](TraceBuilder::J *j) { j->outBuf = 4; }},
  };
  for (auto &c : cases) {
    TraceBuilder b;
    b.header(64, 48, {8192, 3 * 4096, 4096}, 1);
    TraceBuilder::J jb;
    jb.runs = {{8, 25}, {64, 5}, {128, 4}};
    jb.relocs = {{128, 1, 0}, {129, 1, 0}, {130, 2, 0}, {131, 3, 64}};
    jb.uploads.push_back({1, 0, {1, 2, 3, 4}});
    jb.outBuf = 2;
    jb.outLen = 100;
    c.mutate(&jb);
    b.job(jb);
    Trace t2;
    const bool accepted = t2.parse(b.out.data(), b.out.size());
    if (accepted) printf("accepted: %s\n", c.what);
    CHECK(!accepted);
  }
  // the unmutated single job is fine (so the cases above fail for the reason they name)
  {
    TraceBuilder b;
    b.header(64, 48, {8192, 3 * 4096, 4096}, 1);
    TraceBuilder::J jb;
    jb.runs = {{8, 25}, {64, 5}, {128, 4}};
    jb.relocs = {{128, 1, 0}, {129, 1, 0}, {130, 2, 0}, {131, 3, 64}};
    jb.uploads.push_back({1, 0, {1, 2, 3, 4}});
    jb.outBuf = 2;
    jb.outLen = 100;
    b.job(jb);
    Trace t2;
    CHECK(t2.parse(b.out.data(), b.out.size()));
  }
}

static void testPlay() {
  Fixture f;
  std::vector<uint8_t> t = f.build();
  Trace trace;
  CHECK(trace.parse(t.data(), t.size()));

  // a good run: three jobs, every picture right, the register traffic as the vendor driver does it
  {
    Model m(trace.windowBytes());
    m.polls = 3;
    m.pictureLen = f.outLen;
    Hw hw = hwFor(&m);
    Player p(hw, trace);
    CHECK(p.prepare());
    size_t cursor = 0;
    Job j;
    for (unsigned k = 0; k < 3; ++k) {
      CHECK(trace.job(&cursor, &j));
      m.expectedStreamWord = f.streamWords[k];
      m.order.clear();
      JobResult r;
      CHECK(p.run(j, 1000, &r));
      CHECK(r.done && r.crcOk && r.crc == j.outCrc && !(r.status & kStaErrorMask));
      CHECK(r.runUs > 0);
      // order: the six cache registers, then the three runs in ascending order without the start register, then the start register, then the interrupt clear
      std::vector<uint32_t> expect = {kRegCache0Size, kRegCache1Size, kRegCache2Size, kRegClrCache0, kRegClrCache1, kRegClrCache2};
      for (unsigned i = 8; i < 33; ++i)
        if (i != kRegStart) expect.push_back(4 * i);
      for (unsigned i = 64; i < 69; ++i) expect.push_back(4 * i);
      for (unsigned i = 128; i < 132; ++i) expect.push_back(4 * i);
      expect.push_back(4 * kRegStart);
      expect.push_back(kRegIrq);
      CHECK(m.order == expect);
      // the registers the driver wrote: the address registers hold bus addresses, the others the trace's values
      CHECK(m.reg[128 * 4] == Model::kBus + trace.bufOffset(1));
      CHECK(m.reg[129 * 4] == Model::kBus + trace.bufOffset(1));
      CHECK(m.reg[130 * 4] == Model::kBus + trace.bufOffset(2) + k * 4096u);
      CHECK(m.reg[131 * 4] == Model::kBus + trace.bufOffset(3) + 64);
      CHECK(m.reg[11 * 4] == 0x2c0de);
      CHECK(m.reg[10 * 4] == (0x100u | 1u));
      CHECK(m.reg[64 * 4] == 0xdead0000u + 64);
      CHECK(m.reg.count(34 * 4) == 0 && m.reg.count(70 * 4) == 0);   // not in a run: not written
      CHECK(m.reg[kRegIrq] == 0);
      m.reg.clear();
    }
    CHECK(m.violations.empty());
    for (auto &v : m.violations) printf("violation: %s\n", v.c_str());
    CHECK(m.starts == 3);
  }
  // a block that never answers: a timeout is reported, the run takes about the time asked
  {
    Model m(trace.windowBytes());
    m.neverFinish = true;
    m.pictureLen = f.outLen;
    Hw hw = hwFor(&m);
    Player p(hw, trace);
    CHECK(p.prepare());
    size_t cursor = 0;
    Job j;
    CHECK(trace.job(&cursor, &j));
    m.expectedStreamWord = f.streamWords[0];
    JobResult r;
    CHECK(!p.run(j, 500, &r));
    CHECK(!r.done && r.runUs > 500 && r.runUs < 600);
    CHECK(!r.crcOk);
  }
  // an error bit in the status: the picture may even have the right CRC, the job still fails
  {
    Model m(trace.windowBytes());
    m.errorBits = kStaError;
    m.pictureLen = f.outLen;
    Hw hw = hwFor(&m);
    Player p(hw, trace);
    CHECK(p.prepare());
    size_t cursor = 0;
    Job j;
    CHECK(trace.job(&cursor, &j));
    m.expectedStreamWord = f.streamWords[0];
    JobResult r;
    CHECK(!p.run(j, 500, &r));
    CHECK(r.done && r.crc == j.outCrc && !r.crcOk);
  }
  // a wrong picture: the CRC says so; a stale picture from before does not count (the player clears it)
  {
    Model m(trace.windowBytes());
    m.pictureLen = f.outLen - 1;   // one byte short: the last byte stays what the player cleared it to
    Hw hw = hwFor(&m);
    Player p(hw, trace);
    CHECK(p.prepare());
    size_t cursor = 0;
    Job j;
    CHECK(trace.job(&cursor, &j));
    m.expectedStreamWord = f.streamWords[0];
    JobResult r;
    CHECK(!p.run(j, 500, &r));
    CHECK(r.done && r.crc != j.outCrc && !r.crcOk);
    // a right picture, then the same job again with a block that writes nothing: the picture of the first run must not be taken for the second's
    m.pictureLen = f.outLen;
    CHECK(p.run(j, 500, &r));
    m.pictureLen = 0;
    CHECK(!p.run(j, 500, &r));
    CHECK(r.crc != j.outCrc);
  }
  // a window that is too small, or above 4 GiB, is refused
  {
    Model m(trace.windowBytes() - 4096);
    Hw hw = hwFor(&m);
    Player p(hw, trace);
    CHECK(!p.prepare());
    Model m2(trace.windowBytes());
    Hw hw2 = hwFor(&m2);
    hw2.dmaBus = 0xfffff000ull;
    Player p2(hw2, trace);
    CHECK(!p2.prepare());
  }
}

static std::vector<uint8_t> readFile(const char *path) {
  std::vector<uint8_t> data;
  FILE *f = fopen(path, "rb");
  if (!f) return data;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) data.insert(data.end(), buf, buf + n);
  fclose(f);
  return data;
}

// The committed traces: they parse, every job has the registers MPP writes for this block (the ranges of the vdpu34x register generator), the address
// registers are in the places the vendor kernel's translation table lists, and the first job brings in the stream.
static void testRealTrace(const char *path) {
  std::vector<uint8_t> t = readFile(path);
  CHECK(!t.empty());
  Trace trace;
  const bool ok = trace.parse(t.data(), t.size());
  if (!ok) printf("%s: %s\n", path, trace.error());
  CHECK(ok);
  if (!ok) return;
  printf("%s: %ux%u, %u jobs, %u buffers, window %u bytes\n", path, trace.width(), trace.height(), trace.jobCount(), trace.bufCount(), trace.windowBytes());
  CHECK(trace.width() > 0 && trace.height() > 0 && trace.jobCount() > 0);
  CHECK(trace.windowBytes() <= 16u << 20);
  size_t cursor = 0;
  Job j;
  for (unsigned n = 0; n < trace.jobCount(); ++n) {
    CHECK(trace.job(&cursor, &j));
    // the start register is in a run and (as MPP leaves it) has the enable bit clear; the decoded picture is a whole NV12 frame
    bool startInRun = false;
    for (unsigned i = 0; i < j.runCount; ++i)
      if (j.runs[i][0] <= kRegStart && kRegStart < j.runs[i][0] + j.runs[i][1]) startInRun = true;
    CHECK(startInRun);
    CHECK(j.outLen == trace.width() * trace.height() * 3 / 2);
    bool outReg = false;
    for (unsigned i = 0; i < j.relocCount; ++i)
      if (j.relocs[i].reg == 130 && j.relocs[i].buf == j.outBuf && j.relocs[i].offset == j.outOffset) outReg = true;
    CHECK(outReg);
    if (n == 0) {
      uint32_t bytes = 0;
      for (unsigned i = 0; i < j.uploadCount; ++i) bytes += j.uploads[i].len;
      CHECK(bytes > 0);
    }
  }

  // Replayed against the model, the picture CRCs cannot match (the model does not decode), but everything else must be as for the synthetic trace:
  // every job starts once, with the address registers inside the window and the stream where reg 128 says.
  Model m(trace.windowBytes());
  m.polls = 1;
  m.pictureLen = 0;
  Hw hw = hwFor(&m);
  Player p(hw, trace);
  CHECK(p.prepare());
  cursor = 0;
  for (unsigned n = 0; n < trace.jobCount(); ++n) {
    CHECK(trace.job(&cursor, &j));
    JobResult r;
    m.expectedStreamWord = 0;   // the stream word is checked below instead
    m.violations.clear();
    p.run(j, 100, &r);
    CHECK(r.done);
    for (unsigned i = 0; i < j.relocCount; ++i) CHECK(m.inWindow(m.reg[4 * j.relocs[i].reg]));
    m.reg.clear();
  }
  CHECK(m.starts == trace.jobCount());
}

int main(int argc, char **argv) {
  testParse();
  testPlay();
  for (int i = 1; i < argc; ++i) testRealTrace(argv[i]);
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail != 0;
}
