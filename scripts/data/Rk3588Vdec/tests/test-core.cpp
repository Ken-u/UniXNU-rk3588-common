// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// test-core.cpp -- host test of the decoder core (core/): MPP's H.264 parser and vdpu34x register generator, run in a program of ours on a stream, must
// produce exactly the decode jobs MPP produced on a real board when it decoded the same stream (traces/*.vdt, recorded with tools/trace-shim.c).
//
//   test-core STREAM.h264 TRACE.vdt STREAM.crc ...     (three arguments per stream)
//
// There is no block here: the "block" accepts every job and answers "done". For each job the test checks, against the trace's job of the same number:
//   - the same registers are written, with the same values (the address registers excepted),
//   - the address registers point at the same place: same offset into a buffer, and the buffers correspond one to one over the whole stream,
//   - the CPU-written content of every buffer the job uses (the stream, the parameter sets, the reference lists, the scaling lists, the CABAC tables) is
//     byte for byte what the trace has when its player starts the job on the board.
// And at the end: as many pictures come out as the reference has, in the reference's order, each picture being the output of the job whose recorded CRC
// is the reference's CRC of that picture (the job markers of make-trace.py say which job made which picture; here the order alone has to agree).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "../VdecTrace.h"
extern "C" {
#include "../core/VdecCore.h"
#include "../core/VdecPlatform.h"
}

using namespace vdec;

static int g_checks = 0, g_fail = 0;
static int g_verbose = 0;
#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_fail;                                                              \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                 \
    }                                                                        \
  } while (0)
#define CHECKF(cond, ...)                                                    \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_fail;                                                              \
      printf("FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond);                 \
      printf(__VA_ARGS__);                                                   \
      printf("\n");                                                          \
    }                                                                        \
  } while (0)

// ---- platform hooks of the core
static long g_live = 0;
extern "C" void *vdc_calloc(size_t size) {
  ++g_live;
  return calloc(1, size);
}
extern "C" void vdc_free(void *p, size_t) {
  if (p) --g_live;
  free(p);
}
extern "C" void vdc_log(int level, const char *line) {
  if (level <= 2 || g_verbose) printf("  [mpp %d] %s\n", level, line);
}

static std::vector<uint8_t> readFile(const std::string &path) {
  std::vector<uint8_t> v;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return v;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) v.insert(v.end(), buf, buf + n);
  fclose(f);
  return v;
}

static std::vector<uint32_t> readCrcs(const std::string &path) {
  std::vector<uint32_t> v;
  FILE *f = fopen(path.c_str(), "r");
  if (!f) return v;
  char line[256];
  while (fgets(line, sizeof line, f))
    if (line[0] != '#') v.push_back(static_cast<uint32_t>(strtoul(line, nullptr, 16)));
  fclose(f);
  return v;
}

// ---- the stand-in for the block, and the comparison with the trace
struct Harness {
  const Trace *trace = nullptr;
  size_t cursor = 0;
  std::vector<Job> jobs;                    // the trace's jobs, in order
  std::vector<uint8_t> model;               // the trace's buffers as its player has them
  std::vector<uint8_t> window;
  uint64_t bus = 0x65000000;
  uint32_t jobIndex = 0;
  std::map<uint32_t, uint32_t> toMine, toTrace;   // trace buffer -> shim buffer and back
  std::map<uint32_t, uint32_t> lastJobOfBuffer;   // start of a buffer in the window -> the job that last decoded into it
  std::vector<uint32_t> jobOrderOfPictures;
  int mismatches = 0;
  uint8_t *virt = nullptr;

  void report(const char *fmt, uint32_t a = 0, uint32_t b = 0, uint32_t c = 0, uint32_t d = 0) {
    if (mismatches++ < 12) {
      printf("  job %u: ", jobIndex);
      printf(fmt, a, b, c, d);  // NOLINT
      printf("\n");
    }
  }

  static void before(void *ctx, const VdecJobInfo *job) { static_cast<Harness *>(ctx)->compare(job); }
  static void after(void *, const VdecJobInfo *, int, uint32_t) {}

  void compare(const VdecJobInfo *job) {
    if (jobIndex >= jobs.size()) {
      report("more jobs than the trace has (%u)", static_cast<uint32_t>(jobs.size()));
      ++jobIndex;
      return;
    }
    const Job &tj = jobs[jobIndex];
    for (uint32_t i = 0; i < tj.uploadCount; ++i) {
      const Upload &u = tj.uploads[i];
      memcpy(&model[trace->bufOffset(u.buf) + u.offset], u.data, u.len);
    }
    // the registers written
    bool mine[kRegCount] = {}, theirs[kRegCount] = {};
    for (unsigned i = 0; i < job->nRuns; ++i)
      for (unsigned r = job->runs[i].first; r < static_cast<unsigned>(job->runs[i].first + job->runs[i].count); ++r) mine[r] = true;
    for (uint32_t i = 0; i < tj.runCount; ++i)
      for (uint32_t r = tj.runs[i][0]; r < tj.runs[i][0] + tj.runs[i][1]; ++r) theirs[r] = true;
    bool isAddr[kRegCount] = {};
    for (uint32_t i = 0; i < tj.relocCount; ++i) isAddr[tj.relocs[i].reg] = true;
    for (unsigned i = 0; i < job->nReloc; ++i) {
      if (job->reloc[i].reg < kRegCount && !isAddr[job->reloc[i].reg]) report("register %u is an address here but not in the trace", job->reloc[i].reg);
    }
    for (uint32_t r = 0; r < kRegCount; ++r) {
      if (mine[r] != theirs[r]) {
        report("register %u is written %s here, %s in the trace", r, mine[r] ? 1 : 0, theirs[r] ? 1 : 0);
        continue;
      }
      if (mine[r] && !isAddr[r] && job->regs[r] != tj.regs[r]) report("register %u = %#x, the trace has %#x", r, job->regs[r], tj.regs[r]);
    }
    if (g_verbose) {
      printf("  job %u relocs (reg: trace buf+off / shim buf+off size):", jobIndex);
      for (uint32_t i = 0; i < tj.relocCount; ++i) {
        const VdecReloc *mr = nullptr;
        for (unsigned k = 0; k < job->nReloc; ++k)
          if (job->reloc[k].reg == tj.relocs[i].reg) mr = &job->reloc[k];
        printf(" %u: %u+%u / ", tj.relocs[i].reg, tj.relocs[i].buf, tj.relocs[i].offset);
        if (mr) printf("%u+%u %u;", mr->buf, mr->offset, mr->bufSize);
        else printf("-;");
      }
      printf("\n");
    }
    // the address registers
    std::map<uint32_t, uint32_t> checkedBufs;   // trace buffer -> start in my window
    for (uint32_t i = 0; i < tj.relocCount; ++i) {
      const Reloc &tr = tj.relocs[i];
      const VdecReloc *mr = nullptr;
      for (unsigned k = 0; k < job->nReloc; ++k)
        if (job->reloc[k].reg == tr.reg) mr = &job->reloc[k];
      if (!mr) {
        report("address register %u is not set here", tr.reg);
        continue;
      }
      if (mr->offset != tr.offset) report("address register %u: offset %u here, %u in the trace", tr.reg, mr->offset, tr.offset);
      auto a = toMine.find(tr.buf);
      if (a == toMine.end()) {
        auto b = toTrace.find(mr->buf);
        if (b != toTrace.end() && b->second != tr.buf)
          report("shim buffer %u was trace buffer %u and is now trace buffer %u", mr->buf, b->second, tr.buf);
        toMine[tr.buf] = mr->buf;
        toTrace[mr->buf] = tr.buf;
        if (trace->bufSize(tr.buf) != mr->bufSize && ((mr->bufSize + 4095) & ~4095u) != ((trace->bufSize(tr.buf) + 4095) & ~4095u))
          report("buffer %u: %u bytes here, %u in the trace", tr.buf, mr->bufSize, trace->bufSize(tr.buf));
      } else if (a->second != mr->buf) {
        report("trace buffer %u is shim buffer %u, then %u", tr.buf, a->second, mr->buf);
      }
      uint32_t start = static_cast<uint32_t>(job->regs[tr.reg] - bus) - mr->offset;
      checkedBufs[tr.buf] = start;
      if (tr.reg == 130) lastJobOfBuffer[start] = jobIndex;
    }
    // the CPU-written content
    for (auto &kv : checkedBufs) {
      uint32_t size = std::min<uint32_t>(trace->bufSize(kv.first), 1u << 30);
      const uint8_t *m = &model[trace->bufOffset(kv.first)];
      const uint8_t *w = virt + kv.second;
      uint32_t at = 0;
      while (at < size && m[at] == w[at]) ++at;
      if (at < size) {
        // the picture buffers hold what the block writes (nothing here) and the job's own output is not to be compared
        report("buffer %u differs from the trace at byte %u (here %#x, trace %#x)", kv.first, at, w[at], m[at]);
      }
    }
    // the picture this job makes
    {
      const Reloc *cur = nullptr;
      for (uint32_t i = 0; i < tj.relocCount; ++i)
        if (tj.relocs[i].reg == 130) cur = &tj.relocs[i];
      if (!cur || cur->buf != tj.outBuf || cur->offset != tj.outOffset) report("the trace's picture is not the one register 130 points at");
    }
    ++jobIndex;
  }

  static int run(void *, const uint32_t *, const VdecRun *, unsigned, uint32_t *status) {
    *status = kStaIrqRaw | kStaReady;
    return 0;
  }
};

struct Pictures {
  Harness *h;
  std::vector<uint32_t> jobs;   // for every picture that came out: the job that decoded it
  std::vector<uint32_t> poc;
  static void sink(void *ctx, const VdecPicture *p) {
    Pictures *self = static_cast<Pictures *>(ctx);
    uint32_t start = static_cast<uint32_t>(p->data - self->h->virt);
    auto it = self->h->lastJobOfBuffer.find(start);
    self->jobs.push_back(it == self->h->lastJobOfBuffer.end() ? 0xffffffffu : it->second);
    self->poc.push_back(p->poc);
    CHECK(p->width > 0 && p->height > 0 && p->hstride >= p->width && p->vstride >= p->height);
    CHECK(p->bytes >= p->hstride * p->vstride * 3 / 2);
  }
};

// The stream goes in as one piece (chunk 0) or in pieces of `chunk` bytes, the last one with the end-of-stream flag.
static void testStream(const std::string &streamPath, const std::string &tracePath, const std::string &crcPath, size_t chunk) {
  printf("%s, %s\n", streamPath.c_str(), chunk ? (std::to_string(chunk) + " byte pieces").c_str() : "in one piece");
  std::vector<uint8_t> stream = readFile(streamPath), traceBytes = readFile(tracePath);
  std::vector<uint32_t> refCrc = readCrcs(crcPath);
  CHECK(!stream.empty() && !traceBytes.empty() && !refCrc.empty());
  if (stream.empty() || traceBytes.empty() || refCrc.empty()) return;
  Trace trace;
  CHECKF(trace.parse(traceBytes.data(), traceBytes.size()), "%s", trace.error());

  Harness h;
  h.trace = &trace;
  h.model.assign(trace.windowBytes(), 0);
  h.window.assign(64u << 20, 0);
  h.virt = h.window.data();
  size_t cursor = 0;
  for (uint32_t i = 0; i < trace.jobCount(); ++i) {
    Job j;
    CHECK(trace.job(&cursor, &j));
    h.jobs.push_back(j);
  }

  VdecHw hw = {&h, h.virt, h.bus, static_cast<uint32_t>(h.window.size()), Harness::run};
  VdecCore *core = nullptr;
  CHECK(vdec_core_create(&core, &hw, kVdecH264) == 0);
  if (!core) return;
  VdecJobObserver obs = {&h, Harness::before, Harness::after};
  vdec_core_set_observer(core, &obs);
  Pictures pics{&h, {}, {}};
  VdecSink sink = {&pics, Pictures::sink, nullptr};
  int n = 0;
  for (size_t at = 0; at < stream.size();) {
    size_t len = chunk ? std::min(chunk, stream.size() - at) : stream.size();
    int r = vdec_core_decode(core, stream.data() + at, len, static_cast<int64_t>(at), at + len == stream.size(), &sink);
    CHECK(r >= 0);
    if (r < 0) break;
    n += r;
    at += len;
  }
  CHECKF(n == static_cast<int>(refCrc.size()), "%d pictures came out, the reference has %zu", n, refCrc.size());
  VdecStats st;
  vdec_core_stats(core, &st);
  CHECKF(st.jobs == trace.jobCount(), "%u jobs here, %u in the trace", st.jobs, trace.jobCount());
  CHECKF(h.mismatches == 0, "%d differences from the trace", h.mismatches);
  CHECK(pics.jobs.size() == refCrc.size());
  for (size_t d = 0; d < pics.jobs.size() && d < refCrc.size(); ++d) {
    bool ok = pics.jobs[d] < h.jobs.size() && h.jobs[pics.jobs[d]].outCrc == refCrc[d];
    CHECKF(ok, "picture %zu (poc %u) came from job %d, whose picture has CRC %08x; the reference's is %08x", d, pics.poc[d],
           static_cast<int>(pics.jobs[d]), pics.jobs[d] < h.jobs.size() ? h.jobs[pics.jobs[d]].outCrc : 0, refCrc[d]);
  }
  printf("  %u jobs, %u pictures, window peak %u KiB\n", st.jobs, st.pictures, st.windowPeakBytes >> 10);
  vdec_core_destroy(core);
  // MPP keeps a few pools (frames, packets, meta) for the life of the process, created on first use: after the first stream the number of live allocations
  // must not move any more, whatever is decoded and however often a core is made and destroyed.
  static long baseline = -1;
  if (baseline < 0) baseline = g_live;
  CHECKF(g_live == baseline, "%ld allocations live after this stream, %ld after the first", g_live, baseline);
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  int first = 1;
  if (argc > 1 && !strcmp(argv[1], "-v")) {
    g_verbose = 1;
    vdc_log_level = 6;
    first = 2;
  }
  if ((argc - first) % 3 != 0 || argc == first) {
    printf("usage: test-core [-v] STREAM TRACE CRCS ...\n");
    return 2;
  }
  for (size_t chunk : {size_t(0), size_t(4093), size_t(1000), size_t(7)})
    for (int i = first; i + 2 < argc; i += 3) testStream(argv[i], argv[i + 1], argv[i + 2], chunk);
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
