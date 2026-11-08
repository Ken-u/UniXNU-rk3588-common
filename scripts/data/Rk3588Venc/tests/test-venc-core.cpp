// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Host test of core/: the encoder made of MPP's own code (core/VencCore.c over core/EncShim.c) is run on pictures like the ones the recordings were made with, with
// the same settings, against a fake block that answers "done, no error, 4000 bytes" (what tools/trace-shim-enc.c answered when the recordings were made). Every job
// the core submits is compared with the recording of the same job made by the unmodified MPP on the host (traces/*.vet):
//   - the set of registers written is the same, and each has the recorded value (the address registers: the same offset into a buffer of the same size);
//   - the pages the CPU wrote (parameter tables, scaling lists, rate control tables; the source picture is ours, so those pages are skipped) hold the recorded bytes;
//   - the access unit that comes back starts with the parameter sets on IDR pictures and has no more bytes than the block was said to write.
// A core that agrees with MPP on every job of six recordings (fixed QP, constant bit rate with its feedback, IDR every third picture, three sizes) is the
// encoder MPP is.
//
// usage: [VENC_TRACES=DIR] [VENC_LOG=LEVEL] test-venc-core [RECORDING-NAME...]   (default: all six; MPP's own messages up to VENC_LOG, 6 is everything)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <map>
#include <string>
#include <vector>

#include "../VencTrace.h"
#include "../core/VencPlatform.h"
#include "../core/VencCore.h"

// what the core asks of its embedder
extern "C" {
void *vec_calloc(size_t size) { return calloc(1, size); }
void vec_free(void *p, size_t size) {
  (void)size;
  free(p);
}
void vec_log(int level, const char *line) {
  static const int shown = getenv("VENC_LOG") ? atoi(getenv("VENC_LOG")) : 3;
  if (level <= shown) printf("  [mpp %d] %s\n", level, line);
}
int64_t vec_time_us(void) {
  static int64_t now;
  return now += 1;   // a clock that moves a microsecond per look: deterministic, and (as on the host the recordings were made on, where a picture takes well under a millisecond
                     // of work) never the quarter of a second after which the rate control starts to measure its real frame rate
}
}

namespace {

int gChecks = 0, gFailures = 0;
#define CHECK(cond)                                                      \
  do {                                                                   \
    ++gChecks;                                                           \
    if (!(cond)) {                                                       \
      ++gFailures;                                                       \
      if (gFailures < 60) printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                                    \
  } while (0)

std::vector<uint8_t> readFile(const std::string &path) {
  FILE *f = fopen(path.c_str(), "rb");
  std::vector<uint8_t> data;
  if (!f) return data;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) data.insert(data.end(), buf, buf + n);
  fclose(f);
  return data;
}

constexpr uint64_t kBus = 0x6a000000ull;
constexpr uint32_t kWindow = 32u << 20;

// How each recording was made (tools/make-enc-trace.sh, UniXNU-orangepi5/evidence/host-venc-20261108.txt): MPP's test tool, planar input.
struct Recipe {
  const char *name;
  int frames;
  int rcMode;
  int qp;
  uint32_t bitrate, bitrateMin, bitrateMax;
  uint32_t gop;
};
const Recipe kRecipes[] = {
    {"e320_h264_ip", 2, kVencRcFixQp, 28, 0, 0, 0, 0},
    {"e320_h264_p5", 6, kVencRcFixQp, 28, 0, 0, 0, 0},
    {"e320_h264_cbr", 6, kVencRcCbr, 0, 300000, 200000, 400000, 0},
    {"e320_h264_gop", 7, kVencRcFixQp, 24, 0, 0, 0, 3},
    {"e360_h264", 3, kVencRcFixQp, 30, 0, 0, 0, 0},
    {"e720_h264", 3, kVencRcFixQp, 30, 0, 0, 0, 0},
};

struct Fake {
  std::vector<uint8_t> window;
  uint8_t *base = nullptr;
  uint32_t streamLength = 4000;
  int runs = 0;
  int reads = 0;
  uint32_t lastLength = 0;
};

int fakeRun(void *ctx, const uint32_t *regs, const VencRun *runs, unsigned nRuns, uint32_t *status) {
  (void)regs;
  (void)runs;
  (void)nRuns;
  static_cast<Fake *>(ctx)->runs++;
  *status = 1;   // int_sta: done, no error
  return 0;
}

void fakeReadRegs(void *ctx, uint32_t byteOffset, uint32_t *dst, unsigned words) {
  Fake *fake = static_cast<Fake *>(ctx);
  memset(dst, 0, 4u * words);
  if (byteOffset == 0x4000 && words >= 1) {
    // st.bs_lgth_l32. tools/trace-shim-enc.c, which made the recordings, answered one queued read per poll and MPP queues two per job (the interrupt status and the
    // status class, for two register sets in turn): the status of every second job was not delivered. The rate control saw those pictures as empty (bit_real 0),
    // which is why the constant-bit-rate recordings have the quantiser going 23 22 25 24 27 26. The same history here.
    fake->lastLength = (fake->reads++ & 1) ? fake->streamLength : 0;
    dst[0] = fake->lastLength;
  }
}

struct Compare {
  const venc::Trace *trace = nullptr;
  size_t cursor = 0;
  int jobIndex = 0;
  Fake *fake = nullptr;
  const uint8_t *source = nullptr;   // the core's source buffer (skipped when comparing uploads)
  uint32_t sourceBytes = 0;
  std::map<uint32_t, uint32_t> mine;   // recorded buffer id -> offset of my buffer in the window
  std::map<uint32_t, uint32_t> bufSize;
  std::string name;
  int mismatches = 0;
  uint32_t streamBuf = 0;   // the recorded id of the bit stream buffer (the one the stream address register, 175, points into)
};

void before(void *ctx, const VencJobInfo *info) {
  Compare *c = static_cast<Compare *>(ctx);
  venc::Job job;
  const bool have = c->trace->job(&c->cursor, &job);
  CHECK(have);
  if (!have) return;
  const std::string tag = c->name + " job " + std::to_string(c->jobIndex);

  // the registers written
  std::vector<uint8_t> written(venc::kRegCount, 0);
  for (unsigned r = 0; r < info->nRuns; ++r)
    for (unsigned w = 0; w < info->runs[r].count; ++w) written[info->runs[r].first + w] = 1;
  std::vector<uint8_t> recorded(venc::kRegCount, 0);
  for (uint32_t i = 0; i < job.regCount; ++i) recorded[job.regs[i].index] = 1;
  std::map<uint32_t, const venc::Reloc *> relocOf;
  for (uint32_t i = 0; i < job.relocCount; ++i) relocOf[job.relocs[i].reg] = &job.relocs[i];
  std::map<uint32_t, const VencReloc *> myRelocOf;
  for (unsigned i = 0; i < info->nReloc; ++i) myRelocOf[info->reloc[i].reg] = &info->reloc[i];

  int missing = 0, extra = 0, wrong = 0;
  for (unsigned i = 0; i < venc::kRegCount; ++i) {
    if (i == venc::kRegStart) continue;   // the start register is compared below
    if (recorded[i] && !written[i]) {
      if (missing++ < 4) printf("  %s: register %u recorded, not written\n", tag.c_str(), i);
    }
    if (!recorded[i] && written[i]) {
      if (extra++ < 4) printf("  %s: register %u written (0x%08x), not recorded\n", tag.c_str(), i, info->rawRegs[i]);
    }
  }
  for (uint32_t i = 0; i < job.regCount; ++i) {
    const uint32_t idx = job.regs[i].index;
    if (idx == venc::kRegStart || !written[idx] || relocOf.count(idx)) continue;
    if (info->rawRegs[idx] != job.regs[i].value) {
      if (wrong++ < 8) printf("  %s: register %u is 0x%08x, recorded 0x%08x\n", tag.c_str(), idx, info->rawRegs[idx], job.regs[i].value);
    }
  }
  // start register
  for (uint32_t i = 0; i < job.regCount; ++i)
    if (job.regs[i].index == venc::kRegStart) {
      CHECK(written[venc::kRegStart]);
      if (info->rawRegs[venc::kRegStart] != job.regs[i].value) {
        printf("  %s: start register is 0x%08x, recorded 0x%08x\n", tag.c_str(), info->rawRegs[venc::kRegStart], job.regs[i].value);
        wrong++;
      }
    }
  // the address registers: same registers, same offset into a buffer of the same size
  int relocWrong = 0;
  CHECK(info->nReloc == job.relocCount);
  for (auto &kv : relocOf) {
    auto it = myRelocOf.find(kv.first);
    if (it == myRelocOf.end()) {
      if (relocWrong++ < 6) printf("  %s: address register %u recorded, not written\n", tag.c_str(), kv.first);
      continue;
    }
    const venc::Reloc *rec = kv.second;
    const VencReloc *my = it->second;
    if (my->offset != rec->offset || my->bufSize != c->trace->bufSize(rec->buf)) {
      if (relocWrong++ < 6)
        printf("  %s: address register %u: offset %u buffer %u bytes, recorded offset %u buffer %u bytes\n", tag.c_str(), kv.first, my->offset, my->bufSize, rec->offset,
               c->trace->bufSize(rec->buf));
    }
    // which of my buffers is the recorded one
    const uint32_t bus = info->regs[my->reg];
    const uint32_t winOff = static_cast<uint32_t>(bus - kBus - my->offset);
    auto prev = c->mine.find(rec->buf);
    if (prev == c->mine.end()) c->mine[rec->buf] = winOff;
    else if (prev->second != winOff) {
      if (relocWrong++ < 6) printf("  %s: recorded buffer %u is at window +%u, was at +%u in an earlier job\n", tag.c_str(), rec->buf, winOff, prev->second);
    }
  }
  for (auto &kv : myRelocOf)
    if (!relocOf.count(kv.first)) {
      if (relocWrong++ < 6) printf("  %s: address register %u written, not recorded\n", tag.c_str(), kv.first);
    }

  // the pages the CPU wrote
  if (relocOf.count(175)) c->streamBuf = relocOf[175]->buf;
  int uploadWrong = 0;
  const uint32_t sourceOff = static_cast<uint32_t>(c->source - c->fake->base);
  for (uint32_t i = 0; i < job.uploadCount; ++i) {
    const venc::Upload &u = job.uploads[i];
    auto at = c->mine.find(u.buf);
    if (at == c->mine.end()) {
      if (uploadWrong++ < 6) printf("  %s: upload into recorded buffer %u that no address register of ours has reached\n", tag.c_str(), u.buf);
      continue;
    }
    if (at->second == sourceOff) continue;   // the source picture: ours
    if (c->jobIndex == 0 && u.buf == c->streamBuf) continue;   // the parameter sets MPP's test tool asked for were written into the stream buffer (and the block overwrites them)
    if (memcmp(c->fake->base + at->second + u.offset, u.data, u.len) != 0) {
      if (uploadWrong++ < 6) {
        uint32_t k = 0;
        while (k < u.len && c->fake->base[at->second + u.offset + k] == u.data[k]) ++k;
        printf("  %s: upload into buffer %u +%u (%u bytes) differs from byte %u (ours 0x%02x, recorded 0x%02x)\n", tag.c_str(), u.buf, u.offset, u.len, k,
               c->fake->base[at->second + u.offset + k], u.data[k]);
      }
    }
  }

  CHECK(missing == 0);
  CHECK(extra == 0);
  CHECK(wrong == 0);
  CHECK(relocWrong == 0);
  CHECK(uploadWrong == 0);
  if (missing || extra || wrong || relocWrong || uploadWrong) c->mismatches++;
  c->jobIndex++;
}

void after(void *ctx, const VencJobInfo *info, int rc, uint32_t status) {
  (void)ctx;
  (void)info;
  CHECK(rc == 0);
  CHECK(status == 1);
}

void fillPicture(uint8_t *dst, uint32_t hstride, uint32_t vstride, uint32_t width, uint32_t height, int frame) {
  // something that is not flat (planar 4:2:0): the content cannot change the registers of these recordings
  memset(dst, 128, hstride * vstride * 3 / 2);
  for (uint32_t y = 0; y < height; ++y)
    for (uint32_t x = 0; x < width; ++x) dst[y * hstride + x] = static_cast<uint8_t>(x + 2 * y + 7 * frame);
}

bool runRecipe(const Recipe &recipe, const std::string &dir) {
  const std::string path = dir + "/" + recipe.name + ".vet";
  std::vector<uint8_t> data = readFile(path);
  venc::Trace trace;
  if (data.empty() || !trace.parse(data.data(), data.size())) {
    printf("%s: cannot read the trace (%s)\n", path.c_str(), data.empty() ? "no file" : trace.error());
    return false;
  }
  CHECK(trace.jobCount() == static_cast<uint32_t>(recipe.frames));

  Fake fake;
  fake.window.assign(kWindow + 4096, 0);
  fake.base = reinterpret_cast<uint8_t *>((reinterpret_cast<uintptr_t>(fake.window.data()) + 4095) & ~uintptr_t(4095));
  VencHw hw = {};
  hw.ctx = &fake;
  hw.dmaVirt = fake.base;
  hw.dmaBus = kBus;
  hw.dmaBytes = kWindow;
  hw.run = fakeRun;
  hw.readRegs = fakeReadRegs;

  VencParams params = {};
  params.codec = kVencH264;
  params.width = trace.width();
  params.height = trace.height();
  params.format = kVencI420;
  params.rcMode = recipe.rcMode;
  params.qp = recipe.qp;
  params.bitrate = recipe.bitrate;
  params.bitrateMin = recipe.bitrateMin;
  params.bitrateMax = recipe.bitrateMax;
  params.fpsNum = 30;
  params.fpsDen = 1;
  params.gop = recipe.gop;
  params.motionInfo = 1;
  if (getenv("VENC_PERTURB")) {   // negative control (mutate-core.sh): one setting differs from the recording's, the comparison must notice
    params.qp += 1;
    params.bitrate += 4000;
    params.gop += 1;
  }
  VencCore *core = nullptr;
  const int rc = venc_core_create(&core, &hw, &params);
  CHECK(rc == 0);
  if (rc) {
    printf("%s: venc_core_create failed (%d)\n", recipe.name, rc);
    return false;
  }

  Compare compare;
  compare.trace = &trace;
  compare.fake = &fake;
  compare.name = recipe.name;
  VencJobObserver observer = {&compare, before, after};
  venc_core_set_observer(core, &observer);

  // MPP's test tool asks for the parameter sets once before the first picture (and writes them to the file itself): the first job has no header in front
  uint8_t header[512];
  const int headerLength = venc_core_header(core, header, sizeof header);
  CHECK(headerLength > 8);
  CHECK(header[0] == 0 && header[1] == 0 && header[2] == 0 && header[3] == 1 && (header[4] & 0x1f) == 7);   // an SPS first, Annex B

  uint32_t sourceBytes = 0;
  uint8_t *source = venc_core_source(core, &sourceBytes);
  compare.source = source;
  compare.sourceBytes = sourceBytes;
  const uint32_t hstride = (trace.width() + 15) & ~15u, vstride = (trace.height() + 1) & ~1u;
  for (int f = 0; f < recipe.frames; ++f) {
    fillPicture(source, hstride, vstride, trace.width(), trace.height(), f);
    VencPacket packet;
    const int er = venc_core_encode(core, f, 0, &packet);
    CHECK(er == 0);
    if (er) {
      printf("%s: picture %d: encode failed (%d)\n", recipe.name, f, er);
      break;
    }
    CHECK(packet.data != nullptr);
    CHECK(packet.length == packet.hwOffset + fake.lastLength);   // what the block wrote, and the parameter sets in front of an IDR picture
    const bool idr = f == 0 || (recipe.gop && f % recipe.gop == 0);
    CHECK((packet.isIdr != 0) == idr);
    if (idr && f > 0) {
      CHECK(packet.hwOffset > 0);   // parameter sets in front
      CHECK(packet.data[0] == 0 && packet.data[1] == 0 && packet.data[2] == 0 && packet.data[3] == 1);
      CHECK((packet.data[4] & 0x1f) == 7);   // an SPS first
    }
    if (f == 0 && (recipe.rcMode != kVencRcFixQp || recipe.gop)) printf("  %s picture %d: %u bytes, qp %d, idr %d\n", recipe.name, f, packet.length, packet.qp, packet.isIdr);
  }
  CHECK(compare.jobIndex == recipe.frames);
  VencStats stats;
  venc_core_stats(core, &stats);
  CHECK(stats.frames == static_cast<uint32_t>(recipe.frames));
  CHECK(stats.jobErrors == 0);
  printf("%-16s %d pictures, %d jobs compared, %s (window peak %u KiB)\n", recipe.name, recipe.frames, compare.jobIndex, compare.mismatches ? "MISMATCH" : "all registers, address offsets and pages equal",
         stats.windowPeakBytes >> 10);
  venc_core_destroy(core);
  return compare.mismatches == 0;
}

}  // namespace

int main(int argc, char **argv) {
  std::string dir = "traces";
  if (const char *d = getenv("VENC_TRACES")) dir = d;
  if (const char *l = getenv("VENC_LOG")) vec_log_level = atoi(l);
  bool ok = true;
  for (const Recipe &recipe : kRecipes) {
    bool wanted = argc < 2;   // names on the command line pick recordings
    for (int i = 1; i < argc; ++i) wanted = wanted || strcmp(argv[i], recipe.name) == 0;
    if (wanted) ok = runRecipe(recipe, dir) && ok;
  }
  printf("%d checks, %d failures\n", gChecks, gFailures);
  return gFailures == 0 && ok ? 0 : 1;
}
