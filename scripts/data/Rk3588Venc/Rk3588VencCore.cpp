// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The encoder core in the kernel (core/VencCore.h: Rockchip's MPP H.264 encoder over the shim of core/EncShim.c): what it asks of its embedder, the board
// self-test of boot argument rk3588-venc=6, and the streams the AppleAVE2 service (Rk3588VencAve2.cpp) encodes the application's pictures with.
//
// The self-test makes pictures itself (a pattern that tools/check-core-output.py reproduces), encodes them in sizes and at rates nobody recorded, and prints a line
// per picture; with rk3588-venc-core-dump=1 also the picture's bytes, 32 to a line, which the checker decodes with libavc and compares with the source. The kernel stack
// is 16 KiB: the stack the core really uses is measured by painting the free part of it first.
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <stdint.h>

#include "Rk3588Venc.h"
#include "VencTrace.h"
#include "core/VencCore.h"
#include "core/VencPlatform.h"

namespace {

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

}  // namespace

// ---- what the core asks of the program it is linked into (core/VencPlatform.h)

extern "C" {

void *vec_calloc(size_t size) { return IOMallocZero(size); }
void vec_free(void *p, size_t size) { IOFree(p, size); }
void vec_log(int level, const char *line) { printf("rk3588-venc: mpp %d: %s\n", level, line); }
int64_t vec_time_us(void) { return static_cast<int64_t>(nowUs()); }

}  // extern "C"

namespace {

// ---- stack use: the part of the stack below the driver thread's frame is painted before a run and looked at afterwards
constexpr uintptr_t kPaintBytes = 12 * 1024, kPaintGap = 64;
constexpr uint8_t kPaint = 0xa5;

// Paints the stack below the caller's frame and returns where the paint starts: that is the reference for stackDepth(). It has to be measured from this function's
// own frame, not from a caller's frame address with a fixed gap: a caller that has the encoder's functions inlined has a frame of more than a kilobyte, and
// paint over it overwrote that caller's own variables (board runs vc1 and vt2 lost the dump request that way: the one more call after the argument was read in
// vc3 and vt3 moved the variable).
__attribute__((noinline)) uintptr_t paintStack() {
  const uintptr_t top = reinterpret_cast<uintptr_t>(__builtin_frame_address(0)) - kPaintGap;
  volatile uint8_t *p = reinterpret_cast<volatile uint8_t *>(top);
  for (uintptr_t i = 0; i < kPaintBytes; ++i) p[-static_cast<intptr_t>(i)] = kPaint;
  return top;
}

// How many bytes below the top the deepest write reached (the paint's extent, plus the gap, when it went all the way).
uintptr_t stackDepth(uintptr_t top) {
  const volatile uint8_t *low = reinterpret_cast<volatile uint8_t *>(top - kPaintBytes + 1);
  for (uintptr_t i = 0; i < kPaintBytes; ++i)
    if (low[i] != kPaint) return top - reinterpret_cast<uintptr_t>(low + i);
  return 0;
}

// ---- the pictures
// Planar 4:2:0 (I420). The pattern has edges and gradients that move from picture to picture; tools/check-core-output.py makes the same.
void fillPicture(uint8_t *dst, uint32_t hstride, uint32_t vstride, uint32_t width, uint32_t height, unsigned frame) {
  uint8_t *u = dst + hstride * vstride, *v = u + (hstride / 2) * (vstride / 2);
  for (uint32_t y = 0; y < height; ++y)
    for (uint32_t x = 0; x < width; ++x) dst[y * hstride + x] = static_cast<uint8_t>(x + 2 * y + 7 * frame + (((x >> 4) & (y >> 4)) * 9));
  for (uint32_t y = 0; y < height / 2; ++y)
    for (uint32_t x = 0; x < width / 2; ++x) {
      u[y * (hstride / 2) + x] = static_cast<uint8_t>(112 + ((3 * x + 5 * frame) & 31));
      v[y * (hstride / 2) + x] = static_cast<uint8_t>(112 + ((3 * y + 7 * frame) & 31));
    }
}

// The same picture as semi-planar 4:2:0 (NV12): luma, then U and V interleaved, `hstride` bytes per row in both.
void fillPictureNv12(uint8_t *dst, uint32_t hstride, uint32_t vstride, uint32_t width, uint32_t height, unsigned frame) {
  uint8_t *uv = dst + hstride * vstride;
  for (uint32_t y = 0; y < height; ++y)
    for (uint32_t x = 0; x < width; ++x) dst[y * hstride + x] = static_cast<uint8_t>(x + 2 * y + 7 * frame + (((x >> 4) & (y >> 4)) * 9));
  for (uint32_t y = 0; y < height / 2; ++y)
    for (uint32_t x = 0; x < width / 2; ++x) {
      uv[y * hstride + 2 * x] = static_cast<uint8_t>(112 + ((3 * x + 5 * frame) & 31));
      uv[y * hstride + 2 * x + 1] = static_cast<uint8_t>(112 + ((3 * y + 7 * frame) & 31));
    }
}

struct Recipe {
  const char *name;
  int codec;          // kVencH264 or kVencH265
  uint32_t width, height;
  unsigned frames;
  int rcMode;
  int qp;
  uint32_t bitrate;   // bit/s
  uint32_t gop;
  bool nv12;          // semi-planar source with 16-aligned rows (what the AppleAVE2 service hands the core), else planar
};
// 320x240 as recorded; the others were never recorded.
const Recipe kRecipes[] = {
    {"c320_qp", kVencH264, 320, 240, 3, kVencRcFixQp, 28, 0, 0, false},
    {"c176_cbr", kVencH264, 176, 144, 6, kVencRcCbr, 0, 100000, 3, false},
    {"c640_cbr", kVencH264, 640, 360, 8, kVencRcCbr, 0, 600000, 4, false},
    {"c1280_cbr", kVencH264, 1280, 720, 5, kVencRcCbr, 0, 2000000, 0, false},
    {"c1920_qp", kVencH264, 1920, 1080, 3, kVencRcFixQp, 30, 0, 0, false},
    {"c1366_cbr", kVencH264, 1366, 768, 4, kVencRcCbr, 0, 1500000, 0, false},
    // semi-planar sources: what VideoToolbox's pixel buffers are
    {"n640_cbr", kVencH264, 640, 360, 6, kVencRcCbr, 0, 800000, 5, true},
    {"n1366_cbr", kVencH264, 1366, 768, 4, kVencRcCbr, 0, 1500000, 0, true},
    {"n1920_qp", kVencH264, 1920, 1080, 2, kVencRcFixQp, 30, 0, 0, true},
    // the same for H.265 (the h-names): the parameter sets are a VPS, an SPS and a PPS, the pictures are coded in CTUs of 64x64
    {"h320_qp", kVencH265, 320, 240, 3, kVencRcFixQp, 28, 0, 0, false},
    {"h176_cbr", kVencH265, 176, 144, 6, kVencRcCbr, 0, 100000, 3, false},
    {"h640_cbr", kVencH265, 640, 360, 8, kVencRcCbr, 0, 600000, 4, false},
    {"h1366_cbr", kVencH265, 1366, 768, 4, kVencRcCbr, 0, 1500000, 0, false},
    {"hn640_cbr", kVencH265, 640, 360, 6, kVencRcCbr, 0, 800000, 5, true},
    {"hn1366_cbr", kVencH265, 1366, 768, 4, kVencRcCbr, 0, 1500000, 0, true},
    {"hn1920_qp", kVencH265, 1920, 1080, 2, kVencRcFixQp, 30, 0, 0, true},
};

// ---- what the block's run callbacks add up to (the time the block took, on top of the work of MPP's code)
uint64_t gBlockUs;

int runCounted(void *ctx, const uint32_t *regs, const VencRun *runs, unsigned nRuns, uint32_t *status) {
  const uint64_t start = nowUs();
  const int rc = rk3588_venc_core_run(ctx, regs, runs, nRuns, status);
  gBlockUs += nowUs() - start;
  return rc;
}

void dumpBytes(const char *name, unsigned picture, const uint8_t *p, uint32_t len) {
  static const char hex[] = "0123456789abcdef";
  for (uint32_t off = 0; off < len; off += 32) {
    char line[80];
    unsigned n = 0;
    for (uint32_t i = 0; i < 32 && off + i < len; ++i) {
      line[n++] = hex[p[off + i] >> 4];
      line[n++] = hex[p[off + i] & 15];
    }
    line[n] = 0;
    printf("rk3588-venc: core %s pic %u %06x %s\n", name, picture, off, line);
  }
}

bool runRecipe(const Recipe &recipe, bool dump) {
  uint8_t *window = nullptr;
  uint64_t bus = 0;
  uint32_t bytes = 0;
  rk3588_venc_core_window(&window, &bus, &bytes);
  if (!window) {
    printf("rk3588-venc: core %s: no DMA window\n", recipe.name);
    return false;
  }
  memset(window, 0, bytes);   // (the encoder's buffers are zero-filled, as in the recordings)

  VencHw hw = {};
  hw.dmaVirt = window;
  hw.dmaBus = bus;
  hw.dmaBytes = bytes;
  hw.run = runCounted;
  hw.readRegs = rk3588_venc_core_read_regs;

  VencParams params = {};
  params.codec = recipe.codec;
  params.width = recipe.width;
  params.height = recipe.height;
  params.format = recipe.nv12 ? kVencNv12 : kVencI420;
  if (recipe.nv12) {
    params.hstride = (recipe.width + 15) & ~15u;
    params.vstride = (recipe.height + 15) & ~15u;
  }
  params.rcMode = recipe.rcMode;
  params.qp = recipe.qp;
  params.bitrate = recipe.bitrate;
  params.fpsNum = 30;
  params.fpsDen = 1;
  params.gop = recipe.gop;
  params.motionInfo = 1;

  VencCore *core = nullptr;
  const uint64_t t0 = nowUs();
  const int rc = venc_core_create(&core, &hw, &params);
  if (rc) {
    printf("rk3588-venc: core %s: venc_core_create failed (%d)\n", recipe.name, rc);
    return false;
  }
  printf("rk3588-venc: core %s: %s %ux%u, %u pictures, %s, created in %llu us\n", recipe.name, recipe.codec == kVencH265 ? "H.265" : "H.264", recipe.width, recipe.height, recipe.frames,
         recipe.rcMode == kVencRcCbr ? "constant bit rate" : "fixed quantiser", static_cast<unsigned long long>(nowUs() - t0));

  uint8_t header[512];
  const int headerLength = venc_core_header(core, header, sizeof header);
  const int firstNal = recipe.codec == kVencH265 ? (header[4] >> 1) & 0x3f : header[4] & 0x1f;   // H.265: two header bytes, the type in bits 1 to 6 of the first
  if (headerLength < 8 || header[0] || header[1] || header[2] || header[3] != 1 || firstNal != (recipe.codec == kVencH265 ? 32 : 7)) {
    printf("rk3588-venc: core %s: no %s from venc_core_header (%d)\n", recipe.name, recipe.codec == kVencH265 ? "VPS" : "SPS", headerLength);
    venc_core_destroy(core);
    return false;
  }
  printf("rk3588-venc: core %s: parameter sets %d bytes\n", recipe.name, headerLength);
  if (dump) dumpBytes(recipe.name, 0xffff, header, static_cast<uint32_t>(headerLength));

  uint32_t sourceBytes = 0;
  uint8_t *source = venc_core_source(core, &sourceBytes);
  const uint32_t hstride = (recipe.width + 15) & ~15u, vstride = recipe.nv12 ? (recipe.height + 15) & ~15u : (recipe.height + 1) & ~1u;   // (planar: the rows the core's own defaults give)
  bool ok = source != nullptr;
  uint64_t totalBytes = 0;
  for (unsigned f = 0; f < recipe.frames && ok; ++f) {
    if (recipe.nv12) fillPictureNv12(source, hstride, vstride, recipe.width, recipe.height, f);
    else fillPicture(source, hstride, vstride, recipe.width, recipe.height, f);
    gBlockUs = 0;
    VencPacket packet;
    const uint64_t t = nowUs();
    const int er = venc_core_encode(core, f, 0, &packet);
    const uint64_t us = nowUs() - t;
    if (er || !packet.data || !packet.length) {
      printf("rk3588-venc: core %s pic %u: encode failed (%d)\n", recipe.name, f, er);
      ok = false;
      break;
    }
    printf("rk3588-venc: core %s pic %u: %u bytes (block's from +%u) idr %d qp %d, %llu us in all, %llu us on the block, crc %#x\n", recipe.name, f, packet.length,
           packet.hwOffset, packet.isIdr, packet.qp, static_cast<unsigned long long>(us), static_cast<unsigned long long>(gBlockUs),
           venc::crc32(packet.data, packet.length));
    if (dump) dumpBytes(recipe.name, f, packet.data, packet.length);
    totalBytes += packet.length;
  }
  VencStats stats = {};
  venc_core_stats(core, &stats);
  printf("rk3588-venc: core %s: %u jobs, %u job errors, %llu bytes, window peak %u bytes: %s\n", recipe.name, stats.jobs, stats.jobErrors,
         static_cast<unsigned long long>(totalBytes), stats.windowPeakBytes, ok && !stats.jobErrors ? "ok" : "FAILED");
  venc_core_destroy(core);
  return ok && !stats.jobErrors;
}

// MPP's modules take no second registration: once per boot, whichever of the self-test and the first stream comes first (they never overlap: the stream
// is opened by a client of the service, long after the self-test).
void registerOnce() {
  static bool registered;
  if (registered) return;
  registered = true;
  venc_core_register_singletons();
}

}  // namespace

extern "C" void rk3588_venc_core_selftest(void) {
  vec_log_level = static_cast<int>(bootArg("rk3588-venc-core-log", 3));
  const bool dump = bootArg("rk3588-venc-core-dump", 0) != 0;
  printf("rk3588-venc: core self-test: dump %u, MPP log level %d\n", dump ? 1u : 0u, vec_log_level);
  const uintptr_t top = paintStack();
  registerOnce();
  unsigned failed = 0, count = 0;
  for (const Recipe &recipe : kRecipes) {
    ++count;
    if (!runRecipe(recipe, dump)) ++failed;
  }
  printf("rk3588-venc: core self-test: %u of %u recipes ok; the core went %llu bytes deep into the stack (of the %llu painted)\n", count - failed, count,
         static_cast<unsigned long long>(stackDepth(top)), static_cast<unsigned long long>(kPaintBytes));
}

// ---- streams for the AppleAVE2 service
//
// The core runs on a thread of its own: the service's calls come on a thread of the application's (mediaserverd) that is already deep in the kernel (a system call,
// the Mach message, IOKit's dispatch, the service's own frames) and the core needs about 5 KiB more than a quarter of what is left. The thread takes one job at a time:
// a function and its argument, handed over under a lock; the caller sleeps until it is done. Callers wait for each other (one core, one block).
namespace {

struct CoreThread {
  IOLock *lock;       // the mailbox
  IOLock *callers;    // one caller at a time
  void (*fn)(void *);
  void *arg;
  bool pending, finished;
  uint8_t event[2];   // addresses to sleep on
  volatile int state; // 0 not started, 1 starting, 2 running, -1 failed
  uintptr_t top;      // the thread's own frame, for the stack measurement
} gThread;

void coreThreadMain(void *, wait_result_t) {
  gThread.top = paintStack();
  gThread.state = 2;
  for (;;) {
    IOLockLock(gThread.lock);
    while (!gThread.pending) IOLockSleep(gThread.lock, &gThread.event[0], THREAD_UNINT);
    void (*fn)(void *) = gThread.fn;
    void *arg = gThread.arg;
    IOLockUnlock(gThread.lock);
    fn(arg);
    IOLockLock(gThread.lock);
    gThread.pending = false;
    gThread.finished = true;
    IOLockWakeup(gThread.lock, &gThread.event[1], true);
    IOLockUnlock(gThread.lock);
  }
}

bool ensureThread() {
  if (gThread.state == 2) return true;
  if (__sync_bool_compare_and_swap(&gThread.state, 0, 1)) {
    gThread.lock = IOLockAlloc();
    gThread.callers = IOLockAlloc();
    thread_t thread = nullptr;
    if (!gThread.lock || !gThread.callers || kernel_thread_start(coreThreadMain, nullptr, &thread) != KERN_SUCCESS || !thread) {
      gThread.state = -1;
      return false;
    }
    thread_deallocate(thread);
  }
  for (unsigned i = 0; i < 2000 && gThread.state == 1; ++i) IOSleep(1);
  return gThread.state == 2;
}

bool onCoreThread(void (*fn)(void *), void *arg) {
  if (!ensureThread()) return false;
  IOLockLock(gThread.callers);
  IOLockLock(gThread.lock);
  gThread.fn = fn;
  gThread.arg = arg;
  gThread.finished = false;
  gThread.pending = true;
  IOLockWakeup(gThread.lock, &gThread.event[0], true);
  while (!gThread.finished) IOLockSleep(gThread.lock, &gThread.event[1], THREAD_UNINT);
  gThread.finished = false;
  IOLockUnlock(gThread.lock);
  IOLockUnlock(gThread.callers);
  return true;
}

}  // namespace

struct Rk3588VencCoreStream {
  VencCore *core;
  VencHw hw;
  Rk3588VencCoreSettings settings;
  uint32_t yStride, vStride;
  unsigned pictures, failures;
  uint64_t totalUs, blockUs, bytes;
  // the call that goes to the core's thread, and its answer
  int rc;
  int64_t pts;
  int forceIdr;
  VencPacket packet;
  uint64_t callUs, callBlockUs;
};

namespace {

void openOnCore(void *arg) {
  Rk3588VencCoreStream *s = static_cast<Rk3588VencCoreStream *>(arg);
  registerOnce();
  uint8_t *window = nullptr;
  uint64_t bus = 0;
  uint32_t bytes = 0;
  rk3588_venc_core_window(&window, &bus, &bytes);
  s->rc = -1;
  if (!window) return;
  memset(window, 0, bytes);   // (the encoder's buffers are zero-filled, as in the recordings)
  s->hw = {};
  s->hw.dmaVirt = window;
  s->hw.dmaBus = bus;
  s->hw.dmaBytes = bytes;
  s->hw.run = runCounted;
  s->hw.readRegs = rk3588_venc_core_read_regs;
  VencParams params = {};
  params.codec = s->settings.hevc ? kVencH265 : kVencH264;
  params.width = s->settings.width;
  params.height = s->settings.height;
  params.format = kVencNv12;
  params.hstride = s->yStride;
  params.vstride = s->vStride;
  params.rcMode = s->settings.bitrate ? kVencRcCbr : kVencRcFixQp;
  params.qp = s->settings.bitrate ? 0 : 28;
  params.bitrate = s->settings.bitrate;
  params.fpsNum = s->settings.fpsNum ? s->settings.fpsNum : 30;
  params.fpsDen = s->settings.fpsDen ? s->settings.fpsDen : 1;
  params.gop = s->settings.gop;
  params.motionInfo = 1;
  s->rc = venc_core_create(&s->core, &s->hw, &params);
}

void encodeOnCore(void *arg) {
  Rk3588VencCoreStream *s = static_cast<Rk3588VencCoreStream *>(arg);
  gBlockUs = 0;
  const uint64_t t = nowUs();
  s->rc = venc_core_encode(s->core, s->pts, s->forceIdr, &s->packet);
  s->callUs = nowUs() - t;
  s->callBlockUs = gBlockUs;
}

void closeOnCore(void *arg) {
  Rk3588VencCoreStream *s = static_cast<Rk3588VencCoreStream *>(arg);
  if (s->core) venc_core_destroy(s->core);
  s->core = nullptr;
}

}  // namespace

extern "C" struct Rk3588VencCoreStream *rk3588_venc_core_open(const struct Rk3588VencCoreSettings *settings, struct Rk3588VencCoreSource *source) {
  if (!settings || !source || !settings->width || !settings->height || (settings->width & 1) || (settings->height & 1) || settings->width > 4096 || settings->height > 4096)
    return nullptr;
  if (!rk3588_venc_window_claim()) return nullptr;
  Rk3588VencCoreStream *s = static_cast<Rk3588VencCoreStream *>(IOMallocZero(sizeof(Rk3588VencCoreStream)));
  if (!s) {
    rk3588_venc_window_release();
    return nullptr;
  }
  s->settings = *settings;
  s->yStride = (settings->width + 15) & ~15u;   // whole macroblocks: the block reads the pictures that way, and what is past the visible edge is the caller's to fill
  s->vStride = (settings->height + 15) & ~15u;
  vec_log_level = static_cast<int>(bootArg("rk3588-venc-core-log", 3));
  bool ok = onCoreThread(openOnCore, s) && s->rc == 0 && s->core;
  uint8_t *base = nullptr;
  uint32_t bytes = 0;
  if (ok) base = venc_core_source(s->core, &bytes);
  if (!base || bytes < s->yStride * s->vStride * 3 / 2) ok = false;
  if (!ok) {
    printf("rk3588-venc: core stream %ux%u: cannot be set up (%d)\n", settings->width, settings->height, s->rc);
    if (s->core) onCoreThread(closeOnCore, s);
    IOFree(s, sizeof(Rk3588VencCoreStream));
    rk3588_venc_window_release();
    return nullptr;
  }
  source->y = base;
  source->uv = base + static_cast<size_t>(s->yStride) * s->vStride;
  source->yStride = s->yStride;
  source->vStride = s->vStride;
  printf("rk3588-venc: core stream %s %ux%u: %u bit/s, %u/%u frames per second, %u pictures between IDR pictures (0: two seconds), source rows of %u bytes\n", settings->hevc ? "H.265" : "H.264",
         settings->width, settings->height, settings->bitrate, settings->fpsNum ? settings->fpsNum : 30, settings->fpsDen ? settings->fpsDen : 1, settings->gop, s->yStride);
  return s;
}

extern "C" int rk3588_venc_core_encode(struct Rk3588VencCoreStream *s, int64_t pts, int forceIdr, struct Rk3588VencCoreResult *result) {
  if (!s || !result || !s->core) return 0;
  *result = {};
  s->pts = pts;
  s->forceIdr = forceIdr;
  s->packet = {};
  if (!onCoreThread(encodeOnCore, s) || s->rc || !s->packet.data || !s->packet.length) {
    ++s->failures;
    return 0;
  }
  result->data = s->packet.data;
  result->length = s->packet.length;
  result->isIdr = s->packet.isIdr;
  result->qp = s->packet.qp;
  result->totalUs = static_cast<uint32_t>(s->callUs);
  result->blockUs = static_cast<uint32_t>(s->callBlockUs);
  ++s->pictures;
  s->totalUs += s->callUs;
  s->blockUs += s->callBlockUs;
  s->bytes += s->packet.length;
  return 1;
}

extern "C" void rk3588_venc_core_close(struct Rk3588VencCoreStream *s) {
  if (!s) return;
  VencStats stats = {};
  if (s->core) venc_core_stats(s->core, &stats);
  onCoreThread(closeOnCore, s);
  printf("rk3588-venc: core stream %ux%u closed: %u pictures (%u failed, %u jobs, %u job errors), %llu bytes, %llu us a picture, %llu us of it on the block; the core thread went %llu bytes deep into the stack\n",
         s->settings.width, s->settings.height, s->pictures, s->failures, stats.jobs, stats.jobErrors, static_cast<unsigned long long>(s->bytes),
         static_cast<unsigned long long>(s->pictures ? s->totalUs / s->pictures : 0), static_cast<unsigned long long>(s->pictures ? s->blockUs / s->pictures : 0),
         static_cast<unsigned long long>(stackDepth(gThread.top)));
  IOFree(s, sizeof(Rk3588VencCoreStream));
  rk3588_venc_window_release();
}
