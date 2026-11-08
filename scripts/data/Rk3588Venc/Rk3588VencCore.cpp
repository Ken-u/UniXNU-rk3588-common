// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The encoder core in the kernel (core/VencCore.h: Rockchip's MPP H.264 encoder over the shim of core/EncShim.c): what it asks of its embedder, and the board
// self-test of boot argument rk3588-venc=6.
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
constexpr uintptr_t kPaintBytes = 12 * 1024, kPaintGap = 1024;
constexpr uint8_t kPaint = 0xa5;
uintptr_t gStackTop;

__attribute__((noinline)) void paintStack() {
  volatile uint8_t *p = reinterpret_cast<volatile uint8_t *>(gStackTop - kPaintGap);
  for (uintptr_t i = 0; i < kPaintBytes; ++i) p[-static_cast<intptr_t>(i)] = kPaint;
}

// How many bytes below the top the deepest write reached (the paint's extent, plus the gap, when it went all the way).
uintptr_t stackDepth() {
  const volatile uint8_t *low = reinterpret_cast<volatile uint8_t *>(gStackTop - kPaintGap - kPaintBytes + 1);
  for (uintptr_t i = 0; i < kPaintBytes; ++i)
    if (low[i] != kPaint) return gStackTop - reinterpret_cast<uintptr_t>(low + i);
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

struct Recipe {
  const char *name;
  uint32_t width, height;
  unsigned frames;
  int rcMode;
  int qp;
  uint32_t bitrate;   // bit/s
  uint32_t gop;
};
// 320x240 as recorded; the others were never recorded.
const Recipe kRecipes[] = {
    {"c320_qp", 320, 240, 3, kVencRcFixQp, 28, 0, 0},
    {"c176_cbr", 176, 144, 6, kVencRcCbr, 0, 100000, 3},
    {"c640_cbr", 640, 360, 8, kVencRcCbr, 0, 600000, 4},
    {"c1280_cbr", 1280, 720, 5, kVencRcCbr, 0, 2000000, 0},
    {"c1920_qp", 1920, 1080, 3, kVencRcFixQp, 30, 0, 0},
    {"c1366_cbr", 1366, 768, 4, kVencRcCbr, 0, 1500000, 0},
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
  params.codec = kVencH264;
  params.width = recipe.width;
  params.height = recipe.height;
  params.format = kVencI420;
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
  printf("rk3588-venc: core %s: %ux%u, %u pictures, %s, created in %llu us\n", recipe.name, recipe.width, recipe.height, recipe.frames,
         recipe.rcMode == kVencRcCbr ? "constant bit rate" : "fixed quantiser", static_cast<unsigned long long>(nowUs() - t0));

  uint8_t header[512];
  const int headerLength = venc_core_header(core, header, sizeof header);
  if (headerLength < 8 || header[0] || header[1] || header[2] || header[3] != 1 || (header[4] & 0x1f) != 7) {
    printf("rk3588-venc: core %s: no SPS from venc_core_header (%d)\n", recipe.name, headerLength);
    venc_core_destroy(core);
    return false;
  }
  printf("rk3588-venc: core %s: parameter sets %d bytes\n", recipe.name, headerLength);
  if (dump) dumpBytes(recipe.name, 0xffff, header, static_cast<uint32_t>(headerLength));

  uint32_t sourceBytes = 0;
  uint8_t *source = venc_core_source(core, &sourceBytes);
  const uint32_t hstride = (recipe.width + 15) & ~15u, vstride = (recipe.height + 1) & ~1u;
  bool ok = source != nullptr;
  uint64_t totalBytes = 0;
  for (unsigned f = 0; f < recipe.frames && ok; ++f) {
    fillPicture(source, hstride, vstride, recipe.width, recipe.height, f);
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

}  // namespace

extern "C" void rk3588_venc_core_selftest(void) {
  gStackTop = reinterpret_cast<uintptr_t>(__builtin_frame_address(0));
  vec_log_level = static_cast<int>(bootArg("rk3588-venc-core-log", 3));
  const bool dump = bootArg("rk3588-venc-core-dump", 0) != 0;
  {
    // what the kernel's boot-arg parser says about names of this kind (a name that is never found tells more than a missing line)
    uint32_t v = 0;
    const int a = PE_parse_boot_argn("rk3588-venc-core-dump", &v, sizeof v), b = PE_parse_boot_argn("rk3588-venc-core-log", &v, sizeof v);
    uint32_t w = 0;
    const int c = PE_parse_boot_argn("wdt-kick", &w, sizeof w);
    printf("rk3588-venc: core boot-args: core-dump %s (%u), core-log %s, wdt-kick %s (%u)\n", a ? "found" : "absent", dump ? 1u : 0u, b ? "found" : "absent",
           c ? "found" : "absent", w);
  }
  paintStack();
  static bool registered;   // once per boot: MPP's modules take no second registration
  if (!registered) {
    registered = true;
    venc_core_register_singletons();
  }
  unsigned failed = 0, count = 0;
  for (const Recipe &recipe : kRecipes) {
    ++count;
    if (!runRecipe(recipe, dump)) ++failed;
  }
  printf("rk3588-venc: core self-test: %u of %u recipes ok; the core went %llu bytes deep into the stack (of the %llu painted)\n", count - failed, count,
         static_cast<unsigned long long>(stackDepth()), static_cast<unsigned long long>(kPaintBytes + kPaintGap));
}
