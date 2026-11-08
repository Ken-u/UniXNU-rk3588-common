// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VencCore.h -- a video encoder made of Rockchip's own MPP code: the H.264 syntax writers, the reference management, the rate control and the register
// generator of the RK3588's RKVENC0 block (MPP's "vepu580"), all unmodified, driven by a small loop of ours (core/VencCore.c) instead of MPP's threads and
// its Linux driver (core/EncShim.c). The caller hands over one picture at a time (planar or semi-planar 4:2:0, written into the DMA window); the encoded
// access unit comes back as bytes in the window, with the parameter sets in front of every IDR picture; every job the block has to run goes to the `run`
// callback.
//
// Plain C (the header is also read by the kernel driver, which is C++).
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VENC_REG_COUNT 4096   // the register file MPP writes, in words: control at 0, base at 160, rate control tables at 1024, ..., osd at 3072
#define VENC_MAX_RELOCS 32

typedef struct VencRun {   // a range of registers a job writes
  uint16_t first, count;
} VencRun;

// The block. `run` writes the job's registers (the ranges in `runs`, ascending; the start register, word 4, last), starts it and waits for it; the address
// registers in `regs` already hold bus addresses. Returns 0 when the block answered, with the interrupt status in *status. `readRegs` reads `words` words
// of the register file starting at byte offset `byteOffset` (the status class, from 0x4000: the length of the bit stream, the quantiser sum, ...) after
// a job.
typedef struct VencHw {
  void *ctx;
  uint8_t *dmaVirt;    // the DMA window: every buffer the block reads or writes lives in it
  uint64_t dmaBus;     // its address on the block's bus (below 4 GiB)
  uint32_t dmaBytes;
  int (*run)(void *ctx, const uint32_t *regs, const VencRun *runs, unsigned nRuns, uint32_t *status);
  void (*readRegs)(void *ctx, uint32_t byteOffset, uint32_t *dst, unsigned words);
} VencHw;

typedef struct VencReloc {
  uint32_t reg, buf, offset, bufSize;   // register `reg` = bus address of buffer `buf` (the shim's number) + offset
} VencReloc;

// A job as it goes to the block; handed to the observer (the host test compares it with a recording of MPP on a real board).
typedef struct VencJobInfo {
  const uint32_t *rawRegs;   // as the register generator wrote them (address registers: fd | offset << 10)
  const uint32_t *regs;      // as they go to the block
  const VencRun *runs;
  unsigned nRuns;
  VencReloc reloc[VENC_MAX_RELOCS];
  unsigned nReloc;
} VencJobInfo;

typedef struct VencJobObserver {
  void *ctx;
  void (*before)(void *ctx, const VencJobInfo *job);
  void (*after)(void *ctx, const VencJobInfo *job, int rc, uint32_t status);
} VencJobObserver;

enum { kVencH264 = 0, kVencH265 = 1 };
enum { kVencNv12 = 0, kVencI420 = 1 };
enum { kVencRcFixQp = 0, kVencRcCbr = 1, kVencRcVbr = 2 };   // MPP's MPP_ENC_RC_MODE_FIXQP / CBR / VBR are 2 / 1 / 0: the core maps them

typedef struct VencParams {
  int codec;                      // kVencH264 (H.265 is not in the core yet)
  uint32_t width, height;         // the visible picture
  uint32_t hstride, vstride;      // bytes per row of the luma plane of the source picture, rows per plane; 0: MPP's defaults (width rounded up to 16 for planar, 8 for semi-planar; height rounded up to 2)
  int format;                     // kVencNv12 or kVencI420
  int rcMode;                     // kVencRc*
  int qp;                         // fixed QP in kVencRcFixQp; the initial QP otherwise (0: let the rate control choose)
  int qpMin, qpMax;               // 0: 10 / 51
  uint32_t bitrate;               // bit/s target (0: width * height / 8 * fps, MPP's default); ignored in kVencRcFixQp
  uint32_t bitrateMin, bitrateMax;   // 0: MPP's defaults for the mode
  uint32_t fpsNum, fpsDen;        // 0: 30 / 1
  uint32_t gop;                   // frames between IDR pictures (0: two seconds)
  int motionInfo;                 // 1: the block also writes motion information per 64x64 block (MPP's test tool does; the recordings have it), into the window
  int profile, level, cabac;      // H.264: 100 / 40 / 1 are the defaults (a value of 0 means the default; cabac: 1 on, -1 off)
} VencParams;

typedef struct VencStats {
  uint32_t jobs, jobErrors, frames, windowPeakBytes;
} VencStats;

typedef struct VencPacket {
  const uint8_t *data;   // Annex B: parameter sets (IDR pictures), then the slices; in the DMA window, valid until the next call
  uint32_t length;
  int64_t pts;
  int isIdr;             // an IDR picture (the parameter sets are in front)
  int isIntra;
  int qp;                // the average QP the block reported
  uint32_t hwOffset;     // where the block's own bytes start in `data` (what is in front is parameter sets written by the CPU)
} VencPacket;

typedef struct VencCore VencCore;

// Opens an encoder on `hw`. One block, one window: only one core may be open at a time. Returns 0 on success.
int venc_core_create(VencCore **out, const VencHw *hw, const VencParams *params);
void venc_core_destroy(VencCore *core);
void venc_core_set_observer(VencCore *core, VencJobObserver *observer);

// MPP's named settings (the fields of MppEncCfg: "rc:bps_target", "h264:profile", ...), taken up at the next picture. 0 on success.
int venc_core_set_s32(VencCore *core, const char *name, int32_t value);

// The source picture: where the caller writes it (hstride x vstride luma, then the chroma planes) and how many bytes that is.
uint8_t *venc_core_source(VencCore *core, uint32_t *bytes);

// The parameter sets (SPS and PPS, Annex B) of the current settings, for a caller that keeps them apart from the pictures (an avcC record): copied to `out`
// (`capacity` bytes), returns their length or a negative value. After this call the next picture does not start with them again (MPP's rule: the headers
// are given out once); IDR pictures from then on do carry them (header mode "each IDR").
int venc_core_header(VencCore *core, uint8_t *out, uint32_t capacity);

// Encodes the picture now in the source buffer. `forceIdr` makes it an IDR picture. Returns 0 and fills *out, or a negative value (the encoder is left usable:
// the next picture is an IDR picture).
int venc_core_encode(VencCore *core, int64_t pts, int forceIdr, VencPacket *out);

void venc_core_stats(const VencCore *core, VencStats *stats);

#ifdef __cplusplus
}
#endif
