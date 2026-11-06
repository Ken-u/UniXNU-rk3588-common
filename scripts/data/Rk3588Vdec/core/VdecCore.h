// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VdecCore.h -- a video decoder made of Rockchip's own MPP code: the parser of the codec (H.264 so far) and the register generator of the rkvdec2 block,
// both unmodified, driven by a small loop of ours (core/VdecCore.c) instead of MPP's threads and its Linux driver (core/MppShim.c). The caller hands
// over stream bytes; pictures (NV12, in the DMA window) come back through a callback; every job the block has to run goes to the `run` callback.
//
// Plain C (the header is also read by the kernel driver, which is C++).
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VDEC_REG_COUNT 360   // RKVDEC_REG_NUM
#define VDEC_MAX_RELOCS 64

typedef struct VdecRun {   // a range of registers a job writes
  uint16_t first, count;
} VdecRun;

// The block. `run` writes the job's registers (the ranges in `runs`, ascending; the start register 10 last), starts it and waits for it; the
// address registers in `regs` already hold bus addresses. Returns 0 when the block answered, with the interrupt status in *status.
typedef struct VdecHw {
  void *ctx;
  uint8_t *dmaVirt;    // the DMA window: every buffer the block reads or writes lives in it
  uint64_t dmaBus;     // its address on the block's bus (below 4 GiB)
  uint32_t dmaBytes;
  int (*run)(void *ctx, const uint32_t *regs, const VdecRun *runs, unsigned nRuns, uint32_t *status);
} VdecHw;

typedef struct VdecReloc {
  uint32_t reg, buf, offset, bufSize;   // register `reg` = bus address of buffer `buf` (the shim's number) + offset
} VdecReloc;

// A job as it goes to the block; handed to the observer (the host test compares it with a recording of MPP on a real board).
typedef struct VdecJobInfo {
  const uint32_t *rawRegs;   // as the register generator wrote them (address registers: fd | offset << 10)
  const uint32_t *regs;      // as they go to the block
  const VdecRun *runs;
  unsigned nRuns;
  VdecReloc reloc[VDEC_MAX_RELOCS];
  unsigned nReloc;
} VdecJobInfo;

typedef struct VdecJobObserver {
  void *ctx;
  void (*before)(void *ctx, const VdecJobInfo *job);
  void (*after)(void *ctx, const VdecJobInfo *job, int rc, uint32_t status);
} VdecJobObserver;

enum {
  kVdecPicEos = 1,        // last picture of the stream
  kVdecPicError = 2,      // decoded from damaged or missing references
  kVdecPicDiscard = 4,    // should not be shown
};

typedef struct VdecPicture {
  const uint8_t *data;        // NV12 in the DMA window: the luma plane, then the interleaved chroma at data + hstride * vstride
  uint32_t bytes;             // size of the buffer
  uint32_t width, height;     // visible size
  uint32_t hstride, vstride;  // bytes per row of a plane, rows per plane
  uint32_t cropLeft, cropTop; // where the visible part starts
  int64_t pts;
  uint32_t flags;             // kVdecPic*
  uint32_t poc;
} VdecPicture;

typedef struct VdecSink {
  void *ctx;
  void (*picture)(void *ctx, const VdecPicture *pic);   // the pixels are only valid during the call
} VdecSink;

typedef struct VdecStats {
  uint32_t jobs, jobErrors, pictures, windowPeakBytes;
} VdecStats;

typedef struct VdecCore VdecCore;

enum { kVdecH264 = 0 };

// Opens a decoder on `hw`. One block, one window: only one core may be open at a time.
int vdec_core_create(VdecCore **out, const VdecHw *hw, int codec);
void vdec_core_destroy(VdecCore *core);
void vdec_core_set_observer(VdecCore *core, VdecJobObserver *observer);

// Feeds stream bytes (Annex B byte stream; any split) with the presentation time of the first one; every picture that is complete comes out through
// the sink before the call returns. `eos` flushes the decoder: the pictures still held back for reordering come out. Returns the number of pictures
// delivered, or a negative value when the decoder cannot go on (reset it).
int vdec_core_decode(VdecCore *core, const uint8_t *data, size_t size, int64_t pts, int eos, const VdecSink *sink);
void vdec_core_reset(VdecCore *core);
void vdec_core_stats(const VdecCore *core, VdecStats *stats);

#ifdef __cplusplus
}
#endif
