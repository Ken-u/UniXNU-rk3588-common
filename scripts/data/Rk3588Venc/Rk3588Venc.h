// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Venc: the RK3588's video encoder (RKVENC0: the block the vendor kernel drives as "rkv-encoder-v2" and Rockchip's MPP library as vepu580), shared by
// the boards.
//
// What is here, step by step (each step is a boot argument level so a board run shows how far the hardware gets):
//   * the power domain, clocks and resets of the block, written the way the vendor kernel does it (the PMU at 0xfd8d8000 and the CRU at 0xfd7c0000 are
//     SoC facts, so they are not in a board table);
//   * (later) a self-test that replays recorded encode jobs of Rockchip's user-space library and checks the bit stream the block writes by decoding it;
//   * (later) the kernel side of a VideoToolbox encoder: the AppleAVE2 service and its user client over an encoder core made of MPP's parts.
//
// What a board supplies: a DMA window the kernel does not use, below 4 GiB (the block is left in IOMMU bypass, so its bus addresses are physical), as
// the `rk3588_venc_config` of a source file of its own that `build-rk3588-venc.py` compiles next to these.
//
// The firmware (edk2-rk3588 / U-Boot) does not touch the encoder; the driver finds out what state it leaves the power domain, clocks and resets in.
//
// Boot argument rk3588-venc=N: 0 off (default); 1 look only (print the PMU and CRU state that concerns the encoder: power domains, gates, resets, PLL rates;
// nothing is written); 2 also power the domain on, set the clocks, release the resets, read the block's identification and status registers and do the
// vendor driver's safe reset once; 3 also map the DMA window and replay the encode-job traces built in (VencTrace.h) once, printing for every job the status, the
// time the block took, and the bit stream it wrote (tools/check-enc-output.py decodes that on the host); 4 replays them rk3588-venc-passes=N times (default 20),
// comparing every pass's bit stream with the first's (add rk3588-venc-dump=1 to print the stream of the first pass too); 5 only powers, clocks and maps the block
// for the AppleAVE2 service (rk3588-venc-ave2-encode=1 does that without this argument); 6 runs the self-test of the encoder core (core/, Rk3588VencCore.cpp: MPP's
// own H.264 encoder over pictures made here, sizes and rates that were never recorded), printing a line per picture and, with rk3588-venc-core-dump=1, its bytes
// for tools/check-core-output.py; rk3588-venc-core-log=N lets MPP's own messages through up to level N (default 3). After the self-test the block stays powered and
// the window mapped for the AppleAVE2 service (rk3588-venc-ave2-encode=2: the core encodes the application's pictures; =1: the traces' template encoder).
#pragma once
#include <stdint.h>

struct Rk3588VencConfig {
  uint64_t dma_phys;    // a DMA window the kernel does not use, below 4 GiB, mapped as normal non-cacheable memory
  uint32_t dma_bytes;
};

#ifdef __cplusplus
extern "C" {
#endif

extern const struct Rk3588VencConfig rk3588_venc_config;

// Called by the board's probe once the kernel is up; reads the boot arguments and returns at once without rk3588-venc=N.
void rk3588_venc_start(void);

// ---- the trace-template encoder behind the AppleAVE2 service (boot argument rk3588-venc-ave2-encode=1 next to rk3588-venc-ave2=1; Rk3588Venc.cpp)
//
// The replay in Rk3588Venc.cpp repeats recorded encode jobs; with the source picture replaced it encodes other pictures: a built-in trace of the size and
// codec asked for is played over and over (its jobs in order, from the first again after the last: for the traces recorded with a closed GOP that means an
// IDR picture every few frames), the planar 4:2:0 source picture of every job being whatever the caller put into `picture`. The settings are those of the
// recording (quantiser, GOP, the parameter sets); nothing is rate controlled. One stream at a time uses the block; the DMA window is shared.
struct Rk3588VencPicture {
  uint8_t *y, *u, *v;                    // the three planes in the DMA window (write-combined: write them, do not read them back)
  uint32_t width, height, yStride, uStride;
};
struct Rk3588VencResult {
  const uint8_t *stream;                 // the bit stream of the job as the CPU sees it: what the CPU put in front of the block's start, then the block's bytes
  uint32_t length;
  const uint8_t *headers;                // the parameter sets of the trace (Annex B), 0 when the trace has none
  uint32_t headerLength;
  uint32_t job;                          // index of the job in the trace
  uint32_t runUs;                        // time the block took
  uint32_t hwOffset;                     // where in the stream the block's own bytes begin: what is in front of it was written by the CPU for the recording and is stale when replayed
};
struct Rk3588VencStream;

// 1 when the block is powered, reset and its window mapped (the boot arguments rk3588-venc-ave2=2 or rk3588-venc=N did that).
int rk3588_venc_ready(void);
// Picks the trace (width x height as the encoder's pictures, hevc 0 / 1); 0 when the block is not ready, a stream is open already or no trace fits. *name is the trace's.
struct Rk3588VencStream *rk3588_venc_stream_open(uint32_t width, uint32_t height, int hevc, struct Rk3588VencPicture *picture, const char **name);
void rk3588_venc_stream_close(struct Rk3588VencStream *stream);
// Encodes the picture now in the DMA window as the next job of the trace; 1 when the block finished it without an error status.
int rk3588_venc_stream_encode(struct Rk3588VencStream *stream, struct Rk3588VencResult *result);

// ---- the block for the encoder core (core/VencCore.h): what VencHw's callbacks are on the real hardware. rk3588_venc_core_window gives the DMA window mapped by the
// self-test or the service (null until the block is ready).
struct VencRun;
int rk3588_venc_core_run(void *ctx, const uint32_t *regs, const struct VencRun *runs, unsigned nRuns, uint32_t *status);
void rk3588_venc_core_read_regs(void *ctx, uint32_t byteOffset, uint32_t *dst, unsigned words);
void rk3588_venc_core_window(uint8_t **virt, uint64_t *bus, uint32_t *bytes);
// The self-test of the encoder core (boot argument rk3588-venc=6), run by the driver thread once the block is up.
void rk3588_venc_core_selftest(void);

// ---- the encoder core behind the AppleAVE2 service (boot argument rk3588-venc-ave2-encode=2; Rk3588VencCore.cpp)
//
// H.264 or H.265 pictures of any size, at the bit rate and with the key-frame interval the application asked for: MPP's encoder (core/) instead of a recording's trace.
// One stream at a time, and it shares the block and the DMA window with the template encoder above (rk3588_venc_window_claim). The core runs on a thread of
// its own (16 KiB of kernel stack, almost all of it the core's: it goes about 5 KiB deep), the calls below wait for it.
struct Rk3588VencCoreSettings {
  uint32_t width, height;     // the visible picture; even
  uint32_t bitrate;           // bit/s, constant bit rate; 0: a fixed quantiser (QP 28)
  uint32_t fpsNum, fpsDen;    // 0: 30 / 1
  uint32_t gop;               // pictures between IDR pictures; 0: two seconds' worth
  uint32_t hevc;              // 0: H.264, 1: H.265
};
struct Rk3588VencCoreSource {
  uint8_t *y;                 // the NV12 source picture, in the DMA window (write-combined: write it, do not read it back): `yStride` bytes per row, `vStride` rows of luma, then
  uint8_t *uv;                // the interleaved chroma at `uv`, `yStride` bytes per row too
  uint32_t yStride, vStride;
};
struct Rk3588VencCoreResult {
  const uint8_t *data;        // Annex B, valid until the next encode call: the parameter sets (SPS, PPS; H.265: VPS, SPS, PPS) in front of an IDR picture, then the slice(s)
  uint32_t length;
  int isIdr;
  int qp;                     // what the block reported
  uint32_t totalUs;           // the whole call, and the part of it the block took
  uint32_t blockUs;
};
struct Rk3588VencCoreStream;

// 1 when the block and the window are free and the core could be set up; the block is claimed from here to rk3588_venc_core_close.
int rk3588_venc_window_claim(void);
void rk3588_venc_window_release(void);
struct Rk3588VencCoreStream *rk3588_venc_core_open(const struct Rk3588VencCoreSettings *settings, struct Rk3588VencCoreSource *source);
void rk3588_venc_core_close(struct Rk3588VencCoreStream *stream);
// Encodes the picture now in the source buffer; 1 on success. `forceIdr`: make it an IDR picture.
int rk3588_venc_core_encode(struct Rk3588VencCoreStream *stream, int64_t pts, int forceIdr, struct Rk3588VencCoreResult *result);

// Registers the AppleAVE2Driver service (the logging stage, Rk3588VencAve2.cpp) when the boot argument rk3588-venc-ave2=N is set; called by rk3588_venc_start().
void rk3588_venc_ave2_serve(void);

#ifdef __cplusplus
}
#endif
