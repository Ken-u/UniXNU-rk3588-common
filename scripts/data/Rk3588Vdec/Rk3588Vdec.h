// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Vdec: the RK3588's rkvdec2 video decoder (RKVDEC0: the block the vendor kernel drives as "rkv-decoder-v2" and Rockchip's MPP library as
// vdpu34x), shared by the boards.
//
// What is here, step by step (each step is a boot argument level so a board run shows how far the hardware gets):
//   * the power domain, clocks and resets of the block, written the way the vendor kernel does it (the PMU at 0xfd8d8000 and the CRU at 0xfd7c0000 are
//     SoC facts, so they are not in a board table);
//   * a self-test that needs no operating system above the kernel: a recorded decode job sequence (traces/*.vdt) of Rockchip's own user-space library for
//     an H.264 stream is replayed through the register file, the block decodes it, and the pictures are compared by CRC-32 with the output of a
//     bit-exact software decoder (VdecTrace.h, tools/);
//   * a decoder core (core/) that does the parsing itself: Rockchip's MPP H.264 parser and rkvdec2 register generator, compiled unmodified into the kernel
//     behind a shim, turn embedded H.264 streams into jobs for the block, and the pictures are compared by CRC-32 with the reference decoder's;
//   * (later) the kernel-side of a VideoToolbox decoder: hand the pictures to IOSurface through the user client of the iPadOS decoder service.
//
// What a board supplies: a DMA window the kernel does not use, below 4 GiB (the block is left in IOMMU bypass, so its bus addresses are physical), as
// the `rk3588_vdec_config` of a source file of its own that `build-rk3588-vdec.py` compiles next to these.
//
// The firmware (edk2-rk3588 / U-Boot) does not touch the decoder; the self-test finds out what state it leaves the power domain, clocks and resets in.
//
// Boot argument rk3588-vdec=N: 0 off (default); 1 look only (print the PMU and CRU state that concerns the decoder: power domains, PLL rates; nothing is written);
// 2 also power the domains on, set the clocks, release the resets and read the block's identification and IOMMU registers;
// 3 also replay every embedded trace once and compare the pictures; 4 as 3, repeated rk3588-vdec-passes=N times (default 20) for timing;
// 5 as 3, then every embedded H.264 stream through the decoder core, once; 6 only the decoder core, rk3588-vdec-passes=N times.
// 7 powers the block on (as 2) and registers the AppleD5500 service (Rk3588VdecD5500.cpp): VideoToolbox's H.264 decoder plug-in finds it, opens a user client and
// decodes through the decoder core; rk3588-vdec-d5500-log=N is how much of that traffic is printed (default 1).
// rk3588-vdec-dump=1 hex-dumps the output picture of the first job that does not match. rk3588-vdec-chunk=N feeds the streams to the decoder core in
// pieces of N bytes (default: whole); rk3588-vdec-mpp-log=N is how loud MPP's own messages are (1 fatal .. 6 verbose, default 3).
#pragma once
#include <stdint.h>

struct Rk3588VdecConfig {
  uint64_t dma_phys;    // a DMA window the kernel does not use, below 4 GiB, mapped as normal non-cacheable memory
  uint32_t dma_bytes;   // at least what the trace needs (the trace's windowBytes(), printed by mode 1)
};

#ifdef __cplusplus
extern "C" {
#endif

extern const struct Rk3588VdecConfig rk3588_vdec_config;

// Called by the board's probe once the kernel is up; reads the boot arguments and returns at once without rk3588-vdec=N.
void rk3588_vdec_start(void);

struct VdecHw;
// Fills in the block as the decoder core (core/VdecCore.h) wants it: the DMA window and the job runner. Non-zero when the driver has the block powered and
// mapped (rk3588-vdec=2 and up). Only one core may use it at a time.
int rk3588_vdec_hw(struct VdecHw *hw);

// What the user client's jobs have cost so far: how many, the time from start to interrupt on the block, the time inside the job runner (microseconds).
void rk3588_vdec_block_stats(uint64_t *jobs, uint64_t *runUs, uint64_t *wallUs);

// Rk3588VdecD5500.cpp: makes and registers the AppleD5500 service (once IOKit is up; called from the driver's thread).
void rk3588_vdec_serve(void);

#ifdef __cplusplus
}
#endif
