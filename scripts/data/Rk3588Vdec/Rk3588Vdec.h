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
//   * (later) the kernel-side of a VideoToolbox decoder: parse the H.264 / HEVC headers, build the register image, hand the pictures to IOSurface.
//
// What a board supplies: a DMA window the kernel does not use, below 4 GiB (the block is left in IOMMU bypass, so its bus addresses are physical), as
// the `rk3588_vdec_config` of a source file of its own that `build-rk3588-vdec.py` compiles next to these.
//
// The firmware (edk2-rk3588 / U-Boot) does not touch the decoder; the self-test finds out what state it leaves the power domain, clocks and resets in.
//
// Boot argument rk3588-vdec=N: 0 off (default); 1 look only (print the PMU and CRU state that concerns the decoder: power domains, PLL rates; nothing is written);
// 2 also power the domains on, set the clocks, release the resets and read the block's identification and IOMMU registers;
// 3 also replay every embedded trace once and compare the pictures; 4 as 3, repeated rk3588-vdec-passes=N times (default 20) for timing.
// rk3588-vdec-dump=1 hex-dumps the output picture of the first job that does not match.
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

#ifdef __cplusplus
}
#endif
