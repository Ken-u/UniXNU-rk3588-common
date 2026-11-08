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
// comparing every pass's bit stream with the first's (add rk3588-venc-dump=1 to print the stream of the first pass too).
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

// Registers the AppleAVE2Driver service (the logging stage, Rk3588VencAve2.cpp) when the boot argument rk3588-venc-ave2=N is set; called by rk3588_venc_start().
void rk3588_venc_ave2_serve(void);

#ifdef __cplusplus
}
#endif
