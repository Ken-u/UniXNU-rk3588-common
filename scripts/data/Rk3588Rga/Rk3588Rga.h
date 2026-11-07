// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Rga: the RK3588's RGA3 2D engine (core 0) as a picture copier for the video path, shared by the boards.
//
// What it is for: the decoder (Rk3588Vdec) leaves a picture in its DMA window; the user client of the AppleD5500 service has to put it into an IOSurface
// whose pages are scattered all over memory, above 4 GiB. The CPU does that copy through uncached memory (about 1.1 ms for a 720p picture). RGA3 can do it
// on its own, and unlike the older RGA2 (a flat 32-bit page table) RGA3's IOMMU (Rockchip IOMMU v2, 40-bit) reaches every page of an IOSurface. So the
// driver owns a page table (RkIommu.h) in a small window of memory, maps the decoder's window and each IOSurface into the block's 32-bit IO address space,
// and runs copy jobs in slave mode (Rga3.h).
//
// What a board supplies: a window the kernel does not use, below 4 GiB (page tables, a scratch page, the self-test's pictures), as the `rk3588_rga_config` of
// a source file of its own that `build-rk3588-rga.py` compiles next to these.
//
// Boot argument rk3588-rga=N: 0 off (default); 1 look only (power domains, clocks, resets; nothing is written); 2 also power the block on, set its clock,
// release its resets and read its version and IOMMU registers; 3 also copy test pictures with the IOMMU off (physical addresses, inside the window);
// 4 bring the IOMMU up and copy test pictures into scattered kernel memory, rk3588-rga-passes=N times (default 10); 5 bring it up and leave it ready for
// clients (rk3588_rga_ready); 6 as 4 with one pass, then ready. rk3588-rga-mhz=N is the block's clock (default 750: cpll / 2, the setting found at boot; the nearest rate not above N from gpll or cpll), rk3588-rga-fault=1 adds a job with an
// unmapped destination to mode 4 (the error path: a page fault, a reset, the next job must work).
//
// The firmware (edk2-rk3588 / U-Boot) does not touch the block.
#pragma once
#include <stdint.h>

struct Rk3588RgaConfig {
  uint64_t dma_phys;    // a window the kernel does not use, below 4 GiB, mapped as normal non-cacheable memory
  uint32_t dma_bytes;   // at least 8 MiB: 512 KiB of page tables, a scratch page, then the self-test pictures from 1 MiB on
};

#ifdef __cplusplus
extern "C" {
#endif

extern const struct Rk3588RgaConfig rk3588_rga_config;

// Called by the board's probe once the kernel is up; reads the boot arguments and returns at once without rk3588-rga=N.
void rk3588_rga_start(void);

// Non-zero once the block is powered, its IOMMU translates and a client may map memory and copy (modes 5 and 6; also while mode 4's test runs: no).
int rk3588_rga_ready(void);

// A picture in NV12: the IO virtual address of the first byte of each plane (the chroma plane is half as many rows with the same pitch) and the pitch.
struct Rk3588RgaPicture {
  uint32_t y, uv, stride;
};

// Maps physical memory (page aligned, `bytes` rounded up to pages) into the block's address space, read-only or writable; the IO address of its first byte,
// or 0 when there is no room or the block is not ready. Four guard pages behind the range map a scratch page, so a block that runs over the end does no harm.
uint32_t rk3588_rga_map_physical(uint64_t phys, uint64_t bytes, int writable);
// Unmaps what the calls above returned (the same `bytes`).
void rk3588_rga_unmap(uint32_t iova, uint64_t bytes);

// Copies `width` x `height` pixels of an NV12 picture, starting at (srcX, srcRow) in the source, to the start of the destination picture. 0 when done;
// -1 when this copy is not for the block (a limit of RGA3: size, alignment, pitch; the caller does it itself); -2 when the block failed (it has been reset;
// after a few of these in a row the driver stops being ready). `*us` receives the time the block ran, when given.
int rk3588_rga_copy_nv12(const struct Rk3588RgaPicture *src, uint32_t srcX, uint32_t srcRow, const struct Rk3588RgaPicture *dst, uint32_t width, uint32_t height,
                         uint32_t *us);

// What the block has done: jobs finished, microseconds it ran (start to end), jobs that failed.
void rk3588_rga_stats(uint64_t *jobs, uint64_t *runUs, uint64_t *failed);

#ifdef __cplusplus
}
class IOMemoryDescriptor;
// Maps a prepared memory descriptor (an IOSurface's, say): its pages go into the block's address space whatever their physical addresses are. The first
// physical segment must be page aligned. The IO address of byte 0 or 0 on failure; unmapped with rk3588_rga_unmap(iova, length).
uint32_t rk3588_rga_map_memory(IOMemoryDescriptor *memory, uint64_t length);
#endif
