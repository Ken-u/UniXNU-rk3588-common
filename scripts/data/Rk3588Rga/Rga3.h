// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rga3: the RK3588's RGA3 2D engine (two cores; core 0 at 0xfdb60000), driven the way the vendor kernel drives it in "slave mode" (the CPU writes the 48
// command registers itself; the IOMMU in front of the core cannot reach a command buffer in master mode). Only what the video path needs is here: a copy of
// an 8-bit semi-planar 4:2:0 picture (NV12) from one buffer to another, with a crop of the source and different row pitches. Everything else the block can
// do (scaling, rotation, colour conversion, blending, other formats, tiled and compressed layouts) is left out.
//
// No kernel types: the register file is reached through an accessor, so the host test (tests/) runs this code against a model of the block.
//
// Facts from the vendor kernel's open driver (drivers/video/rockchip/rga3/rga3_reg_info.c, rga_hw_config.c); nothing is copied. The values of the command
// registers below are what that driver writes for a copy without scaling or blending.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace rga3 {

// System registers (offsets from the block's base).
enum : uint32_t {
  kSysCtrl = 0x00, kCmdCtrl = 0x04, kCmdAddr = 0x08, kVersion = 0x18, kVersionTime = 0x1c, kIntEn = 0x20, kIntRaw = 0x24, kIntMask = 0x28, kIntClear = 0x2c,
  kRoSrst = 0x30, kStatus0 = 0x34, kCmdState = 0x40,
  kCmdBase = 0x100,       // the 48 command registers of a job
  kCmdWords = 48,
  kIommuBase = 0xf00,     // the IOMMU (rkiommu::) sits in the same 4 KiB page
};
enum : uint32_t {
  kSysStart = 1u << 0, kSysCmdMode = 1u << 1, kSysLgcClkOn = 1u << 2, kSysAclkReset = 1u << 3, kSysCclkReset = 1u << 4, kSysFrmendAutoReset = 1u << 11,
};
enum : uint32_t { kResetDone = 0x3f };   // RO_SRST
enum : uint32_t {
  kIntFrameDone = 1u << 0, kIntMmu = 1u << 1, kIntCmdLineFinish = 1u << 3,
  // every bit the vendor driver treats as an error: MMU, read bus, window format/size/FIFO errors, write size/bus/FIFO errors
  kIntErrors = 0x3f3fcee6u,
};

// Offsets inside the command register block (add kCmdBase for the register).
enum : uint32_t {
  kWin0RdCtrl = 0x00, kWin0YBase = 0x10, kWin0UBase = 0x14, kWin0VBase = 0x18, kWin0VirStride = 0x1c, kWin0SrcSize = 0x24, kWin0ActOff = 0x28, kWin0ActSize = 0x2c,
  kWin0DstSize = 0x30, kWin0SclFac = 0x34, kWin0UvVirStride = 0x38,
  kWin1RdCtrl = 0x40,
  kOvlpCtrl = 0x80, kOvlpOff = 0x84, kOvlpTopCtrl = 0x90, kOvlpBotCtrl = 0x94, kOvlpTopAlpha = 0x98, kOvlpBotAlpha = 0x9c,
  kWrCtrl = 0xa0, kWrFbceCtrl = 0xa4, kWrVirStride = 0xa8, kWrPlVirStride = 0xac, kWrYBase = 0xb0, kWrUBase = 0xb4, kWrVBase = 0xb8,
};

// The limits of RGA3 (rga_hw_config.c): input 68x2 .. 8176x8176 (with the crop offset), output 68x2 .. 8128x8128, row pitch a multiple of 16 bytes up to 32768.
constexpr uint32_t kMinWidth = 68, kMinHeight = 2, kMaxInput = 8176, kMaxOutput = 8128, kMaxStride = 32768, kStrideAlign = 16;

// One copy: NV12 pictures, each a luma plane and an interleaved chroma plane of half the rows with the same row pitch. Addresses are what the block puts
// on the bus (IO virtual addresses when the IOMMU translates, physical ones below 4 GiB when it does not).
struct Copy {
  uint32_t srcY, srcUv, srcStride;     // the source planes and their row pitch in bytes
  uint32_t srcX, srcRow;               // where the visible part starts (pixels, rows): even
  uint32_t dstY, dstUv, dstStride;
  uint32_t width, height;              // what is copied: even
};

// Why a copy cannot be done by the block (null when it can); the caller falls back to the CPU.
inline const char *check(const Copy &c) {
  if (!c.srcY || !c.srcUv || !c.dstY || !c.dstUv) return "an address is zero";
  if ((c.srcY | c.srcUv | c.dstY | c.dstUv) & 15) return "a plane address is not 16-byte aligned";
  if ((c.srcStride | c.dstStride) & (kStrideAlign - 1)) return "a row pitch is not a multiple of 16";
  if (!c.srcStride || !c.dstStride || c.srcStride > kMaxStride || c.dstStride > kMaxStride) return "a row pitch is out of range";
  if ((c.srcX | c.srcRow | c.width | c.height) & 1) return "an odd offset or size";
  if (c.width < kMinWidth || c.height < kMinHeight) return "smaller than the block's minimum";
  if (c.width > kMaxOutput || c.height > kMaxOutput) return "larger than the block's output limit";
  if (c.srcX + c.width > kMaxInput || c.srcRow + c.height > kMaxInput) return "larger than the block's input limit";
  if (c.srcX + c.width > c.srcStride || c.width > c.dstStride) return "a row does not fit in its pitch";
  return nullptr;
}

constexpr uint32_t align16(uint32_t v) { return (v + 15u) & ~15u; }
constexpr uint32_t strideWords(uint32_t bytes) { return align16(bytes) >> 2; }

// The 48 command words of a copy (check() must have passed).
inline void build(const Copy &c, uint32_t cmd[kCmdWords]) {
  memset(cmd, 0, kCmdWords * sizeof(uint32_t));
  auto reg = [&](uint32_t off) -> uint32_t & { return cmd[off / 4]; };
  // Window 0 reads the source. NV12 is picture format 0 in semi-planar layout (RD_FORMAT 1), 8 bit, no scaling (HOR_BY, VER_BY), no colour conversion,
  // raster mode, enabled; the 10-bit "compact" bit is set as the vendor driver sets it for 8-bit formats too.
  reg(kWin0RdCtrl) = 1u | (0u << 4) | (1u << 8) | (1u << 10) | (1u << 20) | (1u << 22);
  reg(kWin0YBase) = c.srcY;
  reg(kWin0UBase) = c.srcUv;
  reg(kWin0VirStride) = strideWords(c.srcStride);
  reg(kWin0UvVirStride) = strideWords(c.srcStride);
  reg(kWin0ActOff) = c.srcX | (c.srcRow << 16);
  reg(kWin0SrcSize) = align16(c.width + c.srcX) | (align16(c.height + c.srcRow) << 16);
  reg(kWin0ActSize) = c.width | (c.height << 16);
  reg(kWin0DstSize) = c.width | (c.height << 16);
  reg(kWin0SclFac) = 0;
  // Window 1 stays off, with the register image the vendor driver leaves an unused window in.
  reg(kWin1RdCtrl) = (1u << 8) | (1u << 10) | (1u << 20) | (1u << 22);
  // Overlap: the copy goes straight through (the field bit is what the driver sets for YUV output; the bottom layer's global alpha is opaque).
  reg(kOvlpCtrl) = 1u << 2;
  reg(kOvlpBotCtrl) = 0xffu << 16;
  // Write-back: NV12, raster, the vendor's outstanding-request limit of 15.
  reg(kWrCtrl) = (0u << 4) | (1u << 8) | (1u << 2) | (1u << 10) | (0xfu << 13);
  reg(kWrVirStride) = strideWords(c.dstStride);
  reg(kWrPlVirStride) = strideWords(c.dstStride);
  reg(kWrYBase) = c.dstY;
  reg(kWrUBase) = c.dstUv;
}

struct Regs {
  void *ctx;
  uint32_t (*read)(void *ctx, uint32_t off);          // offsets from the block's base (the system registers, the command registers)
  void (*write)(void *ctx, uint32_t off, uint32_t value);
  void (*delayUs)(void *ctx, uint32_t us);
  uint64_t (*nowUs)(void *ctx);
};

struct Result {
  bool done;            // the frame-done bit came up
  uint32_t intRaw;      // RGA3_INT_RAW at the end
  uint32_t status0;
  uint32_t cmdState;
  uint32_t us;          // start to completion (or to the give-up)
};

// Starts a job in slave mode and polls for its end: true when it finished with no error bit. Interrupts are not used (the block's line is shared with its
// IOMMU's, and nothing is registered for it): INT_EN is written so the raw status shows the bits, as the vendor driver's handler would see them.
inline bool run(const Regs &r, const uint32_t cmd[kCmdWords], uint32_t timeoutUs, Result *out) {
  const uint32_t sys = kSysLgcClkOn | kSysFrmendAutoReset;
  r.write(r.ctx, kIntClear, kIntFrameDone | kIntCmdLineFinish | kIntErrors);
  r.write(r.ctx, kIntEn, kIntFrameDone | kIntCmdLineFinish | kIntErrors);
  r.write(r.ctx, kSysCtrl, sys);   // slave mode: CMD_MODE 0
  r.write(r.ctx, kCmdAddr, 0);
  r.write(r.ctx, kCmdCtrl, 0);
  for (uint32_t i = 0; i < kCmdWords; ++i) r.write(r.ctx, kCmdBase + 4 * i, cmd[i]);
  const uint64_t t0 = r.nowUs(r.ctx);
  r.write(r.ctx, kSysCtrl, sys | kSysStart);
  Result res = {};
  for (;;) {
    res.intRaw = r.read(r.ctx, kIntRaw);
    if (res.intRaw & (kIntFrameDone | kIntErrors)) break;
    if (r.nowUs(r.ctx) - t0 > timeoutUs) break;
    r.delayUs(r.ctx, 1);
  }
  res.us = static_cast<uint32_t>(r.nowUs(r.ctx) - t0);
  res.done = (res.intRaw & kIntFrameDone) != 0;
  res.status0 = r.read(r.ctx, kStatus0);
  res.cmdState = r.read(r.ctx, kCmdState);
  r.write(r.ctx, kIntClear, kIntFrameDone | kIntCmdLineFinish | kIntErrors);
  if (out) *out = res;
  return res.done && !(res.intRaw & kIntErrors);
}

// The vendor driver's rga3_soft_reset: both resets on, wait for the done bits, off. The IOMMU lives in the reset domain (the vendor driver writes its
// directory address back and enables translation again): the caller brings it up again with rkiommu::enable. False when the block did not finish the reset.
inline bool softReset(const Regs &r) {
  r.write(r.ctx, kSysCtrl, kSysCclkReset | kSysAclkReset);
  bool done = false;
  for (unsigned i = 0; i < 1000 && !done; ++i) {
    if ((r.read(r.ctx, kRoSrst) & kResetDone) != 0) done = true;
    else r.delayUs(r.ctx, 1);
  }
  r.write(r.ctx, kSysCtrl, 0);
  return done;
}

}  // namespace rga3
