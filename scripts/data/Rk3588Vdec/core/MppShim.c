// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// MppShim.c -- the part of Rockchip's MPP that is NOT run as it is: the operating-system layer under the parser and the register generator.
//
// What runs unmodified (see core/mpp-files.txt): the H.264 parser, the vdpu34x register generator, the buffer-slot (decoded picture bookkeeping), frame
// and packet objects, the bit reader. What is replaced here is what MPP gets from Linux: memory, logging, threads, environment variables, SoC
// detection, the buffer pool (dma-buf in MPP, one physically contiguous window here), and the device (/dev/mpp_service in MPP, the rkvdec2 block here:
// registers collected into a job, address registers translated, the job handed to the embedder's run callback).
//
// Everything in MPP's tree that this file does not define is compiled from the pinned checkout. The names and signatures below are MPP's own.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "VdecPlatform.h"
#include "VdecCore.h"
#include "MppShim.h"

#include "mpp_buffer.h"
#include "mpp_callback.h"
#include "mpp_common.h"
#include "mpp_compat_impl.h"
#include "mpp_debug.h"
#include "mpp_device.h"
#include "mpp_env.h"
#include "mpp_log.h"
#include "mpp_mem.h"
#include "mpp_meta.h"
#include "mpp_singleton.h"
#include "mpp_soc.h"
#include "mpp_2str.h"
#include "mpp_sys_cfg.h"
#include "mpp_thread.h"
#include "mpp_time.h"
#include "rk_mpp_cfg.h"
#include "rk_vdec_cfg.h"
#include "hal_dbg.h"

// ------------------------------------------------------------------------------------------------------------------------------------------------
// memory, log, environment, singletons
// ------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct ShimAlloc {
  size_t size;
  size_t pad;   // keeps the payload 16-byte aligned
} ShimAlloc;

void *mpp_osal_calloc(const char *caller, size_t size) {
  (void)caller;
  ShimAlloc *a = (ShimAlloc *)vdc_calloc(sizeof(ShimAlloc) + size);
  if (!a) return NULL;
  a->size = size;
  return a + 1;
}

void *mpp_osal_malloc(const char *caller, size_t size) { return mpp_osal_calloc(caller, size); }

void mpp_osal_free(const char *caller, void *ptr) {
  (void)caller;
  if (!ptr) return;
  ShimAlloc *a = (ShimAlloc *)ptr - 1;
  vdc_free(a, sizeof(ShimAlloc) + a->size);
}

void *mpp_osal_realloc(const char *caller, void *ptr, size_t size) {
  if (!ptr) return mpp_osal_calloc(caller, size);
  ShimAlloc *a = (ShimAlloc *)ptr - 1;
  if (size <= a->size) return ptr;
  void *n = mpp_osal_calloc(caller, size);
  if (!n) return NULL;
  memcpy(n, ptr, a->size);
  mpp_osal_free(caller, ptr);
  return n;
}

int vdc_log_level = 3;
RK_U32 mpp_debug = 0;

void _mpp_log_l(int level, const char *tag, const char *fmt, const char *func, ...) {
  char msg[256];
  va_list ap;
  if (level <= 0 || level > vdc_log_level) return;
  int n = snprintf(msg, sizeof msg, "%s: %s ", tag ? tag : "mpp", func ? func : "");
  if (n < 0 || n >= (int)sizeof msg) n = 0;
  va_start(ap, func);
  vsnprintf(msg + n, sizeof msg - (size_t)n, fmt, ap);
  va_end(ap);
  size_t len = strlen(msg);
  while (len && (msg[len - 1] == '\n' || msg[len - 1] == ' ')) msg[--len] = 0;
  vdc_log(level, msg);
}

RK_S32 mpp_env_get_u32(const char *name, RK_U32 *value, RK_U32 default_value) {
  (void)name;
  *value = default_value;
  return 0;
}

RK_U32 mpp_hal_debug = 0;

// osal/mpp_soc.c: the slot of a coding type in the capability bit maps (the macro CODING_TO_IDX there)
RK_S32 mpp_coding_to_index(MppCodingType coding) {
  rk_u32 type = (rk_u32)coding;
  if (type >= (rk_u32)MPP_VIDEO_CodingKhronosExtensions) return (RK_S32)(rk_u32)-1;
  if (type >= (rk_u32)MPP_VIDEO_CodingVC1) return (RK_S32)(type - (rk_u32)MPP_VIDEO_CodingVC1 + 16);
  return (RK_S32)(type - (rk_u32)MPP_VIDEO_CodingUnused);
}

// mpp/base/mpp_2str.c: only the two codings the core knows are named
const char *strof_coding_type(MppCodingType coding) {
  switch (coding) {
    case MPP_VIDEO_CodingAVC: return "h264";
    case MPP_VIDEO_CodingHEVC: return "h265";
    default: return "unknown";
  }
}

rk_s32 mpp_singleton_add(MppSglnInfo *info, const char *caller) {
  (void)info;
  (void)caller;
  return 0;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// locks: one decoder runs at a time (the embedder serialises), so MPP's mutexes have nothing to protect
// ------------------------------------------------------------------------------------------------------------------------------------------------

void mpp_mutex_init(MppMutex *mutex) { (void)mutex; }
void mpp_mutex_destroy(MppMutex *mutex) { (void)mutex; }
void mpp_mutex_lock(MppMutex *mutex) { (void)mutex; }
void mpp_mutex_unlock(MppMutex *mutex) { (void)mutex; }

MppStopwatch mpp_stopwatch_get(const char *name) {
  (void)name;
  return NULL;
}
void mpp_stopwatch_set_show_on_exit(MppStopwatch stopwatch, rk_s32 show_on_exit) {
  (void)stopwatch;
  (void)show_on_exit;
}
void mpp_stopwatch_put(MppStopwatch timer) { (void)timer; }

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SoC: an RK3588 with the rkvdec2 block (MPP's table entry "rk3588": the vdpu38x capability record)
// ------------------------------------------------------------------------------------------------------------------------------------------------

static MppDecHwCap gHwCap;

static void init_hwcap(void) {
  if (gHwCap.type) return;
  memset(&gHwCap, 0, sizeof gHwCap);
  gHwCap.type = VPU_CLIENT_RKVDEC;
  gHwCap.cap_fbc = SOC_CAP_AFBC_V2;
  gHwCap.cap_data_layout = SOC_CAP_RASTER | SOC_CAP_AFBC16X16;
  gHwCap.cap_4k = 1;
  gHwCap.cap_8k = 1;
  gHwCap.cap_colmv_compress = 1;
  gHwCap.cap_hw_h265_rps = 1;
  gHwCap.cap_jpg_pp_out = 0;
  gHwCap.cap_10bit = 1;
  gHwCap.cap_down_scale = 1;
  gHwCap.cap_lmt_linebuf = 0;
  gHwCap.cap_core_num = 2;
  gHwCap.cap_hw_jpg_fix = 0;
}

const char *mpp_get_soc_name(void) { return "rk3588"; }
RockchipSocType mpp_get_soc_type(void) { return ROCKCHIP_SOC_RK3588; }
rk_u32 mpp_get_vcodec_type(void) { return 1u << VPU_CLIENT_RKVDEC; }
rk_u32 mpp_check_soc_cap(MppCtxType type, MppCodingType coding) {
  return type == MPP_CTX_DEC && (coding == MPP_VIDEO_CodingAVC || coding == MPP_VIDEO_CodingHEVC);
}
const MppDecHwCap *mpp_get_dec_hw_info_by_client_type(MppClientType client_type) {
  init_hwcap();
  return client_type == VPU_CLIENT_RKVDEC ? &gHwCap : NULL;
}

// MPP's compatibility switches (osal/mpp_compat.c): all off, like on a stock board
static RK_S32 gZero;
RK_S32 *compat_ext_fbc_buf_size = &gZero;
RK_S32 *compat_ext_async_input = &gZero;
RK_S32 *compat_ext_fbc_hdr_256_odd = &gZero;

// ------------------------------------------------------------------------------------------------------------------------------------------------
// debug dumps of the HAL (files in MPP): off
// ------------------------------------------------------------------------------------------------------------------------------------------------

MPP_RET hal_dbg_init(HalDbgCtx **ctx, const char *dump_sub_dir) {
  (void)dump_sub_dir;
  if (ctx) *ctx = NULL;
  return MPP_OK;
}
MPP_RET hal_dbg_deinit(HalDbgCtx **ctx) {
  if (ctx) *ctx = NULL;
  return MPP_OK;
}
MPP_RET hal_dbg_setup(HalDbgCtx *ctx, const char *fmt, ...) {
  (void)ctx; (void)fmt;
  return MPP_OK;
}
MPP_RET hal_dbg_finish(HalDbgCtx *ctx) {
  (void)ctx;
  return MPP_OK;
}
MPP_RET hal_dbg_dump_data(HalDbgCtx *ctx, char *fname, void *data, RK_U32 data_bit_size, RK_U32 line_bits, RK_U32 big_end, const char *mode) {
  (void)ctx; (void)fname; (void)data; (void)data_bit_size; (void)line_bits; (void)big_end; (void)mode;
  return MPP_OK;
}
MPP_RET hal_dbg_dump_raw_data(HalDbgCtx *ctx, const char *fname, void *data, size_t byte_sz, const char *mode) {
  (void)ctx; (void)fname; (void)data; (void)byte_sz; (void)mode;
  return MPP_OK;
}
MPP_RET hal_dbg_dump_regs(HalDbgCtx *ctx, RK_U32 *regs, RK_U32 reg_cnt, RK_U32 base_idx, const char *fname, const char *mode) {
  (void)ctx; (void)regs; (void)reg_cnt; (void)base_idx; (void)fname; (void)mode;
  return MPP_OK;
}
MPP_RET hal_dbg_log(HalDbgCtx *ctx, const char *fname, const char *mode, const char *fmt, ...) {
  (void)ctx; (void)fname; (void)mode; (void)fmt;
  return MPP_OK;
}
MPP_RET hal_dbg_load_data(HalDbgCtx *ctx, const char *fname, void *buf, RK_U32 buf_size, const char *mode) {
  (void)ctx; (void)fname; (void)buf; (void)buf_size; (void)mode;
  return MPP_NOK;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// meta data attached to frames and packets: nothing reads it here, so it only has to be a reference-counted handle
// ------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct ShimMeta {
  int ref;
} ShimMeta;

MPP_RET mpp_meta_get_with_tag(MppMeta *meta, const char *tag, const char *caller) {
  (void)tag;
  (void)caller;
  ShimMeta *m = (ShimMeta *)mpp_osal_calloc(caller, sizeof(ShimMeta));
  if (!m) return MPP_ERR_MALLOC;
  m->ref = 1;
  *meta = m;
  return MPP_OK;
}
MPP_RET mpp_meta_put(MppMeta meta) {
  ShimMeta *m = (ShimMeta *)meta;
  if (m && --m->ref <= 0) mpp_osal_free(__FUNCTION__, m);
  return MPP_OK;
}
MPP_RET mpp_meta_inc_ref(MppMeta meta) {
  if (meta) ((ShimMeta *)meta)->ref++;
  return MPP_OK;
}
MppMeta mpp_meta_dup(MppMeta meta) {
  if (meta) ((ShimMeta *)meta)->ref++;
  return meta;
}
MPP_RET mpp_meta_set_s32(MppMeta meta, MppMetaKey key, RK_S32 val) {
  (void)meta;
  (void)key;
  (void)val;
  return MPP_OK;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// decoder configuration objects of MPP's public interface (kmpp objects in MPP): the slots only write the picture size into it
// ------------------------------------------------------------------------------------------------------------------------------------------------

MPP_RET mpp_dec_cfg_set_s32(MppDecCfg cfg, const char *name, RK_S32 val) {
  (void)cfg;
  (void)name;
  (void)val;
  return MPP_OK;
}

// MppSysCfg: the picture buffer size and stride query of the slots (dec_buf_chk). mpp_sys_dec_buf_chk_proc is MPP's own, cut out of mpp_sys_cfg.c by
// core/derive-mpp.py (the rest of that file is the kmpp object machinery); this is the handle around it.
typedef struct ShimSysCfg {
  MppSysCfgSet set;
} ShimSysCfg;

rk_s32 mpp_sys_cfg_get(MppSysCfg *cfg) {
  ShimSysCfg *c = (ShimSysCfg *)mpp_osal_calloc(__FUNCTION__, sizeof(ShimSysCfg));
  if (!c) return -1;
  *cfg = c;
  return 0;
}
rk_s32 mpp_sys_cfg_put(MppSysCfg cfg) {
  mpp_osal_free(__FUNCTION__, cfg);
  return 0;
}
MPP_RET mpp_sys_cfg_ioctl(MppSysCfg cfg) {
  ShimSysCfg *c = (ShimSysCfg *)cfg;
  if (c->set.dec_buf_chk.enable) {
    mpp_sys_dec_buf_chk_proc(&c->set.dec_buf_chk);
    c->set.dec_buf_chk.enable = 0;
  }
  return MPP_OK;
}

#define SYS_FIELDS(X) \
  X("dec_buf_chk:enable", enable) \
  X("dec_buf_chk:type", type) \
  X("dec_buf_chk:fmt_codec", fmt_codec) \
  X("dec_buf_chk:fmt_fbc", fmt_fbc) \
  X("dec_buf_chk:fmt_hdr", fmt_hdr) \
  X("dec_buf_chk:width", width) \
  X("dec_buf_chk:height", height) \
  X("dec_buf_chk:h_stride_by_byte", h_stride_by_byte) \
  X("dec_buf_chk:v_stride", v_stride) \
  X("dec_buf_chk:h_stride_by_pixel", h_stride_by_pixel) \
  X("dec_buf_chk:size_total", size_total)

MPP_RET mpp_sys_cfg_set_u32(MppSysCfg cfg, const char *name, RK_U32 val) {
  ShimSysCfg *c = (ShimSysCfg *)cfg;
#define X(key, field) \
  if (!strcmp(name, key)) { \
    c->set.dec_buf_chk.field = val; \
    return MPP_OK; \
  }
  SYS_FIELDS(X)
#undef X
  return MPP_NOK;
}
MPP_RET mpp_sys_cfg_get_u32(MppSysCfg cfg, const char *name, RK_U32 *val) {
  ShimSysCfg *c = (ShimSysCfg *)cfg;
#define X(key, field) \
  if (!strcmp(name, key)) { \
    *val = c->set.dec_buf_chk.field; \
    return MPP_OK; \
  }
  SYS_FIELDS(X)
#undef X
  return MPP_NOK;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// the DMA window and MPP's buffers in it
//
// MPP allocates dma-bufs and passes their file descriptors around; the block's address registers hold "fd | offset << 10" and the kernel driver
// translates them. Here a buffer is a piece of the window and its "fd" is a small number (1..1023, 0 means none); the translation is in mpp_dev_ioctl.
// Buffers of an internal group that are released stay in the group and are handed out again, like MPP does (the register images depend on which
// buffer a picture lands in).
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define kMaxFd 1024
#define kMaxExtents 128
#define kPage 4096u

typedef struct ShimGroup ShimGroup;
typedef struct ShimBuffer {
  struct ShimBuffer *next;   // in the group's unused list
  ShimGroup *group;
  uint32_t id;               // the fd
  uint32_t winOff;           // offset in the window
  uint32_t winLen;           // page rounded
  size_t size;               // as asked for
  uint8_t *ptr;
  size_t offset;
  int index;
  int ref;
  int used;
  int discard;
} ShimBuffer;

struct ShimGroup {
  ShimBuffer *unusedHead, *unusedTail;
  int count;     // the unused ones
  int buffers;   // all of them, used or not
  int dead;      // put by its owner: it goes away with its last buffer
};

typedef struct Extent {
  uint32_t off, len;
} Extent;

static struct {
  uint8_t *virt;
  uint64_t bus;
  uint32_t bytes;
  Extent free[kMaxExtents];
  unsigned nFree;
  ShimBuffer *byId[kMaxFd];
  uint32_t nextId;
  uint32_t liveBuffers;
  uint32_t liveBytes;
  uint32_t peakBytes;
} gWin;

static const VdecHw *gHw;

void vdec_shim_bind(const VdecHw *hw) {
  gHw = hw;
  if (gWin.virt == hw->dmaVirt && gWin.bytes == hw->dmaBytes) return;
  memset(&gWin, 0, sizeof gWin);
  gWin.virt = hw->dmaVirt;
  gWin.bus = hw->dmaBus;
  gWin.bytes = hw->dmaBytes;
  gWin.free[0].off = 0;
  gWin.free[0].len = hw->dmaBytes & ~(kPage - 1);
  gWin.nFree = 1;
  gWin.nextId = 1;
}

void vdec_shim_window_stats(uint32_t *buffers, uint32_t *bytes, uint32_t *peak) {
  if (buffers) *buffers = gWin.liveBuffers;
  if (bytes) *bytes = gWin.liveBytes;
  if (peak) *peak = gWin.peakBytes;
}

static int win_alloc(uint32_t len, uint32_t *off) {
  for (unsigned i = 0; i < gWin.nFree; i++) {
    if (gWin.free[i].len >= len) {
      *off = gWin.free[i].off;
      gWin.free[i].off += len;
      gWin.free[i].len -= len;
      if (gWin.free[i].len == 0) {
        memmove(&gWin.free[i], &gWin.free[i + 1], (gWin.nFree - i - 1) * sizeof(Extent));
        gWin.nFree--;
      }
      return 0;
    }
  }
  return -1;
}

static void win_free(uint32_t off, uint32_t len) {
  unsigned i = 0;
  while (i < gWin.nFree && gWin.free[i].off < off) i++;
  if (i > 0 && gWin.free[i - 1].off + gWin.free[i - 1].len == off) {
    gWin.free[i - 1].len += len;
    if (i < gWin.nFree && gWin.free[i - 1].off + gWin.free[i - 1].len == gWin.free[i].off) {
      gWin.free[i - 1].len += gWin.free[i].len;
      memmove(&gWin.free[i], &gWin.free[i + 1], (gWin.nFree - i - 1) * sizeof(Extent));
      gWin.nFree--;
    }
    return;
  }
  if (i < gWin.nFree && off + len == gWin.free[i].off) {
    gWin.free[i].off = off;
    gWin.free[i].len += len;
    return;
  }
  if (gWin.nFree == kMaxExtents) return;   // leaks the piece: the window fragments beyond what the bookkeeping holds
  memmove(&gWin.free[i + 1], &gWin.free[i], (gWin.nFree - i) * sizeof(Extent));
  gWin.free[i].off = off;
  gWin.free[i].len = len;
  gWin.nFree++;
}

static void buffer_destroy(ShimBuffer *b) {
  ShimGroup *g = b->group;
  gWin.byId[b->id] = NULL;
  gWin.liveBuffers--;
  gWin.liveBytes -= b->winLen;
  win_free(b->winOff, b->winLen);
  mpp_osal_free(__FUNCTION__, b);
  if (--g->buffers == 0 && g->dead) mpp_osal_free(__FUNCTION__, g);
}

static void group_unlink(ShimGroup *g, ShimBuffer *prev, ShimBuffer *b) {
  if (prev) prev->next = b->next;
  else g->unusedHead = b->next;
  if (g->unusedTail == b) g->unusedTail = prev;
  b->next = NULL;
  g->count--;
}

MPP_RET mpp_buffer_group_get(MppBufferGroup *group, MppBufferType type, MppBufferMode mode, const char *tag, const char *caller) {
  (void)type;
  (void)mode;
  (void)tag;
  ShimGroup *g = (ShimGroup *)mpp_osal_calloc(caller, sizeof(ShimGroup));
  if (!g) return MPP_ERR_MALLOC;
  *group = g;
  return MPP_OK;
}

MPP_RET mpp_buffer_group_clear(MppBufferGroup group) {
  ShimGroup *g = (ShimGroup *)group;
  while (g && g->unusedHead) {
    ShimBuffer *b = g->unusedHead;
    group_unlink(g, NULL, b);
    buffer_destroy(b);
  }
  return MPP_OK;
}

MPP_RET mpp_buffer_group_put(MppBufferGroup group) {
  ShimGroup *g = (ShimGroup *)group;
  if (!g) return MPP_OK;
  mpp_buffer_group_clear(g);
  g->dead = 1;   // buffers still in use free themselves when released, the group with the last of them
  if (g->buffers == 0) mpp_osal_free(__FUNCTION__, g);
  return MPP_OK;
}

RK_S32 mpp_buffer_group_unused(MppBufferGroup group) { return group ? ((ShimGroup *)group)->count : 0; }

MPP_RET mpp_buffer_get_with_tag(MppBufferGroup group, MppBuffer *buffer, size_t size, const char *tag, const char *caller) {
  (void)tag;
  ShimGroup *g = (ShimGroup *)group;
  if (!buffer || !size || !g) {
    mpp_err("mpp_buffer_get invalid input: group %p buffer %p size %u from %s\n", group, buffer, (unsigned)size, caller);
    return MPP_ERR_UNKNOW;
  }
  *buffer = NULL;
  // an unused buffer that is large enough first; smaller unused ones in front of it are let go (MPP: mpp_buffer_get_unused)
  ShimBuffer *b;
  while ((b = g->unusedHead) != NULL) {
    group_unlink(g, NULL, b);
    if (b->size >= size) {
      b->ref = 1;
      b->used = 1;
      *buffer = b;
      return MPP_OK;
    }
    buffer_destroy(b);
  }
  // none: a new piece of the window
  uint32_t len = ((uint32_t)size + kPage - 1) & ~(kPage - 1);
  uint32_t off;
  if (size > gWin.bytes || win_alloc(len, &off) < 0) {
    mpp_err("DMA window exhausted: %u bytes asked for by %s (%u buffers, %u bytes live)\n", (unsigned)size, caller, gWin.liveBuffers, gWin.liveBytes);
    return MPP_NOK;
  }
  uint32_t id = 0;
  for (uint32_t n = 0; n < kMaxFd; n++) {
    uint32_t cand = (gWin.nextId + n) % kMaxFd;
    if (cand && !gWin.byId[cand]) {
      id = cand;
      break;
    }
  }
  if (!id) {
    win_free(off, len);
    mpp_err("out of buffer numbers (%u live)\n", gWin.liveBuffers);
    return MPP_NOK;
  }
  gWin.nextId = id + 1;
  b = (ShimBuffer *)mpp_osal_calloc(caller, sizeof(ShimBuffer));
  if (!b) {
    win_free(off, len);
    return MPP_ERR_MALLOC;
  }
  b->group = g;
  g->buffers++;
  b->id = id;
  b->winOff = off;
  b->winLen = len;
  b->size = size;
  b->ptr = gWin.virt + off;
  b->ref = 1;
  b->used = 1;
  b->index = -1;
  memset(b->ptr, 0, len);   // a fresh dma-buf is zero
  gWin.byId[id] = b;
  gWin.liveBuffers++;
  gWin.liveBytes += len;
  if (gWin.liveBytes > gWin.peakBytes) gWin.peakBytes = gWin.liveBytes;
  *buffer = b;
  return MPP_OK;
}

MPP_RET mpp_buffer_put_with_caller(MppBuffer buffer, const char *caller) {
  ShimBuffer *b = (ShimBuffer *)buffer;
  if (!b) {
    mpp_err("mpp_buffer_put invalid input: buffer NULL from %s\n", caller);
    return MPP_ERR_UNKNOW;
  }
  if (b->ref <= 0) {
    mpp_err("buffer %u released more often than taken (from %s)\n", b->id, caller);
    return MPP_NOK;
  }
  if (--b->ref > 0) return MPP_OK;
  b->used = 0;
  ShimGroup *g = b->group;
  if (b->discard || g->dead) {
    buffer_destroy(b);
    return MPP_OK;
  }
  b->next = NULL;
  if (g->unusedTail) g->unusedTail->next = b;
  else g->unusedHead = b;
  g->unusedTail = b;
  g->count++;
  return MPP_OK;
}

MPP_RET mpp_buffer_inc_ref_with_caller(MppBuffer buffer, const char *caller) {
  ShimBuffer *b = (ShimBuffer *)buffer;
  if (!b) {
    mpp_err("mpp_buffer_inc_ref invalid input: buffer NULL from %s\n", caller);
    return MPP_ERR_UNKNOW;
  }
  b->ref++;
  return MPP_OK;
}

MPP_RET mpp_buffer_discard_with_caller(MppBuffer buffer, const char *caller) {
  (void)caller;
  if (buffer) ((ShimBuffer *)buffer)->discard = 1;
  return MPP_OK;
}

MPP_RET mpp_buffer_write_with_caller(MppBuffer buffer, size_t offset, void *data, size_t size, const char *caller) {
  ShimBuffer *b = (ShimBuffer *)buffer;
  if (!b || !data) {
    mpp_err("mpp_buffer_write invalid input: buffer %p data %p from %s\n", buffer, data, caller);
    return MPP_ERR_UNKNOW;
  }
  if (!size) return MPP_OK;
  if (offset + size > b->size) return MPP_ERR_VALUE;
  memcpy(b->ptr + offset, data, size);
  return MPP_OK;
}

MPP_RET mpp_buffer_read_with_caller(MppBuffer buffer, size_t offset, void *data, size_t size, const char *caller) {
  ShimBuffer *b = (ShimBuffer *)buffer;
  if (!b || !data) {
    mpp_err("mpp_buffer_read invalid input: buffer %p data %p from %s\n", buffer, data, caller);
    return MPP_ERR_UNKNOW;
  }
  if (!size) return MPP_OK;
  if (offset + size > b->size) return MPP_ERR_VALUE;
  memcpy(data, b->ptr + offset, size);
  return MPP_OK;
}

void *mpp_buffer_get_ptr_with_caller(MppBuffer buffer, const char *caller) {
  (void)caller;
  return buffer ? ((ShimBuffer *)buffer)->ptr : NULL;
}
int mpp_buffer_get_fd_with_caller(MppBuffer buffer, const char *caller) {
  (void)caller;
  return buffer ? (int)((ShimBuffer *)buffer)->id : -1;
}
size_t mpp_buffer_get_size_with_caller(MppBuffer buffer, const char *caller) {
  (void)caller;
  return buffer ? ((ShimBuffer *)buffer)->size : 0;
}
int mpp_buffer_get_index_with_caller(MppBuffer buffer, const char *caller) {
  (void)caller;
  return buffer ? ((ShimBuffer *)buffer)->index : -1;
}
MPP_RET mpp_buffer_set_index_with_caller(MppBuffer buffer, int index, const char *caller) {
  (void)caller;
  if (buffer) ((ShimBuffer *)buffer)->index = index;
  return MPP_OK;
}
size_t mpp_buffer_get_offset_with_caller(MppBuffer buffer, const char *caller) {
  (void)caller;
  return buffer ? ((ShimBuffer *)buffer)->offset : 0;
}
MPP_RET mpp_buffer_set_offset_with_caller(MppBuffer buffer, size_t offset, const char *caller) {
  (void)caller;
  if (buffer) ((ShimBuffer *)buffer)->offset = offset;
  return MPP_OK;
}

// The window is normal non-cacheable memory (what the CPU writes is what the block reads): nothing to flush.
MPP_RET mpp_buffer_sync_begin_f(MppBuffer buffer, RK_S32 ro, const char *caller) {
  (void)buffer; (void)ro; (void)caller;
  return MPP_OK;
}
MPP_RET mpp_buffer_sync_end_f(MppBuffer buffer, RK_S32 ro, const char *caller) {
  (void)buffer; (void)ro; (void)caller;
  return MPP_OK;
}
MPP_RET mpp_buffer_sync_partial_begin_f(MppBuffer buffer, RK_S32 ro, RK_U32 offset, RK_U32 length, const char *caller) {
  (void)buffer; (void)ro; (void)offset; (void)length; (void)caller;
  return MPP_OK;
}
MPP_RET mpp_buffer_sync_partial_end_f(MppBuffer buffer, RK_S32 ro, RK_U32 offset, RK_U32 length, const char *caller) {
  (void)buffer; (void)ro; (void)offset; (void)length; (void)caller;
  return MPP_OK;
}

uint64_t vdec_shim_buffer_bus(MppBuffer buffer) {
  ShimBuffer *b = (ShimBuffer *)buffer;
  return b ? gWin.bus + b->winOff : 0;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// the device
//
// The HAL describes a job by writing register ranges, address-register offsets and read-backs, then sends and polls. The vendor kernel driver
// (mpp_rkvdec2.c) translates the address registers (fd | offset << 10 -> bus address) of a fixed list and runs the job. Here the same happens and the
// run itself is the embedder's.
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define kRegs VDEC_REG_COUNT
#define kMaxOffs 64
#define kMaxReads 8

// the vendor kernel's trans_tbl_h264d / h265d (mpp_rkvdec2.c): registers that hold "fd | offset << 10"
static const uint16_t kTrans[] = {128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 161, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
                                  173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199};

typedef struct ShimDev {
  uint32_t regs[kRegs];
  uint8_t set[kRegs];
  uint32_t offReg[kMaxOffs], offVal[kMaxOffs];
  unsigned nOff;
  struct {
    void *p;
    uint32_t size, offset;
  } rd[kMaxReads];
  unsigned nRd, rdDone;
  uint32_t status;
  int sent;
  VdecJobObserver *observer;
} ShimDev;

MPP_RET mpp_dev_init(MppDev *ctx, MppClientType type) {
  (void)type;
  ShimDev *d = (ShimDev *)mpp_osal_calloc(__FUNCTION__, sizeof(ShimDev));
  if (!d) return MPP_ERR_MALLOC;
  *ctx = d;
  return MPP_OK;
}

MPP_RET mpp_dev_deinit(MppDev ctx) {
  mpp_osal_free(__FUNCTION__, ctx);
  return MPP_OK;
}

void vdec_shim_set_observer(MppDev dev, VdecJobObserver *observer) {
  if (dev) ((ShimDev *)dev)->observer = observer;
}

static void dev_reset_job(ShimDev *d) {
  memset(d->regs, 0, sizeof d->regs);
  memset(d->set, 0, sizeof d->set);
  d->nOff = 0;
  d->nRd = 0;
  d->rdDone = 0;
}

static void dev_add_offset(ShimDev *d, uint32_t reg, uint32_t offset) {
  if (d->nOff < kMaxOffs) {
    d->offReg[d->nOff] = reg;
    d->offVal[d->nOff] = offset;
    d->nOff++;
  }
}

MPP_RET mpp_dev_set_reg_offset(MppDev dev, RK_S32 index, RK_U32 offset) {
  MppDevRegOffsetCfg cfg = {(RK_U32)index, offset};
  return mpp_dev_ioctl(dev, MPP_DEV_REG_OFFSET, &cfg);
}

static MPP_RET dev_send(ShimDev *d) {
  uint32_t regs[kRegs];
  VdecRun runs[kRegs / 2 + 1];
  unsigned nRuns = 0;
  VdecJobInfo info;
  memcpy(regs, d->regs, sizeof regs);
  memset(&info, 0, sizeof info);
  for (unsigned t = 0; t < sizeof kTrans / sizeof kTrans[0]; t++) {
    uint32_t idx = kTrans[t];
    if (!d->set[idx]) continue;
    uint32_t v = d->regs[idx];
    uint32_t id = v & 0x3ff, off = v >> 10;
    for (unsigned i = 0; i < d->nOff; i++)
      if (d->offReg[i] == idx) off += d->offVal[i];
    ShimBuffer *b = (id && id < kMaxFd) ? gWin.byId[id] : NULL;
    if (!b) {
      if (v) mpp_err("register %u = %#x names no buffer\n", (unsigned)idx, (unsigned)v);
      regs[idx] = 0;
      continue;
    }
    if (off >= b->winLen) {
      mpp_err("register %u: offset %u beyond buffer %u (%u bytes)\n", (unsigned)idx, (unsigned)off, (unsigned)id, (unsigned)b->winLen);
      return MPP_NOK;
    }
    regs[idx] = (uint32_t)(gWin.bus + b->winOff + off);
    if (info.nReloc < VDEC_MAX_RELOCS) {
      info.reloc[info.nReloc].reg = idx;
      info.reloc[info.nReloc].buf = id;
      info.reloc[info.nReloc].offset = off;
      info.reloc[info.nReloc].bufSize = (uint32_t)b->size;
      info.nReloc++;
    }
  }
  for (unsigned i = 0; i < kRegs;) {
    if (!d->set[i]) {
      i++;
      continue;
    }
    unsigned j = i;
    while (j < kRegs && d->set[j]) j++;
    runs[nRuns].first = (uint16_t)i;
    runs[nRuns].count = (uint16_t)(j - i);
    nRuns++;
    i = j;
  }
  info.rawRegs = d->regs;
  info.regs = regs;
  info.runs = runs;
  info.nRuns = nRuns;
  d->status = 0;
  if (d->observer && d->observer->before) d->observer->before(d->observer->ctx, &info);
  int rc = gHw->run(gHw->ctx, regs, runs, nRuns, &d->status);
  if (d->observer && d->observer->after) d->observer->after(d->observer->ctx, &info, rc, d->status);
  d->sent = 1;
  return rc == 0 ? MPP_OK : MPP_NOK;
}

static MPP_RET dev_poll(ShimDev *d) {
  for (unsigned i = d->rdDone; i < d->nRd; i++) {
    memset(d->rd[i].p, 0, d->rd[i].size);
    if (d->rd[i].offset == 224 * 4 && d->rd[i].size >= 4) *(uint32_t *)d->rd[i].p = d->status;   // the interrupt status: the HAL looks at the error bits
  }
  d->rdDone = d->nRd;
  d->sent = 0;
  dev_reset_job(d);
  return MPP_OK;
}

MPP_RET mpp_dev_ioctl(MppDev ctx, RK_S32 cmd, void *param) {
  ShimDev *d = (ShimDev *)ctx;
  if (!d) return MPP_ERR_NULL_PTR;
  switch (cmd) {
    case MPP_DEV_REG_WR: {
      MppDevRegWrCfg *c = (MppDevRegWrCfg *)param;
      uint32_t first = c->offset / 4, n = c->size / 4;
      if (first + n > kRegs) return MPP_ERR_VALUE;
      memcpy(&d->regs[first], c->reg, c->size);
      for (uint32_t i = 0; i < n; i++) d->set[first + i] = 1;
      return MPP_OK;
    }
    case MPP_DEV_REG_RD: {
      MppDevRegRdCfg *c = (MppDevRegRdCfg *)param;
      if (d->nRd >= kMaxReads) return MPP_ERR_VALUE;
      d->rd[d->nRd].p = c->reg;
      d->rd[d->nRd].size = c->size;
      d->rd[d->nRd].offset = c->offset;
      d->nRd++;
      return MPP_OK;
    }
    case MPP_DEV_REG_OFFSET: {
      MppDevRegOffsetCfg *c = (MppDevRegOffsetCfg *)param;
      dev_add_offset(d, c->reg_idx, c->offset);
      return MPP_OK;
    }
    case MPP_DEV_REG_OFFS: {
      MppDevRegOffCfgs *c = (MppDevRegOffCfgs *)param;
      for (RK_S32 i = 0; i < c->count; i++) dev_add_offset(d, c->cfgs[i].reg_idx, c->cfgs[i].offset);
      return MPP_OK;
    }
    case MPP_DEV_CMD_SEND:
      return dev_send(d);
    case MPP_DEV_CMD_POLL:
      return dev_poll(d);
    default:   // batch mode, RCB info (SRAM placement), codec info strings, flags, buffer mapping: nothing to do without the vendor kernel
      return MPP_OK;
  }
}
