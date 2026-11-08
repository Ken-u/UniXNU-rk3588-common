// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// EncShim.c -- the part of Rockchip's MPP that is NOT run as it is, for the encoder: the operating-system layer under the codec, the rate control and the
// register generator (the twin of ../../Rk3588Vdec/core/MppShim.c).
//
// What runs unmodified (see core/mpp-files.txt): the H.264 syntax writers, the reference management, the rate control models, the vepu580 register generator,
// the configuration tables (kmpp objects), frames, packets, tasks and ports. What is replaced here is what MPP gets from Linux: memory, logging, threads
// and locks (one stream at a time, driven by one caller: nothing to protect), environment variables, SoC detection, the buffer pool (dma-buf in MPP, one
// physically contiguous window here), and the device (/dev/mpp_service in MPP, RKVENC0 here: registers collected into a job, address registers translated,
// the job handed to the embedder's run callback).
//
// Everything in MPP's tree that this file does not define is compiled from the pinned checkout. The names and signatures below are MPP's own.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "VencPlatform.h"
#include "VencCore.h"
#include "EncShim.h"

#include "mpp_buffer.h"
#include "mpp_buffer_impl.h"
#include "mpp_callback.h"
#include "mpp_common.h"
#include "mpp_compat_impl.h"
#include "mpp_debug.h"
#include "mpp_device.h"
#include "mpp_dmabuf.h"
#include "mpp_env.h"
#include "mpp_info.h"
#include "mpp_log.h"
#include "mpp_lock.h"
#include "hal_dbg.h"
#include "mpp_mem.h"
#include "mpp_meta.h"
#include "mpp_runtime.h"
#include "mpp_singleton.h"
#include "mpp_soc.h"
#include "mpp_thread.h"
#include "mpp_time.h"
#include "mpp.h"
#include "mpp_impl.h"
#include "hal_hw_id.h"
#include "mpp_platform.h"

// ------------------------------------------------------------------------------------------------------------------------------------------------
// memory, log, environment, singletons
// ------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct ShimAlloc {
  size_t size;
  size_t pad;   // keeps the payload 16-byte aligned
} ShimAlloc;

void *mpp_osal_calloc(const char *caller, size_t size) {
  (void)caller;
  ShimAlloc *a = (ShimAlloc *)vec_calloc(sizeof(ShimAlloc) + size);
  if (!a) return NULL;
  a->size = size;
  return a + 1;
}

void *mpp_osal_malloc(const char *caller, size_t size) { return mpp_osal_calloc(caller, size); }

void mpp_osal_free(const char *caller, void *ptr) {
  (void)caller;
  if (!ptr) return;
  ShimAlloc *a = (ShimAlloc *)ptr - 1;
  vec_free(a, sizeof(ShimAlloc) + a->size);
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

int vec_log_level = 3;
RK_U32 mpp_debug = 0;
RK_U32 mpp_cfg_io_debug = 0;
RK_U32 mpp_enc_hal_debug = 0;
RK_U32 mpp_hal_debug = 0;

void _mpp_log_l(int level, const char *tag, const char *fmt, const char *func, ...) {
  char msg[256];
  va_list ap;
  if (level <= 0 || level > vec_log_level) return;
  int n = snprintf(msg, sizeof msg, "%s: %s ", tag ? tag : "mpp", func ? func : "");
  if (n < 0 || n >= (int)sizeof msg) n = 0;
  va_start(ap, func);
  vsnprintf(msg + n, sizeof msg - (size_t)n, fmt, ap);
  va_end(ap);
  size_t len = strlen(msg);
  while (len && (msg[len - 1] == '\n' || msg[len - 1] == ' ')) msg[--len] = 0;
  vec_log(level, msg);
}

RK_S32 mpp_env_get_u32(const char *name, RK_U32 *value, RK_U32 default_value) {
  (void)name;
  *value = default_value;
  return 0;
}

const char *get_mpp_version(void) { return "mpp-rk3588-core"; }

rk_u32 mpp_rt_get_kmpp_cap(const char *module, const char *group, const char *name) {
  (void)module;
  (void)group;
  (void)name;
  return 0;
}

rk_s64 mpp_time(void) { return vec_time_us(); }

// MPP's modules register their one-time setup (the configuration tables, the object definitions, the rate control table, the buffer and packet pools) from
// constructors, MPP_SINGLETON() in osal/mpp_singleton.h, and a last constructor runs all of them in the order of their ids. Constructor priorities are
// not honoured on Mach-O, so that last step cannot be trusted to come last: the constructors only register here (the same bookkeeping as osal/mpp_singleton.c
// with its list), and venc_shim_run_singletons() runs the setups once, when the first core is created.
#include "mpp_sgln_base.h"

#define kSglnWithId 64
#define kSglnWithoutId 128
static MppSglnInfo gSglnWithId[kSglnWithId], gSglnWithoutId[kSglnWithoutId];
static MppSglnList gSglnListWithId = {0, kSglnWithId, gSglnWithId}, gSglnListWithoutId = {0, kSglnWithoutId, gSglnWithoutId};
static MppSglnBase gSglnBase = {"mpp_sgln_debug", {&gSglnListWithId, &gSglnListWithoutId}, 2, 12, 0, 0};

rk_s32 mpp_singleton_add(MppSglnInfo *info, const char *caller) { return mpp_sgln_base_add(&gSglnBase, info, caller); }

int venc_shim_run_singletons(void) {
  if (gSglnBase.init_done) return 0;
  if (!gSglnListWithId.count) return -1;   // no constructor ran: the module constructors of MPP's objects are not run in this image
  mpp_sgln_base_run_init(&gSglnBase);
  return 0;
}

void venc_shim_deinit_singletons(void) {
  if (!gSglnBase.init_done) return;
  mpp_sgln_base_deinit(&gSglnBase);
  gSglnBase.init_done = 0;
}

// osal/mpp_soc.c: the slot of a coding type in the capability bit maps (the macro CODING_TO_IDX there)
RK_S32 mpp_coding_to_index(MppCodingType coding) {
  rk_u32 type = (rk_u32)coding;
  if (type >= (rk_u32)MPP_VIDEO_CodingKhronosExtensions) return (RK_S32)(rk_u32)-1;
  if (type >= (rk_u32)MPP_VIDEO_CodingVC1) return (RK_S32)(type - (rk_u32)MPP_VIDEO_CodingVC1 + 16);
  return (RK_S32)(type - (rk_u32)MPP_VIDEO_CodingUnused);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// locks, threads, stop watches: one stream at a time, one caller -- nothing runs concurrently, so a lock protects nothing and a wait would never end
// ------------------------------------------------------------------------------------------------------------------------------------------------

void mpp_mutex_init(MppMutex *mutex) { (void)mutex; }
void mpp_mutex_destroy(MppMutex *mutex) { (void)mutex; }
void mpp_mutex_lock(MppMutex *mutex) { (void)mutex; }
void mpp_mutex_unlock(MppMutex *mutex) { (void)mutex; }

void mpp_cond_init(MppCond *condition) { (void)condition; }
void mpp_cond_destroy(MppCond *condition) { (void)condition; }
rk_s32 mpp_cond_wait(MppCond *condition, MppMutex *mutex) {
  (void)condition;
  (void)mutex;
  mpp_err("a wait on a condition would never end in the encoder core\n");
  return -1;
}
rk_s32 mpp_cond_timedwait(MppCond *condition, MppMutex *mutex, rk_s64 timeout) {
  (void)condition;
  (void)mutex;
  (void)timeout;
  return -1;   // timed out: the caller goes on
}
rk_s32 mpp_cond_signal(MppCond *condition) {
  (void)condition;
  return 0;
}
rk_s32 mpp_cond_broadcast(MppCond *condition) {
  (void)condition;
  return 0;
}

void mpp_mutex_cond_init(MppMutexCond *mutexCond) { (void)mutexCond; }
void mpp_mutex_cond_destroy(MppMutexCond *mutexCond) { (void)mutexCond; }
void mpp_mutex_cond_lock(MppMutexCond *mutexCond) { (void)mutexCond; }
void mpp_mutex_cond_unlock(MppMutexCond *mutexCond) { (void)mutexCond; }
int mpp_mutex_cond_trylock(MppMutexCond *mutexCond) {
  (void)mutexCond;
  return 0;
}
rk_s32 mpp_mutex_cond_wait(MppMutexCond *mutexCond) {
  (void)mutexCond;
  mpp_err("a wait on a condition would never end in the encoder core\n");
  return -1;
}
rk_s32 mpp_mutex_cond_timedwait(MppMutexCond *mutexCond, rk_s64 timeout) {
  (void)mutexCond;
  (void)timeout;
  return -1;
}
void mpp_mutex_cond_signal(MppMutexCond *mutexCond) { (void)mutexCond; }
void mpp_mutex_cond_broadcast(MppMutexCond *mutexCond) { (void)mutexCond; }

// one thread, so a lock can never be held by somebody else
void mpp_spinlock_init(spinlock_t *lock) { (void)lock; }
void mpp_spinlock_deinit(spinlock_t *lock, const char *name) { (void)lock; (void)name; }
void mpp_spinlock_lock(spinlock_t *lock) { (void)lock; }
void mpp_spinlock_unlock(spinlock_t *lock) { (void)lock; }
bool mpp_spinlock_trylock(spinlock_t *lock) { (void)lock; return true; }

// MPP's encoder thread is not started (core/VencCore.c runs its steps itself); the calls of the control path that look at it have nothing to do
MppThread *mpp_thread_create(MppThreadFunc func, void *ctx, const char *name) {
  (void)func;
  (void)ctx;
  (void)name;
  return NULL;
}
void mpp_thread_destroy(MppThread *thread) { (void)thread; }
void mpp_thread_start(MppThread *thread) { (void)thread; }
void mpp_thread_stop(MppThread *thread) { (void)thread; }
void mpp_thread_set_status(MppThread *thread, MppThreadStatus status, MppThreadSignalId id) {
  (void)thread;
  (void)status;
  (void)id;
}
MppThreadStatus mpp_thread_get_status(MppThread *thread, MppThreadSignalId id) {
  (void)thread;
  (void)id;
  return MPP_THREAD_RUNNING;
}
void mpp_thread_lock(MppThread *thread, MppThreadSignalId id) {
  (void)thread;
  (void)id;
}
void mpp_thread_unlock(MppThread *thread, MppThreadSignalId id) {
  (void)thread;
  (void)id;
}
void mpp_thread_wait(MppThread *thread, MppThreadSignalId id) {
  (void)thread;
  (void)id;
}
void mpp_thread_signal(MppThread *thread, MppThreadSignalId id) {
  (void)thread;
  (void)id;
}

MppStopwatch mpp_stopwatch_get(const char *name) {
  (void)name;
  return NULL;
}
void mpp_stopwatch_set_show_on_exit(MppStopwatch stopwatch, rk_s32 show_on_exit) {
  (void)stopwatch;
  (void)show_on_exit;
}
void mpp_stopwatch_record(MppStopwatch stopwatch, const char *event) {
  (void)stopwatch;
  (void)event;
}
void mpp_stopwatch_put(MppStopwatch timer) { (void)timer; }

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SoC: an RK3588 with the RKVENC0 block (the vepu580 of MPP)
// ------------------------------------------------------------------------------------------------------------------------------------------------

const char *mpp_get_soc_name(void) { return "rk3588"; }
RockchipSocType mpp_get_soc_type(void) { return ROCKCHIP_SOC_RK3588; }
rk_u32 mpp_get_vcodec_type(void) { return 1u << VPU_CLIENT_RKVENC; }
rk_u32 mpp_check_soc_cap(MppCtxType type, MppCodingType coding) {
  return type == MPP_CTX_ENC && (coding == MPP_VIDEO_CodingAVC || coding == MPP_VIDEO_CodingHEVC);
}

// osal/mpp_platform.c: the ID the block's register 0 reads on an RK3588's RKVENC0 (MPP's HWID_VEPU58X, 0x50603312: the register generator takes its vepu580
// path for it)
rk_u32 mpp_get_client_hw_id(rk_s32 client_type) { return client_type == VPU_CLIENT_RKVENC ? HWID_VEPU58X : 0; }

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
  (void)ctx;
  (void)fmt;
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

// dumps of the frames and packets that pass through the controller (files in MPP): off
MPP_RET mpp_ops_enc_get_pkt(MppDump info, MppPacket pkt) {
  (void)info;
  (void)pkt;
  return MPP_OK;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// the DMA window and MPP's buffers in it
//
// MPP allocates dma-bufs and passes their file descriptors around; the block's address registers hold "fd | offset << 10" and the kernel driver
// translates them. Here a buffer is a piece of the window and its "fd" is a small number (1..1023, 0 means none); the translation is in mpp_dev_ioctl.
// Buffers of an internal group that are released stay in the group and are handed out again, like MPP does.
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

static const VencHw *gHw;

void venc_shim_bind(const VencHw *hw) {
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

void venc_shim_window_stats(uint32_t *buffers, uint32_t *bytes, uint32_t *peak) {
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
MPP_RET mpp_dmabuf_sync_partial_begin(RK_S32 fd, RK_S32 ro, RK_U32 offset, RK_U32 length, const char *caller) {
  (void)fd; (void)ro; (void)offset; (void)length; (void)caller;
  return MPP_OK;
}
// the buffers are in the block's one address space already: nothing to map
MPP_RET mpp_buffer_attach_dev_f(const char *caller, MppBuffer buffer, MppDev dev) {
  (void)caller; (void)buffer; (void)dev;
  return MPP_OK;
}

uint64_t venc_shim_buffer_bus(MppBuffer buffer) {
  ShimBuffer *b = (ShimBuffer *)buffer;
  return b ? gWin.bus + b->winOff : 0;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// the device
//
// The HAL describes a job by writing register classes (control, base, rate control tables, parameters, rdo, scaling lists, osd), address-register offsets
// and read-backs, then sends and polls. The vendor kernel driver (mpp_rkvenc2.c) translates the address registers (fd | offset << 10 -> bus address) of
// a fixed list and runs the job. Here the same happens and the run itself is the embedder's.
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define kRegs VENC_REG_COUNT
#define kMaxOffs 64
#define kMaxReads 8
// the vendor kernel's trans_tbl_h264e_v2 / h265e_v2: the first 24 words of the picture class (registers 160..183) hold "fd | offset << 10"
#define kTransFirst 160
#define kTransCount 24

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
  VencJobObserver *observer;
  // dev_send's working set: kept off the kernel stack
  uint32_t sendRegs[kRegs];
  VencRun sendRuns[kRegs / 2 + 1];
  VencJobInfo sendInfo;
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

void venc_shim_set_observer(MppDev dev, VencJobObserver *observer) {
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

// osal/driver/mpp_device.c: a list of address offsets the HAL builds up and hands over in one call
MPP_RET mpp_dev_multi_offset_init(MppDevRegOffCfgs **cfgs, RK_S32 size) {
  RK_S32 cfg_size = (RK_S32)sizeof(MppDevRegOffCfgs) + size * (RK_S32)sizeof(MppDevRegOffsetCfg);
  MppDevRegOffCfgs *p = NULL;
  if (NULL == cfgs || size <= 0) {
    mpp_err_f("invalid pointer %p size %d\n", cfgs, size);
    return MPP_NOK;
  }
  p = mpp_calloc_size(MppDevRegOffCfgs, cfg_size);
  if (p) p->size = size;
  *cfgs = p;
  return p ? MPP_OK : MPP_NOK;
}

MPP_RET mpp_dev_multi_offset_deinit(MppDevRegOffCfgs *cfgs) {
  MPP_FREE(cfgs);
  return MPP_OK;
}

MPP_RET mpp_dev_multi_offset_reset(MppDevRegOffCfgs *cfgs) {
  if (cfgs) {
    memset(cfgs->cfgs, 0, (size_t)cfgs->count * sizeof(cfgs->cfgs[0]));
    cfgs->count = 0;
  }
  return MPP_OK;
}

MPP_RET mpp_dev_multi_offset_update(MppDevRegOffCfgs *cfgs, RK_S32 index, RK_U32 offset) {
  MppDevRegOffsetCfg *cfg;
  RK_S32 count, i;
  if (NULL == cfgs) return MPP_NOK;
  if (cfgs->count >= cfgs->size) {
    mpp_err_f("invalid cfgs count %d : %d\n", cfgs->count, cfgs->size);
    return MPP_NOK;
  }
  count = cfgs->count;
  cfg = cfgs->cfgs;
  for (i = 0; i < count; i++, cfg++) {
    if (cfg->reg_idx == (RK_U32)index) {
      cfg->offset = offset;
      return MPP_OK;
    }
  }
  cfg->reg_idx = (RK_U32)index;
  cfg->offset = offset;
  cfgs->count++;
  return MPP_OK;
}

static MPP_RET dev_send(ShimDev *d) {
  uint32_t *regs = d->sendRegs;
  VencRun *runs = d->sendRuns;
  unsigned nRuns = 0;
  VencJobInfo *infop = &d->sendInfo;
  memcpy(regs, d->regs, sizeof d->sendRegs);
  memset(infop, 0, sizeof *infop);
  for (unsigned t = 0; t < kTransCount; t++) {
    uint32_t idx = kTransFirst + t;
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
    if (off > b->winLen) {   // one past the end is legal (the stream top address of H.265)
      mpp_err("register %u: offset %u beyond buffer %u (%u bytes)\n", (unsigned)idx, (unsigned)off, (unsigned)id, (unsigned)b->winLen);
      return MPP_NOK;
    }
    regs[idx] = (uint32_t)(gWin.bus + b->winOff + off);
    if (infop->nReloc < VENC_MAX_RELOCS) {
      infop->reloc[infop->nReloc].reg = idx;
      infop->reloc[infop->nReloc].buf = id;
      infop->reloc[infop->nReloc].offset = off;
      infop->reloc[infop->nReloc].bufSize = (uint32_t)b->size;
      infop->nReloc++;
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
  infop->rawRegs = d->regs;
  infop->regs = regs;
  infop->runs = runs;
  infop->nRuns = nRuns;
  d->status = 0;
  if (d->observer && d->observer->before) d->observer->before(d->observer->ctx, infop);
  int rc = gHw->run(gHw->ctx, regs, runs, nRuns, &d->status);
  if (d->observer && d->observer->after) d->observer->after(d->observer->ctx, infop, rc, d->status);
  return rc == 0 ? MPP_OK : MPP_NOK;
}

// After the run: the read-backs the HAL asked for. The interrupt status (register 0x2c) is what the run returned; everything else (the status class: the
// length of the bit stream, the quantiser sum, the picture's statistics) is read from the block.
static MPP_RET dev_poll(ShimDev *d) {
  for (unsigned i = d->rdDone; i < d->nRd; i++) {
    uint32_t words = d->rd[i].size / 4;
    if (d->rd[i].offset == 0x2c) {
      memset(d->rd[i].p, 0, d->rd[i].size);
      if (words) *(uint32_t *)d->rd[i].p = d->status;
    } else {
      gHw->readRegs(gHw->ctx, d->rd[i].offset, (uint32_t *)d->rd[i].p, words);
    }
  }
  d->rdDone = d->nRd;
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
