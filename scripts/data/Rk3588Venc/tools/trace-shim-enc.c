// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// trace-shim-enc: the encoder twin of ../../Rk3588Vdec/tools/trace-shim.c. An LD_PRELOAD library that lets Rockchip's MPP (rockchip-linux/mpp, Apache-2.0,
// built unmodified on an x86 host) believe it runs on an RK3588 with a /dev/mpp_service and a DMA heap, and records every encode job MPP's vepu580 HAL
// submits: the register image (the five register classes MPP writes: control, base, rate control klut, parameters, rdo, scaling lists, osd), the buffers the
// address registers point to (with the content the CPU had put there, e.g. the input picture) and the address offsets. Nothing is encoded; the "hardware"
// answers "done, no error, a bit stream of 4000 bytes" at once.
//
// What the recording is for: the register words are what the RK3588 encoder (RKVENC0, "vepu580") expects, produced by the vendor's own encoder front end
// (parameter sets, rate control, reference management, register generator). The board side replays them (relocating the buffers) and checks what the block
// writes by decoding it.
//
// Environment: VENC_TRACE_DIR = output directory (created). Files: jobNNN.json (registers, relocations, offsets, rcb, info), bufB_jobNNN.bin (the content
// of buffer B at that job's submission, only when it changed since the last dump).
//
// The shape of the protocol follows MPP's osal/driver/mpp_service.c (the user side) and the vendor kernel's mpp_service.c / mpp_rkvdec2.c (the kernel side).
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAXFD 4096
#define NREGS 4096   // the vepu580 register file the HAL writes: control at 0, base at 160, rc klut at 1024, params at 1472, rdo at 2048, scl at 2176, osd at 3072 (words)

typedef struct { uint32_t cmd, flag, size, offset; uint64_t data_ptr; } ReqV1;
typedef struct { uint32_t reg_idx, offset; } RegOff;
typedef struct { uint32_t reg_idx, size; } RcbInfo;
typedef struct { uint32_t type, flag; uint64_t data; } InfoCfg;   // MppDevInfoCfg: {u32 type; u32 flag; u64 data}

enum { K_NONE = 0, K_MPP, K_HEAP, K_BUF };
static uint8_t gKind[MAXFD];
static int gBufId[MAXFD];       // unique buffer id for K_BUF fds
static size_t gBufSize[MAXFD];
static uint32_t gBufCrc[MAXFD]; // content hash at the last dump
static int gNextBuf = 1;

static int (*real_open)(const char *, int, ...);
static int (*real_close)(int);
static int (*real_access)(const char *, int);
static int (*real_ioctl)(int, unsigned long, ...);

// the job being assembled
static uint32_t gRegs[NREGS];
static uint8_t gRegSet[NREGS];
static RegOff gOffs[64]; static int gNOffs;
static RcbInfo gRcb[32]; static int gNRcb;
static InfoCfg gInfo[32]; static int gNInfo;
static int gJob;
static uint32_t gClientType;
static struct { void *p; uint32_t size, offset; } gRdQ[64]; static int gRdHead, gRdTail;   // read-back slots (MPP_CMD_SET_REG_READ) of the submitted jobs, oldest first
static int gBatchHasWrite;
static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;   // MPP's parser and HAL threads call ioctl at the same time (H.265 parses the next picture while the last one is submitted); a batch is handled whole

static void init(void) {
  if (real_open) return;
  real_open = dlsym(RTLD_NEXT, "open");
  real_close = dlsym(RTLD_NEXT, "close");
  real_access = dlsym(RTLD_NEXT, "access");
  real_ioctl = dlsym(RTLD_NEXT, "ioctl");
}

static int mkfdfile(const char *content, size_t len) {
  int fd = memfd_create("shim", 0);
  if (fd < 0) return -1;
  if (len && write(fd, content, len) != (ssize_t)len) {}
  lseek(fd, 0, SEEK_SET);
  return fd;
}

static int do_open(const char *path, int flags, mode_t mode) {
  init();
  if (path) {
    if (!strcmp(path, "/proc/device-tree/compatible")) {
      static const char compat[] = "rockchip,rk3588-evb1-v10\0rockchip,rk3588";
      return mkfdfile(compat, sizeof(compat));
    }
    if (!strcmp(path, "/dev/mpp_service")) {
      int fd = mkfdfile("", 0);
      if (fd >= 0 && fd < MAXFD) gKind[fd] = K_MPP;
      return fd;
    }
    if (!strncmp(path, "/dev/dma_heap/", 14)) {
      int fd = mkfdfile("", 0);
      if (fd >= 0 && fd < MAXFD) gKind[fd] = K_HEAP;
      return fd;
    }
    if (!strncmp(path, "/dev/ion", 8) || !strncmp(path, "/dev/dri/", 9) || !strncmp(path, "/dev/mpp", 8) || !strncmp(path, "/dev/vpu", 8) || !strncmp(path, "/dev/rkvdec", 11)) {
      errno = ENOENT;
      return -1;
    }
  }
  return real_open(path, flags, mode);
}

int open(const char *path, int flags, ...) {
  mode_t mode = 0;
  if (flags & (O_CREAT | O_TMPFILE)) { va_list ap; va_start(ap, flags); mode = va_arg(ap, mode_t); va_end(ap); }
  return do_open(path, flags, mode);
}
int open64(const char *path, int flags, ...) {
  mode_t mode = 0;
  if (flags & (O_CREAT | O_TMPFILE)) { va_list ap; va_start(ap, flags); mode = va_arg(ap, mode_t); va_end(ap); }
  return do_open(path, flags, mode);
}

int access(const char *path, int mode) {
  init();
  if ((!strcmp(path, "/proc/mpp_service/supports-cmd") || !strcmp(path, "/dev/mpp_service") || !strcmp(path, "/dev/dma_heap"))) return 0;
  return real_access(path, mode);
}

int close(int fd) {
  init();
  if (fd >= 0 && fd < MAXFD) { gKind[fd] = K_NONE; gBufId[fd] = 0; }
  return real_close(fd);
}

static uint32_t crc32_(const uint8_t *p, size_t n) {
  static uint32_t tab[256]; static int have;
  if (!have) { for (uint32_t i = 0; i < 256; i++) { uint32_t c = i; for (int k = 0; k < 8; k++) c = c & 1 ? 0xedb88320u ^ (c >> 1) : c >> 1; tab[i] = c; } have = 1; }
  uint32_t c = ~0u;
  while (n--) c = tab[(c ^ *p++) & 255] ^ (c >> 8);
  return ~c;
}

// the vendor kernel's trans_tbl_h264e_v2 / h265e_v2 (mpp_rkvenc2.c): the first 24 words of the picture class (registers 160..183: adr_src0..2, the
// reconstruction and reference pictures, colmv, bit stream, line buffers, ...) hold "fd | offset << 10"
#define TRANS_FIRST 160
#define TRANS_COUNT 24

static int all_zero(const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) if (p[i]) return 0;
  return 1;
}

static void finish_job(void) {
  char dir[512], name[640];
  const char *d = getenv("VENC_TRACE_DIR");
  if (!d) d = "venc-trace";
  snprintf(dir, sizeof dir, "%s", d);
  mkdir(dir, 0755);
  snprintf(name, sizeof name, "%s/job%03d.json", dir, gJob);
  FILE *f = fopen(name, "w");
  if (!f) { perror("trace json"); return; }
  uint32_t std = gRegs[0x300 / 4] & 1;   // ENC_PIC (0x300) bit 0: 0 H.264, 1 H.265
  fprintf(f, "{\n \"job\": %d, \"client_type\": %u, \"std\": %u,\n \"regs\": [", gJob, gClientType, std);
  int first = 1;
  for (int i = 0; i < NREGS; i++) if (gRegSet[i]) { fprintf(f, "%s[%d,%u]", first ? "" : ",", i, gRegs[i]); first = 0; }
  fprintf(f, "],\n \"reg_offsets\": [");
  for (int i = 0; i < gNOffs; i++) fprintf(f, "%s[%u,%u]", i ? "," : "", gOffs[i].reg_idx, gOffs[i].offset);
  fprintf(f, "],\n \"info\": [");
  for (int i = 0; i < gNInfo; i++) fprintf(f, "%s[%u,%u,%llu]", i ? "," : "", gInfo[i].type, gInfo[i].flag, (unsigned long long)gInfo[i].data);
  fprintf(f, "],\n \"relocs\": [");
  first = 1;
  for (int t = 0; t < TRANS_COUNT; t++) {
    uint32_t idx = TRANS_FIRST + t;
    if (!gRegSet[idx]) continue;
    uint32_t v = gRegs[idx];
    int fd = (int)(v & 0x3ff);
    uint32_t off = v >> 10;
    for (int i = 0; i < gNOffs; i++) if (gOffs[i].reg_idx == idx) off += gOffs[i].offset;
    if (fd <= 0 || fd >= MAXFD || gKind[fd] != K_BUF) {
      if (v) fprintf(stderr, "trace-shim-enc: job %d reg %u = %#x is not a known buffer fd\n", gJob, idx, v);
      continue;
    }
    fprintf(f, "%s\n  {\"reg\": %u, \"buf\": %d, \"off\": %u, \"size\": %zu}", first ? "" : ",", idx, gBufId[fd], off, gBufSize[fd]);
    first = 0;
    // dump the content of the buffer if it changed since the last dump and is not all zero
    void *p = mmap(NULL, gBufSize[fd], PROT_READ, MAP_SHARED, fd, 0);
    if (p != MAP_FAILED) {
      uint32_t c = crc32_(p, gBufSize[fd]);
      if (c != gBufCrc[fd] && !all_zero(p, gBufSize[fd])) {
        snprintf(name, sizeof name, "%s/buf%d_job%03d.bin", dir, gBufId[fd], gJob);
        FILE *b = fopen(name, "wb");
        if (b) { fwrite(p, 1, gBufSize[fd], b); fclose(b); }
      }
      gBufCrc[fd] = c;
      munmap(p, gBufSize[fd]);
    }
  }
  fprintf(f, "\n ]\n}\n");
  fclose(f);
  fprintf(stderr, "trace-shim-enc: job %d std %u recorded (%d offsets)\n", gJob, std, gNOffs);
  gJob++;
  memset(gRegSet, 0, sizeof gRegSet); memset(gRegs, 0, sizeof gRegs);
  gNOffs = gNRcb = gNInfo = 0;
}

static int mpp_request(ReqV1 *r) {
  uint32_t *u32 = (uint32_t *)(uintptr_t)r->data_ptr;
  switch (r->cmd) {
    case 0x000: *u32 = 1u << 16; return 0;                      // PROBE_HW_SUPPORT: RKVENC only (VPU_CLIENT_RKVENC = 16)
    case 0x001: *u32 = 0x50603312; return 0;                    // QUERY_HW_ID: what the block's ID register reads on the board (HWID_VEPU58X in MPP)
    case 0x002: {                                               // QUERY_CMD_SUPPORT: how many commands of the group are valid
      uint32_t butt = *u32;
      if (butt == 0x301) *u32 = 0x301;                          // poll group: HW_FINISH only (no per-slice IRQ polling)
      return 0;
    }
    case 0x100: gClientType = *u32; return 0;                   // INIT_CLIENT_TYPE
    case 0x101: case 0x102: return 0;
    case 0x200: {                                               // SET_REG_WRITE
      uint32_t first = r->offset / 4, n = r->size / 4;
      if (first + n > NREGS) { errno = EINVAL; return -1; }
      gBatchHasWrite = 1;
      memcpy(&gRegs[first], u32, r->size);
      for (uint32_t i = 0; i < n; i++) gRegSet[first + i] = 1;
      return 0;
    }
    case 0x201:                                                 // SET_REG_READ: the slot that is filled when the job is polled
      gRdQ[gRdTail++ % 64].p = u32; gRdQ[(gRdTail - 1) % 64].size = r->size; gRdQ[(gRdTail - 1) % 64].offset = r->offset;
      return 0;
    case 0x202: {                                               // SET_REG_ADDR_OFFSET
      RegOff *o = (RegOff *)u32; uint32_t n = r->size / sizeof(RegOff);
      for (uint32_t i = 0; i < n && gNOffs < 64; i++) gOffs[gNOffs++] = o[i];
      return 0;
    }
    case 0x203: {                                               // SET_RCB_INFO
      RcbInfo *o = (RcbInfo *)u32; uint32_t n = r->size / sizeof(RcbInfo);
      for (uint32_t i = 0; i < n && gNRcb < 32; i++) gRcb[gNRcb++] = o[i];
      return 0;
    }
    case 0x403: {                                               // SEND_CODEC_INFO
      InfoCfg *o = (InfoCfg *)u32; uint32_t n = r->size / sizeof(InfoCfg);
      for (uint32_t i = 0; i < n && gNInfo < 32; i++) gInfo[gNInfo++] = o[i];
      return 0;
    }
    case 0x300: case 0x301: {                                   // POLL_HW_FINISH / POLL_HW_IRQ: the oldest submitted job is done
      if (gRdHead != gRdTail) {
        int i = gRdHead++ % 64;
        uint32_t *p = gRdQ[i].p; memset(p, 0, gRdQ[i].size);
        if (gRdQ[i].offset == 0x2c && gRdQ[i].size >= 4) p[0] = 0x1;            // int_sta: encode done, no error bits
        if (gRdQ[i].offset == 4096 * 4 && gRdQ[i].size >= 4) p[0] = 4000;       // st.bs_lgth_l32: a bit stream length
      }
      return 0;
    }
    case 0x400: case 0x401: case 0x402: case 0x404: return 0;   // RESET_SESSION / TRANS_FD_TO_IOVA / RELEASE_FD / ERR_REF_HACK
    default: fprintf(stderr, "trace-shim: unknown mpp_service cmd %#x\n", r->cmd); return 0;
  }
}

int ioctl(int fd, unsigned long req, ...) {
  init();
  va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);
  if (fd >= 0 && fd < MAXFD) {
    if (gKind[fd] == K_HEAP) {
      // DMA_HEAP_IOCTL_ALLOC: struct { u64 len; u32 fd; u32 fd_flags; u64 heap_flags; }
      struct { uint64_t len; uint32_t fd; uint32_t fd_flags; uint64_t heap_flags; } *a = arg;
      int b = memfd_create("dmabuf", 0);
      if (b < 0 || ftruncate(b, (off_t)a->len) < 0) return -1;
      if (b < MAXFD) { gKind[b] = K_BUF; gBufId[b] = gNextBuf++; gBufSize[b] = a->len; gBufCrc[b] = 0; }
      a->fd = (uint32_t)b;
      return 0;
    }
    if (gKind[fd] == K_MPP) {
      ReqV1 *r = (ReqV1 *)arg;
      int rc = 0;
      pthread_mutex_lock(&gLock);
      gBatchHasWrite = 0;
      for (;;) {
        rc = mpp_request(r);
        if (rc) { pthread_mutex_unlock(&gLock); return rc; }
        if (!(r->flag & 1) || (r->flag & 2)) break;
        r++;
      }
      if (gBatchHasWrite) finish_job();   // a submission: the register image is complete now (the next one may follow before the first is polled)
      pthread_mutex_unlock(&gLock);
      return 0;
    }
    if (gKind[fd] == K_BUF) return 0;   // DMA_BUF_IOCTL_SYNC, DMA_BUF_SET_NAME, ...
  }
  return real_ioctl(fd, req, arg);
}
