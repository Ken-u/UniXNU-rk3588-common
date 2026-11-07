// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The video decoder service of iPadOS on the rkvdec2 block: an IOService named AppleD5500 (what H264H8.videodecoder's AppleD5500OpenConnection looks for with
// IOServiceMatching("AppleD5500")) whose user client (connection type 1) answers the selectors of the original kext's user client, so that VideoToolbox's own
// hardware decoder plug-in drives this block. The interface is what the plug-in's code shows (evidence/host-d5500-abi-20261107.txt of UniXNU-orangepi5, and
// the "AppleD5500 interface" section of docs/media.md):
//
//   sel  struct in  struct out   name
//    0    0x110      0x48        CreateDecoder   width, height, +8 (NAL length size << 24 | size of the extradata), ..., +0xa0 the IOSurface id of the bitstream
//                                                ring buffer. The extradata (the avcC record, or the hvcC record of H.265: the parameter sets) is at offset 0 of the
//                                                ring buffer; D5500Stream.h tells the codec from it (and from the first access unit when it is Annex B).
//                                                Out: +0 the picture buffers the decoder needs (the plug-in's pool is this + 3), +8 the decoder number,
//                                                +0x10/+0x14 width and height in macroblocks, +0x30 chroma_format_idc, +0x34/+0x38 bit depth - 8 of luma /
//                                                chroma (the plug-in picks the pixel format of its buffers from these; zeros make it ask for 'L008')
//    1    0x4        0x84        DestroyDecoder
//    2    0x858      0x410       DecodeFrameFig  +0 bytes, +4 sequence number, +8 output picture slot, +0x10 offset of the bytes in the ring, +0x14 offset of
//                                                the result records in the ring; the answer comes asynchronously
//    3    0x8        0x4         MapPixelBuffer  slot, IOSurface id (slot 0x80 is the ring buffer)
//    4    0x4        0x4         UnmapPixelBuffer
//    5    0x18       0x4         SetCryptSession (FairPlay: refused cleanly)
//    6    0x4        0x4         (reads a word of the service)
//    7    0x10       0x4         SetCallback (asynchronous: frames come back as IOKit asynchronous results)
//
// How a frame goes: the plug-in writes the access unit into the ring buffer (an IOSurface both sides map), calls DecodeFrameFig and goes on; the call is queued
// for a thread of the client's own, which hands the bytes to the decoder core (core/: Rockchip's MPP H.264 / H.265 parser and rkvdec2 register generator), copies the
// picture the job made into the output slot's IOSurface (NV12), writes a 0x410-byte result record into the ring buffer after the bitstream area and sends the
// completion (sequence number, status, record index) through the callback. The plug-in's own bookkeeping (what the result record has to say, which buffer
// references the "kernel" holds) is in the comments at the record writer below.
//
// Boot argument rk3588-vdec=7 registers the service; rk3588-vdec-d5500-log=N is how much of the traffic is printed (default 1: the first calls of each kind).
// H.265: the plug-in registers its HEVC decoders (H264H8Register -> AppleD5500CheckPlatform) only if the service's property HEVCSupported is true, which this
// service sets; rk3588-vdec-hevc=0 sets it false (and refuses H.265 in CreateDecoder), which is how the plug-in sees a decoder without H.265.
// The copy of the picture into the output surface is done by the RGA3 block (scripts/data/Rk3588Rga) when that driver is linked and ready (rk3588-rga=5):
// the decoder's window and each surface are mapped into its address space, and a copy it cannot or does not do is done by the CPU as before.
// rk3588-vdec-d5500-rga=0 keeps the copy on the CPU.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <kern/thread.h>
#include <libkern/c++/OSMetaClass.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

#include "../Rk3588Rga/Rk3588Rga.h"
#include "D5500Stream.h"
#include "Rk3588Vdec.h"
#include "VdecTrace.h"
#include "core/VdecCore.h"
#include "core/VdecPlatform.h"

extern "C" void proc_selfname(char *buf, int size);

// The RGA3 driver is optional (a kernel without it still links): its entry points are weak here.
extern "C" int rk3588_rga_ready(void) __attribute__((weak));
extern "C" uint32_t rk3588_rga_map_physical(uint64_t phys, uint64_t bytes, int writable) __attribute__((weak));
extern "C" void rk3588_rga_unmap(uint32_t iova, uint64_t bytes) __attribute__((weak));
extern "C" int rk3588_rga_copy_nv12(const struct Rk3588RgaPicture *src, uint32_t srcX, uint32_t srcRow, const struct Rk3588RgaPicture *dst, uint32_t width, uint32_t height,
                                    uint32_t *us) __attribute__((weak));
uint32_t rk3588_rga_map_memory(IOMemoryDescriptor *memory, uint64_t length) __attribute__((weak));

namespace {

using namespace d5500;

enum : uint32_t {
  kSelCreateDecoder = 0, kSelDestroyDecoder = 1, kSelDecodeFrame = 2, kSelMapPixelBuffer = 3, kSelUnmapPixelBuffer = 4, kSelSetCryptSession = 5,
  kSelSix = 6, kSelSetCallback = 7, kSelCount = 8,
};

const char *const kSelName[kSelCount] = {"CreateDecoder", "DestroyDecoder", "DecodeFrameFig", "MapPixelBuffer", "UnmapPixelBuffer", "SetCryptSession", "sel6", "SetCallback"};

// The structure sizes the kext's table checks (in, out).
const uint32_t kSelIn[kSelCount] = {0x110, 0x4, 0x858, 0x8, 0x4, 0x18, 0x4, 0x10};
const uint32_t kSelOut[kSelCount] = {0x48, 0x84, 0x410, 0x4, 0x4, 0x4, 0x4, 0x4};

constexpr uint32_t kSlots = 256;          // slots the plug-in can name: small numbers for the pictures, 0x80 for the ring buffer
constexpr uint32_t kRecordSize = 0x410;   // the result record the plug-in reads back
constexpr uint32_t kRecordCount = 32;     // of the 63 that fit in the 64 KiB after the bitstream area: round robin, far more than can be in flight
constexpr uint32_t kQueueDepth = 32;
constexpr uint32_t kMaxStream = 4u << 20;

int gLog = 1;
bool gRgaWanted = true;   // rk3588-vdec-d5500-rga: false keeps the copy of the pictures on the CPU
bool gHevc = true;        // rk3588-vdec-hevc: 0 makes "HEVCSupported" false (the plug-in then registers no H.265 decoder) and refuses an H.265 stream

// The block can be used: its driver is in the kernel, wanted, and ready (it is only after its own power-up and self-test).
bool rgaUsable() { return gRgaWanted && rk3588_rga_ready && rk3588_rga_map_memory && rk3588_rga_copy_nv12 && rk3588_rga_ready() != 0; }

// Print the non-zero 32-bit words of a structure as offset:value pairs, a line at a time.
void printWords(const char *what, const void *data, size_t size) {
  const uint32_t *w = static_cast<const uint32_t *>(data);
  char line[220];
  size_t length = 0;
  unsigned count = 0;
  for (size_t i = 0; i < size / 4; ++i) {
    if (!w[i]) continue;
    ++count;
    length += static_cast<size_t>(snprintf(line + length, sizeof line - length, " %zx:%x", i * 4, w[i]));
    if (length > sizeof line - 24) {
      printf("rk3588-vdec: d5500: %s%s\n", what, line);
      length = 0;
    }
  }
  if (length || !count) printf("rk3588-vdec: d5500: %s%s (%u non-zero words of %zu)\n", what, length ? line : " all zero", count, size / 4);
}

uint64_t nowUs() {
  clock_sec_t s;
  clock_usec_t u;
  clock_get_system_microtime(&s, &u);
  return static_cast<uint64_t>(s) * 1000000ull + u;
}

// ---- IOSurface access.
//
// The IOSurface kext keeps its surfaces in a table indexed by id inside the IOCoreSurfaceRoot service; the original kernel's own accessors (the display driver
// binds them: NativeLegacyDisplay's native_legacy_surface_*) read the fields below. Offsets of the original 21H461 kext: surface +16 id, +48 the memory
// descriptor, +88 width, +96 height, +104 bytes per row, +128 pixel format, +144 allocation size, +152 plane count, +384/+392 the plane array (56 bytes
// each: width, height, ..., bytes per row at +32, size at +40); root +208 the table, +216 its entry count, +312 the lock (an IOLock *).
constexpr size_t kRootTable = 208, kRootCount = 216, kRootLock = 312;
constexpr size_t kSurfId = 16, kSurfMemory = 48, kSurfWidth = 88, kSurfHeight = 96, kSurfStride = 104, kSurfFormat = 128, kSurfAlloc = 144, kSurfPlanes = 152,
                 kSurfPlaneArray = 384, kSurfPlaneCount = 392, kPlaneBytes = 56;

template <typename T> T field(const void *base, size_t offset) {
  T value;
  memcpy(&value, static_cast<const uint8_t *>(base) + offset, sizeof value);
  return value;
}

IOService *surfaceRoot() {
  static IOService *root;
  if (root) return root;
  OSDictionary *matching = IOService::nameMatching("IOCoreSurfaceRoot");
  root = matching ? IOService::waitForMatchingService(matching, 2000000000ULL) : nullptr;   // kept for good
  OSSafeReleaseNULL(matching);
  return root;
}

struct Surface {
  OSObject *object;         // retained
  IOMemoryDescriptor *memory;   // retained, prepared
  IOMemoryMap *map;         // the kernel mapping
  uint8_t *kva;
  uint64_t length;
  uint32_t id, format, planes;
  uint64_t width, height, stride, allocation;
  uint32_t rga;             // the IO address of byte 0 of the memory in the RGA3's address space, 0 when it is not mapped there
  struct Plane {
    uint64_t stride, offset, height;
  } plane[2];

  bool valid() const { return object != nullptr; }
  void release();
  bool open(uint32_t surfaceId, bool dump);
};

void Surface::release() {
  if (rga && rk3588_rga_unmap) rk3588_rga_unmap(rga, length);
  if (map) map->release();
  if (memory) {
    (void)memory->complete();
    memory->release();
  }
  if (object) object->release();
  *this = Surface();
}

// Looks the surface up, takes a reference and maps its memory into the kernel.
bool Surface::open(uint32_t surfaceId, bool dump) {
  *this = Surface();
  IOService *root = surfaceRoot();
  if (!root || !surfaceId) return false;
  const uint8_t *rootBytes = reinterpret_cast<const uint8_t *>(root);
  IOLock *lock = field<IOLock *>(rootBytes, kRootLock);
  if (lock) IOLockLock(lock);
  const uint32_t count = field<uint32_t>(rootBytes, kRootCount);
  OSObject *const *table = field<OSObject *const *>(rootBytes, kRootTable);
  OSObject *surface = surfaceId < count && table ? table[surfaceId] : nullptr;
  if (surface && field<uint32_t>(surface, kSurfId) == surfaceId && surface->metaCast("IOSurface")) surface->retain();
  else surface = nullptr;
  if (lock) IOLockUnlock(lock);
  if (!surface) return false;
  IOMemoryDescriptor *descriptor = field<IOMemoryDescriptor *>(surface, kSurfMemory);
  if (!descriptor || !descriptor->metaCast("IOMemoryDescriptor")) {
    surface->release();
    return false;
  }
  object = surface;
  descriptor->retain();
  memory = descriptor;
  if (memory->prepare() != kIOReturnSuccess) {
    memory->release();
    memory = nullptr;
    release();
    return false;
  }
  map = memory->map(kIOMapAnywhere);
  if (!map || !map->getVirtualAddress()) {
    release();
    return false;
  }
  kva = reinterpret_cast<uint8_t *>(map->getVirtualAddress());
  length = memory->getLength();
  id = surfaceId;
  width = field<uint64_t>(surface, kSurfWidth);
  height = field<uint64_t>(surface, kSurfHeight);
  stride = field<uint64_t>(surface, kSurfStride);
  format = field<uint32_t>(surface, kSurfFormat);
  allocation = field<uint64_t>(surface, kSurfAlloc);
  planes = field<uint32_t>(surface, kSurfPlanes);
  if (rgaUsable()) rga = rk3588_rga_map_memory(memory, length);   // 0 when it does not fit: this surface is then copied to by the CPU
  const uint8_t *planeArray = field<const uint8_t *>(surface, kSurfPlaneArray);
  const uint32_t planeCount = field<uint32_t>(surface, kSurfPlaneCount);
  plane[0] = {stride, 0, height};
  plane[1] = {stride, stride * height, height / 2};
  if (planes == 2 && planeArray && planeCount >= 2 && planeCount <= 4) {
    // bytes per row at +32, height at +8; where the second plane starts is either of the words at +16 and +24 (the one that is a plausible offset)
    const uint64_t p0Stride = field<uint64_t>(planeArray, 32), p1Stride = field<uint64_t>(planeArray, kPlaneBytes + 32);
    if (p0Stride) plane[0].stride = p0Stride;
    if (p1Stride) plane[1].stride = p1Stride;
    const uint64_t p0Height = field<uint64_t>(planeArray, 8), p1Height = field<uint64_t>(planeArray, kPlaneBytes + 8);
    if (p0Height) plane[0].height = p0Height;
    if (p1Height) plane[1].height = p1Height;
    // where a plane starts: the word at +16 of its element (the board showed 0x1000 for luma and 0x14000 for chroma of a 320x240 surface: the first plane
    // does not start at the beginning of the allocation); +24 repeats it
    for (unsigned k = 0; k < 2; ++k) {
      const uint64_t v = field<uint64_t>(planeArray, k * kPlaneBytes + 16);
      if (v < length && (k == 0 || v >= plane[0].offset + plane[0].stride * plane[0].height)) plane[k].offset = v;
    }
  }
  if (dump) {
    printf("rk3588-vdec: d5500: IOSurface %u: %llux%llu format '%c%c%c%c' planes %u stride %llu allocation %llu memory %llu bytes; plane 0: stride %llu height %llu, plane 1: offset %llu stride %llu height %llu\n",
           surfaceId, static_cast<unsigned long long>(width), static_cast<unsigned long long>(height), static_cast<char>(format >> 24), static_cast<char>(format >> 16),
           static_cast<char>(format >> 8), static_cast<char>(format), planes, static_cast<unsigned long long>(stride), static_cast<unsigned long long>(allocation),
           static_cast<unsigned long long>(length), static_cast<unsigned long long>(plane[0].stride), static_cast<unsigned long long>(plane[0].height),
           static_cast<unsigned long long>(plane[1].offset), static_cast<unsigned long long>(plane[1].stride), static_cast<unsigned long long>(plane[1].height));
    if (planeArray && planeCount && planeCount <= 4) {
      for (uint32_t p = 0; p < planeCount; ++p) {
        const uint64_t *w = reinterpret_cast<const uint64_t *>(planeArray + p * kPlaneBytes);
        printf("rk3588-vdec: d5500: IOSurface %u plane element %u: %llx %llx %llx %llx %llx %llx %llx\n", surfaceId, p, static_cast<unsigned long long>(w[0]),
               static_cast<unsigned long long>(w[1]), static_cast<unsigned long long>(w[2]), static_cast<unsigned long long>(w[3]), static_cast<unsigned long long>(w[4]),
               static_cast<unsigned long long>(w[5]), static_cast<unsigned long long>(w[6]));
      }
    }
  }
  return true;
}

// ---- one client's state

struct Job {
  uint32_t seq, size, offset, slot, records;
  uint64_t queuedUs;   // when DecodeFrame put it in the queue
};

// Where the time of the frames goes (microseconds, summed over the frames of a decoder; printed when it is destroyed).
struct Profile {
  uint64_t frames, queue, prep, core, pictureCb, flushPre, copy, flushPost, probe, finish;
  uint64_t blockJobs, blockRun, blockWall;   // from the block's job runner, over the same frames
  uint64_t rgaCopies, rgaUs, rgaFailed, cpuCopies;   // pictures copied by the RGA3 (and the time it ran), pictures it failed on, pictures the CPU copied
};

struct Session {
  IOLock *lock;
  Surface slots[kSlots];
  Surface ring;
  VdecCore *core;
  VdecHw hw;
  uint32_t rgaSource;   // the IO address of the decoder's window in the RGA3's address space (read-only), 0 when it is not mapped
  uint32_t width, height;
  Job queue[kQueueDepth];
  uint32_t head, count, nextRecord;
  bool stop, busy, workerStarted, workerExited;
  uint8_t *scratch;
  size_t scratchSize;
  uint32_t jobsDone, jobsFailed;
  Profile prof;
  // the stream: its parameter sets (Annex B), put in front of the first access unit and again after the core was reset; the length of the NAL length fields
  uint8_t params[1024];
  size_t paramsLength;
  bool needParams;
  uint32_t nalSize;
  int codec;   // kVdecH264 or kVdecH265, from the extradata at CreateDecoder
  // set while a job runs, for the decoder core's callback
  const Job *current;
  uint32_t picturesInJob;
  bool pictureCopied;
  uint8_t event[4];   // addresses used as wait events
};

// The window of the decoder as a source of the RGA3: mapped read-only into its address space the first time it is wanted with the block ready, and kept.
IOLock *gRgaLock;
uint32_t gRgaWindow;
uint64_t gRgaWindowBus;
uint32_t gRgaWindowBytes;
bool gRgaWindowTried;

uint32_t rgaWindow(const VdecHw &hw) {
  if (!gRgaLock || !rgaUsable() || !rk3588_rga_map_physical) return 0;
  IOLockLock(gRgaLock);
  if (!gRgaWindow && !gRgaWindowTried) {
    gRgaWindowTried = true;
    gRgaWindow = rk3588_rga_map_physical(hw.dmaBus, hw.dmaBytes, 0);
    gRgaWindowBus = hw.dmaBus;
    gRgaWindowBytes = hw.dmaBytes;
    printf("rk3588-vdec: d5500: the decoder's window (%#llx, %u bytes) %s the RGA3's address space%s\n", static_cast<unsigned long long>(hw.dmaBus), hw.dmaBytes,
           gRgaWindow ? "is in" : "does not fit in", gRgaWindow ? "" : "; the CPU copies the pictures");
  }
  const uint32_t iova = gRgaWindow && gRgaWindowBus == hw.dmaBus && gRgaWindowBytes == hw.dmaBytes ? gRgaWindow : 0;
  IOLockUnlock(gRgaLock);
  return iova;
}

// Copies one decoded picture (NV12 in the DMA window) into an IOSurface of NV12; the surface may be larger or smaller than the picture. The RGA3 does it when
// it can (the surface is mapped in its address space, the sizes and offsets are in its limits); *byBlock says whether it did and *blockUs how long it ran.
bool copyPicture(const Session *s, const VdecPicture *pic, const Surface &dst, bool *byBlock, uint32_t *blockUs, bool *blockFailed) {
  *byBlock = false;
  *blockUs = 0;
  *blockFailed = false;
  const bool nv12 = dst.format == 0x34323076 /* '420v' */ || dst.format == 0x34323066 /* '420f' */;
  if (!nv12 || dst.planes != 2 || !dst.kva) return false;
  const uint64_t width = pic->width < dst.width ? pic->width : dst.width;
  const uint64_t rows = pic->height < dst.height ? pic->height : dst.height;
  const uint64_t lumaEnd = dst.plane[0].offset + dst.plane[0].stride * rows, chromaEnd = dst.plane[1].offset + dst.plane[1].stride * ((rows + 1) / 2);
  if (width > dst.plane[0].stride || width > dst.plane[1].stride || lumaEnd > dst.length || chromaEnd > dst.length || lumaEnd > dst.plane[1].offset) return false;
  if (s->rgaSource && dst.rga && dst.plane[0].stride == dst.plane[1].stride && pic->data >= s->hw.dmaVirt) {
    const uint64_t at = static_cast<uint64_t>(pic->data - s->hw.dmaVirt), chromaAt = at + static_cast<uint64_t>(pic->hstride) * pic->vstride;
    if (chromaAt + static_cast<uint64_t>(pic->hstride) * pic->vstride / 2 <= s->hw.dmaBytes && dst.length < (1ull << 28)) {
      const Rk3588RgaPicture source = {static_cast<uint32_t>(s->rgaSource + at), static_cast<uint32_t>(s->rgaSource + chromaAt), pic->hstride};
      const Rk3588RgaPicture target = {static_cast<uint32_t>(dst.rga + dst.plane[0].offset), static_cast<uint32_t>(dst.rga + dst.plane[1].offset),
                                       static_cast<uint32_t>(dst.plane[0].stride)};
      const int rc = rk3588_rga_copy_nv12(&source, pic->cropLeft, pic->cropTop, &target, static_cast<uint32_t>(width), static_cast<uint32_t>(rows), blockUs);
      if (rc == 0) {
        *byBlock = true;
        return true;
      }
      *blockFailed = rc != -1;   // -1: not a copy for the block (its limits); the CPU does it without a word. -2: the block failed (and was reset).
    }
  }
  const uint8_t *luma = pic->data + static_cast<size_t>(pic->cropTop) * pic->hstride + pic->cropLeft;
  for (uint64_t y = 0; y < rows; ++y)
    memcpy(dst.kva + dst.plane[0].offset + y * dst.plane[0].stride, luma + static_cast<size_t>(y) * pic->hstride, static_cast<size_t>(width));
  const uint8_t *chroma = pic->data + static_cast<size_t>(pic->hstride) * pic->vstride + static_cast<size_t>(pic->cropTop / 2) * pic->hstride + (pic->cropLeft & ~1u);
  for (uint64_t y = 0; y < (rows + 1) / 2; ++y)
    memcpy(dst.kva + dst.plane[1].offset + y * dst.plane[1].stride, chroma + static_cast<size_t>(y) * pic->hstride, static_cast<size_t>(width));
  return true;
}

void pictureDecoded(void *ctx, const VdecPicture *pic) {
  Session *s = static_cast<Session *>(ctx);
  ++s->picturesInJob;
  if (!s->current) return;
  const uint64_t t0 = nowUs();
  IOLockLock(s->lock);
  const Surface &dst = s->slots[s->current->slot];
  // first push out whatever the cache may still hold dirty for these pages (the buffer's own initialisation): evicted later it would overwrite the
  // picture (vt18: the first picture of a session came out with random damage, the core's buffer and the CRC of the later pictures were right). The cause
  // was the batch path of the pmap, which made the pages of a fresh write-combined surface non-cacheable without taking them out of the cache
  // (patches/xnu/rk3588-wc-batch-flush.patch); with that fixed this is only a safeguard for surfaces the plug-in leaves cacheable.
  if (dst.valid()) (void)dst.memory->performOperation(kIOMemoryIncoherentIOFlush, 0, dst.length);
  const uint64_t t1 = nowUs();
  bool byBlock = false, blockFailed = false;
  uint32_t blockUs = 0;
  const bool ok = dst.valid() && copyPicture(s, pic, dst, &byBlock, &blockUs, &blockFailed);
  const uint64_t t2 = nowUs();
  // the reader maps the surface in its own way (the cache mode is the surface's): push what was written out to memory
  if (ok) (void)dst.memory->performOperation(kIOMemoryIncoherentIOFlush, 0, dst.length);
  const uint64_t t3 = nowUs();
  if (ok && gLog >= 2 && s->jobsDone < 12) {
    // a probe (rk3588-vdec-d5500-log=2 and up; it reads the picture twice a byte at a time through uncached memory, 25 ms at 320x240 and 300 ms at 720p, 98 % of a frame's time): the CRC of the picture as the core made it and as it stands in the surface (NV12, visible rectangle: what the test tool computes)
    uint32_t crcSource = 0, crcSurface = 0;
    const uint64_t width = pic->width < dst.width ? pic->width : dst.width, rows = pic->height < dst.height ? pic->height : dst.height;
    const uint8_t *luma = pic->data + static_cast<size_t>(pic->cropTop) * pic->hstride + pic->cropLeft;
    const uint8_t *chroma = pic->data + static_cast<size_t>(pic->hstride) * pic->vstride + static_cast<size_t>(pic->cropTop / 2) * pic->hstride + (pic->cropLeft & ~1u);
    for (uint64_t y = 0; y < rows; ++y) {
      crcSource = vdec::crc32(luma + static_cast<size_t>(y) * pic->hstride, static_cast<size_t>(width), crcSource);
      crcSurface = vdec::crc32(dst.kva + dst.plane[0].offset + y * dst.plane[0].stride, static_cast<size_t>(width), crcSurface);
    }
    for (uint64_t y = 0; y < rows / 2; ++y) {
      crcSource = vdec::crc32(chroma + static_cast<size_t>(y) * pic->hstride, static_cast<size_t>(width), crcSource);
      crcSurface = vdec::crc32(dst.kva + dst.plane[1].offset + y * dst.plane[1].stride, static_cast<size_t>(width), crcSurface);
    }
    printf("rk3588-vdec: d5500: picture of job %u: crc %08x in the core's buffer, %08x in the surface, flags %#x (cpu %d)\n", s->jobsDone + 1, crcSource, crcSurface, pic->flags, int(cpu_number()));
  }
  const uint64_t t4 = nowUs();
  IOLockUnlock(s->lock);
  s->prof.flushPre += t1 - t0;
  s->prof.copy += t2 - t1;
  if (ok && byBlock) {
    ++s->prof.rgaCopies;
    s->prof.rgaUs += blockUs;
  } else if (ok) {
    ++s->prof.cpuCopies;
  }
  if (blockFailed) ++s->prof.rgaFailed;
  s->prof.flushPost += t3 - t2;
  s->prof.probe += t4 - t3;
  s->prof.pictureCb += t4 - t0;
  if (ok) s->pictureCopied = true;
  if (!ok || gLog >= 2)
    printf("rk3588-vdec: d5500: picture %ux%u (stride %u, crop %u,%u) pts %lld poc %u flags %#x -> slot %u: %s\n", pic->width, pic->height, pic->hstride, pic->cropLeft,
           pic->cropTop, static_cast<long long>(pic->pts), pic->poc, pic->flags, s->current->slot, ok ? "copied" : "NOT COPIED");
}

}  // namespace

class AppleD5500 : public IOService {
  OSDeclareDefaultStructors(AppleD5500)
 public:
  using IOService::newUserClient;
  IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) override;
};
OSDefineMetaClassAndStructors(AppleD5500, IOService)

class AppleD5500UserClient : public IOUserClient {
  OSDeclareDefaultStructors(AppleD5500UserClient)
  char ownerName[17] = {};
  task_t ownerTask = nullptr;
  io_user_reference_t asyncReference[8] = {};
  bool haveCallback = false;
  unsigned calls[kSelCount] = {};
  Session *session = nullptr;

  static void worker(void *self, wait_result_t);
  void runJob(const Job &job);
  void finish(const Job &job, uint32_t status, uint32_t index, uint32_t bufferSlot);
  void drain();
  void tearDownDecoder();
  void stopWorker();

 public:
  using IOUserClient::initWithTask;
  using IOUserClient::clientMemoryForType;
  bool initWithTask(task_t owningTask, void *securityToken, UInt32 type) override;
  IOReturn clientClose() override;
  void free() override;
  IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) override;
  IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *arguments, IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) override;
};
OSDefineMetaClassAndStructors(AppleD5500UserClient, IOUserClient)

IOReturn AppleD5500::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) {
  if (!handler) return kIOReturnBadArgument;
  if (type != 1) {
    printf("rk3588-vdec: d5500: open with connection type %u refused\n", static_cast<unsigned>(type));
    return kIOReturnUnsupported;
  }
  AppleD5500UserClient *client = OSTypeAlloc(AppleD5500UserClient);
  if (!client) return kIOReturnNoMemory;
  bool attached = false;
  if (!client->initWithTask(owningTask, securityID, type) || !(attached = client->attach(this)) || !client->start(this)) {
    if (attached) client->detach(this);
    client->release();
    return kIOReturnError;
  }
  *handler = client;
  return kIOReturnSuccess;
}

bool AppleD5500UserClient::initWithTask(task_t owningTask, void *securityToken, UInt32 type) {
  if (!IOUserClient::initWithTask(owningTask, securityToken, type)) return false;
  proc_selfname(ownerName, sizeof ownerName);
  ownerTask = owningTask;
  session = static_cast<Session *>(IOMallocZero(sizeof(Session)));
  if (!session) return false;
  session->lock = IOLockAlloc();
  if (!session->lock) return false;
  printf("rk3588-vdec: d5500: %s opens the service (connection type %u)\n", ownerName, static_cast<unsigned>(type));
  return true;
}

void AppleD5500UserClient::free() {
  if (session) {
    if (session->lock) IOLockFree(session->lock);
    IOFree(session, sizeof(Session));
    session = nullptr;
  }
  IOUserClient::free();
}

IOReturn AppleD5500UserClient::clientMemoryForType(UInt32 type, IOOptionBits *, IOMemoryDescriptor **) {
  printf("rk3588-vdec: d5500: %s asks for client memory of type %u\n", ownerName, static_cast<unsigned>(type));
  return kIOReturnUnsupported;
}

// ---- the thread of the client: one job at a time through the decoder core

void AppleD5500UserClient::worker(void *self, wait_result_t) {
  AppleD5500UserClient *client = static_cast<AppleD5500UserClient *>(self);
  Session *s = client->session;
  IOLockLock(s->lock);
  for (;;) {
    while (!s->stop && !s->count) IOLockSleep(s->lock, &s->event[0], THREAD_UNINT);
    if (s->stop) break;
    const Job job = s->queue[s->head];
    s->head = (s->head + 1) % kQueueDepth;
    --s->count;
    s->busy = true;
    IOLockUnlock(s->lock);
    client->runJob(job);
    IOLockLock(s->lock);
    s->busy = false;
    IOLockWakeup(s->lock, &s->event[1], true);
  }
  s->workerExited = true;
  IOLockWakeup(s->lock, &s->event[2], true);
  IOLockUnlock(s->lock);
}

// The result record the plug-in reads after the completion (AppleD5500DecodeFrameResponse): at +6 a count and at +8 that many 16-bit picture slots the "kernel"
// let go of (the plug-in drops its reference of each: the one a decoder holds while it may still need the picture as a reference; this driver keeps its
// references in the decoder core's own buffers, so the output slot of the job is let go at once), +0x88 / +0x8c the error and the decryption error (0),
// +0x94 the slot of the picture to show (the one this job decoded: VideoToolbox puts frames in display order by itself), +0x9c / +0xa0 the offset and the size of
// the bitstream the job used (the plug-in gives that part of the ring buffer back).
void AppleD5500UserClient::finish(const Job &job, uint32_t status, uint32_t index, uint32_t bufferSlot) {
  Session *s = session;
  IOLockLock(s->lock);
  if (s->ring.valid() && job.records + (index + 1) * kRecordSize <= s->ring.length) {
    uint8_t *record = s->ring.kva + job.records + static_cast<size_t>(index) * kRecordSize;
    bzero(record, kRecordSize);
    const uint16_t one = 1, slot = static_cast<uint16_t>(job.slot);
    memcpy(record + 6, &one, sizeof one);
    memcpy(record + 8, &slot, sizeof slot);
    memcpy(record + 0x94, &bufferSlot, sizeof bufferSlot);
    memcpy(record + 0x9c, &job.offset, sizeof job.offset);
    memcpy(record + 0xa0, &job.size, sizeof job.size);
    __asm__ volatile("dmb sy" ::: "memory");
    (void)s->ring.memory->performOperation(kIOMemoryIncoherentIOFlush, job.records + static_cast<IOByteCount>(index) * kRecordSize, kRecordSize);
  } else {
    printf("rk3588-vdec: d5500: no room for the result record of frame %u (ring %llu bytes, records at %u, index %u)\n", job.seq,
           static_cast<unsigned long long>(s->ring.length), job.records, index);
  }
  IOLockUnlock(s->lock);
  if (haveCallback) {
    io_user_reference_t args[3] = {job.seq, status, index};
    const IOReturn kr = sendAsyncResult64(asyncReference, kIOReturnSuccess, args, 3);
    if (kr != kIOReturnSuccess || gLog >= 2) printf("rk3588-vdec: d5500: completion of frame %u (status %u, record %u): %#x\n", job.seq, status, index, kr);
  } else {
    printf("rk3588-vdec: d5500: frame %u is done and nobody asked to hear about it\n", job.seq);
  }
}

void AppleD5500UserClient::runJob(const Job &job) {
  Session *s = session;
  const uint32_t index = s->nextRecord++ % kRecordCount;
  uint32_t status = 0;
  const uint64_t startUs = nowUs();
  s->prof.queue += startUs - job.queuedUs;
  unsigned nals = 0;
  uint8_t types[8] = {};
  bool annexB = false;
  size_t length = 0;
  VdecStats before = {}, after = {};

  // 1. the bytes, from the ring buffer into the scratch buffer, framed for the parser
  IOLockLock(s->lock);
  if (s->ring.valid() && s->core && static_cast<uint64_t>(job.offset) + job.size <= s->ring.length && job.size) {
    // the plug-in wrote the bytes through its own mapping: whatever the kernel's cache holds for these pages is out of date (or is just not memory yet)
    (void)s->ring.memory->performOperation(kIOMemoryIncoherentIOFlush, job.offset, job.size);
    length = frameAccessUnit(s->codec, s->ring.kva + job.offset, job.size, s->nalSize, s->needParams ? s->params : nullptr, s->needParams ? s->paramsLength : 0, s->scratch,
                             s->scratchSize, &nals, types, &annexB);
    if (gLog >= 1 && s->jobsDone < 4) {
      char hex[3 * 24 + 1] = {};
      for (unsigned i = 0; i < 24 && i < job.size; ++i) snprintf(hex + 3 * i, 4, "%02x ", s->ring.kva[job.offset + i]);
      printf("rk3588-vdec: d5500: frame %u: %u bytes at ring offset %u, first bytes %s(%s)\n", job.seq, job.size, job.offset, hex, annexB ? "Annex B" : "length prefixed");
    }
  }
  IOLockUnlock(s->lock);
  const uint64_t prepEndUs = nowUs();
  s->prof.prep += prepEndUs - startUs;
  if (!length) {
    printf("rk3588-vdec: d5500: frame %u: %u bytes at %u cannot be read as %s (%u NAL units)\n", job.seq, job.size, job.offset, s->codec == kVdecH265 ? "H.265" : "H.264", nals);
    status = 1;
  } else {
    // 2. the decoder core
    s->current = &job;
    s->picturesInJob = 0;
    s->pictureCopied = false;
    VdecSink sink = {};
    sink.ctx = s;
    sink.decoded = pictureDecoded;
    vdec_core_stats(s->core, &before);
    uint64_t jobs0, run0, wall0, jobs1, run1, wall1;
    rk3588_vdec_block_stats(&jobs0, &run0, &wall0);
    const int rc = vdec_core_decode(s->core, s->scratch, length, job.seq, 0, &sink);
    rk3588_vdec_block_stats(&jobs1, &run1, &wall1);
    s->prof.blockJobs += jobs1 - jobs0;
    s->prof.blockRun += run1 - run0;
    s->prof.blockWall += wall1 - wall0;
    vdec_core_stats(s->core, &after);
    s->current = nullptr;
    s->needParams = rc < 0;   // the parameter sets have been seen once the core took a job; a reset forgets them
    if (rc < 0) {
      printf("rk3588-vdec: d5500: frame %u: the decoder core gave up, resetting it\n", job.seq);
      vdec_core_reset(s->core);
      status = 1;
    } else if (after.jobErrors != before.jobErrors) {
      printf("rk3588-vdec: d5500: frame %u: the block reported an error\n", job.seq);
      status = 1;
    } else if (!s->pictureCopied) {
      printf("rk3588-vdec: d5500: frame %u: no picture came out (%u NAL units, types %u %u %u %u; %u pictures decoded)\n", job.seq, nals, types[0], types[1], types[2],
             types[3], s->picturesInJob);
      status = 1;
    }
  }
  const uint64_t endUs = nowUs();
  const uint64_t us = endUs - startUs;
  s->prof.core += endUs - prepEndUs;
  ++s->prof.frames;
  if (status) ++s->jobsFailed;
  if (gLog >= 2 || (gLog >= 1 && s->jobsDone < 8) || status)
    printf("rk3588-vdec: d5500: frame %u: slot %u, %u bytes, %u NAL units, %llu us, status %u (%u frames done, %u failed)\n", job.seq, job.slot, job.size, nals,
           static_cast<unsigned long long>(us), status, s->jobsDone + 1, s->jobsFailed);
  ++s->jobsDone;
  const uint64_t finishUs = nowUs();
  finish(job, status, index, job.slot);
  s->prof.finish += nowUs() - finishUs;
}

void AppleD5500UserClient::drain() {
  Session *s = session;
  IOLockLock(s->lock);
  while ((s->count || s->busy) && !s->workerExited) IOLockSleep(s->lock, &s->event[1], THREAD_UNINT);
  IOLockUnlock(s->lock);
}

static void printProfile(const Session *s) {
  const Profile &p = s->prof;
  if (!p.frames) return;
  const unsigned long long n = p.frames;
  // two lines: the kernel's printf cuts a line at about 230 characters
  printf("rk3588-vdec: d5500: profile of %llu frames, microseconds per frame: queue %llu, prepare %llu, core %llu, finish %llu\n", n, static_cast<unsigned long long>(p.queue / n),
         static_cast<unsigned long long>(p.prep / n), static_cast<unsigned long long>(p.core / n), static_cast<unsigned long long>(p.finish / n));
  printf("rk3588-vdec: d5500: profile, inside core: block %llu start to interrupt, %llu in the job runner (%llu jobs); picture callback %llu = flush %llu + copy %llu + flush %llu + probe %llu; rest %llu\n",
         static_cast<unsigned long long>(p.blockRun / n), static_cast<unsigned long long>(p.blockWall / n), static_cast<unsigned long long>(p.blockJobs),
         static_cast<unsigned long long>(p.pictureCb / n), static_cast<unsigned long long>(p.flushPre / n), static_cast<unsigned long long>(p.copy / n),
         static_cast<unsigned long long>(p.flushPost / n), static_cast<unsigned long long>(p.probe / n),
         static_cast<unsigned long long>((p.core - p.pictureCb - p.blockWall) / n));
  if (p.rgaCopies || p.rgaFailed)
    printf("rk3588-vdec: d5500: profile, copies: %llu by the RGA3 (the block ran %llu us per picture), %llu by the CPU, %llu the RGA3 failed on\n", static_cast<unsigned long long>(p.rgaCopies),
           static_cast<unsigned long long>(p.rgaCopies ? p.rgaUs / p.rgaCopies : 0), static_cast<unsigned long long>(p.cpuCopies), static_cast<unsigned long long>(p.rgaFailed));
}

void AppleD5500UserClient::tearDownDecoder() {
  Session *s = session;
  if (!s->core) return;
  drain();
  printProfile(s);
  vdec_core_destroy(s->core);
  s->core = nullptr;
  if (s->scratch) {
    IOFree(s->scratch, s->scratchSize);
    s->scratch = nullptr;
    s->scratchSize = 0;
  }
}

void AppleD5500UserClient::stopWorker() {
  Session *s = session;
  IOLockLock(s->lock);
  if (s->workerStarted) {
    s->stop = true;
    IOLockWakeup(s->lock, &s->event[0], true);
    while (!s->workerExited) IOLockSleep(s->lock, &s->event[2], THREAD_UNINT);
  }
  IOLockUnlock(s->lock);
}

IOReturn AppleD5500UserClient::clientClose() {
  printf("rk3588-vdec: d5500: %s closes the service (calls:", ownerName);
  for (unsigned i = 0; i < kSelCount; ++i) printf(" %u", calls[i]);
  printf("; %u frames, %u failed)\n", session ? session->jobsDone : 0, session ? session->jobsFailed : 0);
  if (session) {
    if (session->workerStarted) {
      // let the frames in flight finish, then stop the thread
      drain();
      stopWorker();
    }
    tearDownDecoder();
    for (Surface &surface : session->slots) surface.release();
    session->ring.release();
  }
  if (haveCallback) {
    releaseAsyncReference64(asyncReference);
    haveCallback = false;
  }
  if (!isInactive()) (void)terminate();
  return kIOReturnSuccess;
}

IOReturn AppleD5500UserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *, OSObject *, void *) {
  if (!args || selector >= kSelCount || !session) {
    printf("rk3588-vdec: d5500: %s: selector %u: no such method\n", ownerName, selector);
    return kIOReturnBadArgument;
  }
  Session *s = session;
  ++calls[selector];
  if (gLog >= 2 || (gLog >= 1 && (calls[selector] <= 3 || (selector != kSelDecodeFrame && calls[selector] <= 6))))
    printf("rk3588-vdec: d5500: %s: %s (selector %u) call %u%s\n", ownerName, kSelName[selector], selector, calls[selector], args->asyncWakePort ? ", asynchronous" : "");
  if (args->structureInputSize != kSelIn[selector] || args->structureOutputSize != kSelOut[selector]) {
    printf("rk3588-vdec: d5500: %s: struct sizes in %u out %u, expected %#x / %#x\n", kSelName[selector], args->structureInputSize, args->structureOutputSize,
           kSelIn[selector], kSelOut[selector]);
    return kIOReturnBadArgument;
  }
  const uint32_t *in = static_cast<const uint32_t *>(args->structureInput);
  uint32_t *out = static_cast<uint32_t *>(args->structureOutput);
  if (!in || !out) return kIOReturnBadArgument;
  if (gLog >= 1 && calls[selector] <= (selector == kSelDecodeFrame ? 3u : 2u)) printWords("in ", in, args->structureInputSize);
  bzero(out, args->structureOutputSize);

  switch (selector) {
    case kSelCreateDecoder: {
      if (s->core) {
        printf("rk3588-vdec: d5500: a decoder exists already\n");
        return kIOReturnBusy;
      }
      const uint32_t ringId = in[0xa0 / 4];
      if (!s->ring.open(ringId, gLog >= 1)) {
        printf("rk3588-vdec: d5500: CreateDecoder: the ring buffer IOSurface %u cannot be used\n", ringId);
        return kIOReturnNotFound;
      }
      if (!rk3588_vdec_hw(&s->hw)) {
        printf("rk3588-vdec: d5500: CreateDecoder: the block is not ready\n");
        s->ring.release();
        return kIOReturnNotReady;
      }
      s->rgaSource = rgaWindow(s->hw);
      s->scratchSize = kMaxStream;
      s->scratch = static_cast<uint8_t *>(IOMallocZero(s->scratchSize));
      if (!s->scratch) {
        s->scratchSize = 0;
        s->ring.release();
        return kIOReturnNoMemory;
      }
      // the extradata the plug-in left at the start of the ring buffer: the parameter sets, and with them what the plug-in asks of the "hardware" below; and
      // which codec this is (H.264: an avcC record; H.265: an hvcC record; the plug-in's codec word at +0x10 is printed with the call's input but not
      // relied on: its numbering is not decoded)
      s->nalSize = in[2] >> 24;
      const uint32_t extraSize = in[2] & 0xffffff;
      s->codec = kVdecH264;
      if (extraSize && extraSize <= s->ring.length) {
        // the plug-in wrote the extradata through its own mapping of the ring buffer: whatever the kernel's cache holds for these pages is out of date
        (void)s->ring.memory->performOperation(kIOMemoryIncoherentIOFlush, 0, extraSize);
        s->codec = detectCodec(s->ring.kva, extraSize, s->nalSize);
      }
      if (s->codec == kVdecH265 && !gHevc) {
        printf("rk3588-vdec: d5500: CreateDecoder: an H.265 stream, and H.265 is switched off (rk3588-vdec-hevc=0)\n");
        IOFree(s->scratch, s->scratchSize);
        s->scratch = nullptr;
        s->scratchSize = 0;
        s->ring.release();
        return kIOReturnUnsupported;
      }
      if (vdec_core_create(&s->core, &s->hw, s->codec) != 0 || !s->core) {
        printf("rk3588-vdec: d5500: CreateDecoder: the decoder core cannot be opened (another client has it?)\n");
        s->core = nullptr;
        IOFree(s->scratch, s->scratchSize);
        s->scratch = nullptr;
        s->scratchSize = 0;
        s->ring.release();
        return kIOReturnBusy;
      }
      s->width = in[0];
      s->height = in[1];
      StreamInfo info = {};
      s->paramsLength = 0;
      s->needParams = false;
      if (extraSize && extraSize <= s->ring.length) {
        const unsigned units = parseExtradata(s->codec, s->ring.kva, extraSize, s->nalSize, s->params, sizeof s->params, &s->paramsLength, &info);
        if (!s->paramsLength) {
          // Nothing the parser takes as a parameter set: say so, VideoToolbox then fails the session (and uses the software decoder) instead of getting
          // a picture of the wrong stream. (Before the pmap fix of rk3588-wc-batch-flush.patch this was the sessions whose first cache line of the
          // ring buffer a stale line had zeroed.)
          char hex[3 * 32 + 1] = {};
          for (unsigned i = 0; i < 32 && i < extraSize; ++i) snprintf(hex + 3 * i, 4, "%02x ", s->ring.kva[i]);
          printf("rk3588-vdec: d5500: CreateDecoder: %u bytes of extradata, NAL length size %u, no parameter set in them: %s\n", extraSize, s->nalSize, hex);
          tearDownDecoder();
          s->ring.release();
          return kIOReturnBadArgument;
        }
        s->needParams = true;
        char hex[3 * 40 + 1] = {};
        for (unsigned i = 0; i < 40 && i < extraSize; ++i) snprintf(hex + 3 * i, 4, "%02x ", s->ring.kva[i]);
        printf("rk3588-vdec: d5500: CreateDecoder: %s, codec word %#x, %u bytes of extradata, NAL length size %u, %u parameter sets (%u bytes): %s\n", s->codec == kVdecH265 ? "H.265" : "H.264", in[0x10 / 4], extraSize, s->nalSize, units,
               static_cast<unsigned>(s->paramsLength), hex);
      } else {
        printf("rk3588-vdec: d5500: CreateDecoder: extradata of %u bytes (ring buffer %llu bytes): none used\n", extraSize,
               static_cast<unsigned long long>(s->ring.length));
      }
      // the answer: the stream as the SPS says (the size from the call if it did not parse)
      const uint32_t widthMbs = info.valid ? info.widthMbs : (s->width + 15) / 16, heightMbs = info.valid ? info.heightMbs : (s->height + 15) / 16;
      uint32_t dpb = info.valid ? info.numRefFrames : 4;
      dpb = (dpb < 2 ? 2 : dpb) + 2;
      if (dpb > 16) dpb = 16;
      out[0] = dpb;
      out[0x10 / 4] = widthMbs;
      out[0x14 / 4] = heightMbs;
      out[0x30 / 4] = info.valid ? info.chromaFormat : 1;
      out[0x34 / 4] = info.lumaMinus8;
      out[0x38 / 4] = info.chromaMinus8;
      printf("rk3588-vdec: d5500: CreateDecoder: stream %s: profile %u level %u, %ux%u 16x16 blocks, chroma_format_idc %u, bit depth %u/%u, %u reference frames, answering %u picture buffers\n",
             info.valid ? "from the SPS" : "NOT PARSED, assumed", info.profile, info.level, widthMbs, heightMbs, out[0x30 / 4], 8 + info.lumaMinus8, 8 + info.chromaMinus8,
             info.numRefFrames, dpb);
      if (!s->workerStarted) {
        s->workerStarted = true;
        s->stop = false;
        s->workerExited = false;
        thread_t thread = nullptr;
        if (kernel_thread_start(&AppleD5500UserClient::worker, this, &thread) != KERN_SUCCESS || !thread) {
          s->workerStarted = false;
          printf("rk3588-vdec: d5500: CreateDecoder: no thread for the decoder\n");
          tearDownDecoder();
          s->ring.release();
          return kIOReturnNoResources;
        }
        thread_deallocate(thread);
      }
      out[2] = 1;   // the decoder number the plug-in keeps at +0xbd8 of its context
      printf("rk3588-vdec: d5500: decoder created for %ux%u, ring buffer IOSurface %u\n", s->width, s->height, ringId);
      break;
    }
    case kSelDestroyDecoder:
      tearDownDecoder();
      s->ring.release();
      printf("rk3588-vdec: d5500: decoder destroyed after %u frames (%u failed)\n", s->jobsDone, s->jobsFailed);
      break;
    case kSelDecodeFrame: {
      // +0 bytes, +4 sequence number, +8 output slot, +0x10 offset of the bytes in the ring, +0x14 offset of the result records
      Job job = {};
      job.size = in[0];
      job.seq = in[1];
      job.slot = in[2];
      job.offset = in[0x10 / 4];
      job.records = in[0x14 / 4];
      if (!s->core || job.slot >= kSlots || !s->slots[job.slot].valid() || !s->ring.valid() || !job.size || job.size > kMaxStream - 16) {
        printf("rk3588-vdec: d5500: DecodeFrame %u refused: core %d slot %u (mapped %d) size %u\n", job.seq, s->core != nullptr, job.slot,
               job.slot < kSlots && s->slots[job.slot].valid(), job.size);
        out[0x88 / 4] = 1;
        return kIOReturnSuccess;
      }
      IOLockLock(s->lock);
      if (s->count >= kQueueDepth) {
        IOLockUnlock(s->lock);
        printf("rk3588-vdec: d5500: DecodeFrame %u refused: queue full\n", job.seq);
        out[0x88 / 4] = 1;
        return kIOReturnSuccess;
      }
      job.queuedUs = nowUs();
      s->queue[(s->head + s->count) % kQueueDepth] = job;
      ++s->count;
      IOLockWakeup(s->lock, &s->event[0], true);
      IOLockUnlock(s->lock);
      break;
    }
    case kSelMapPixelBuffer: {
      const uint32_t slot = in[0], surfaceId = in[1];
      if (slot >= kSlots) return kIOReturnBadArgument;
      IOLockLock(s->lock);
      s->slots[slot].release();
      const bool ok = s->slots[slot].open(surfaceId, gLog >= 1 && calls[selector] <= 4);
      IOLockUnlock(s->lock);
      if (!ok) {
        printf("rk3588-vdec: d5500: MapPixelBuffer: slot %u, IOSurface %u cannot be used\n", slot, surfaceId);
        return kIOReturnNotFound;
      }
      break;
    }
    case kSelUnmapPixelBuffer: {
      const uint32_t slot = in[0];
      if (slot >= kSlots) return kIOReturnBadArgument;
      IOLockLock(s->lock);
      s->slots[slot].release();
      IOLockUnlock(s->lock);
      break;
    }
    case kSelSetCallback:
      // The plug-in's call carries only the wake port; the function to call and its context are the structure input (four words: callback, then
      // context, 64 bits each). Without them the user-space dispatcher called a null function and videocodecd died (vt14).
      if (args->asyncWakePort && args->structureInputSize >= 16) {
        const uint32_t *words = static_cast<const uint32_t *>(args->structureInput);
        const uint64_t callback = words[0] | (static_cast<uint64_t>(words[1]) << 32);
        const uint64_t context = words[2] | (static_cast<uint64_t>(words[3]) << 32);
        // with the task: that is what marks the reference as the 64-bit kind (without it vt16's completion was read as a 32-bit one and the plug-in's
        // process jumped to a truncated address)
        setAsyncReference64(asyncReference, args->asyncWakePort, static_cast<mach_vm_address_t>(callback), static_cast<io_user_reference_t>(context), ownerTask);
        haveCallback = true;
        if (gLog >= 1)
          printf("rk3588-vdec: d5500: async reference %llx %llx %llx (call's own: %llx %llx %llx, %u entries)\n", static_cast<unsigned long long>(asyncReference[0]),
                 static_cast<unsigned long long>(asyncReference[1]), static_cast<unsigned long long>(asyncReference[2]),
                 static_cast<unsigned long long>(args->asyncReference[0]), static_cast<unsigned long long>(args->asyncReference[1]),
                 static_cast<unsigned long long>(args->asyncReference[2]), static_cast<unsigned>(args->asyncReferenceCount));
        if (gLog >= 1) printf("rk3588-vdec: d5500: callback armed\n");
      }
      break;
    default:   // SetCryptSession (nothing is encrypted here) and the word of the service
      break;
  }
  return kIOReturnSuccess;
}

extern "C" void rk3588_vdec_serve(void) {
  uint32_t value = 0;
  if (PE_parse_boot_argn("rk3588-vdec-d5500-log", &value, sizeof value)) gLog = static_cast<int>(value);
  value = 1;
  if (PE_parse_boot_argn("rk3588-vdec-d5500-rga", &value, sizeof value)) gRgaWanted = value != 0;
  gRgaLock = IOLockAlloc();
  value = 1;
  if (PE_parse_boot_argn("rk3588-vdec-hevc", &value, sizeof value)) gHevc = value != 0;
  value = 3;
  (void)PE_parse_boot_argn("rk3588-vdec-mpp-log", &value, sizeof value);
  vdc_log_level = static_cast<int>(value);
  AppleD5500 *service = OSTypeAlloc(AppleD5500);
  if (!service || !service->init() || !service->attach(IOService::getResourceService()) || !service->start(IOService::getResourceService())) {
    printf("rk3588-vdec: d5500: could not make the service\n");
    return;
  }
  // The platform sandbox decides whether a process may open a user client; it matches the name of the user client's class (AppleD5500UserClient, as the
  // original kext's), and a process in this comma separated list (names compared up to 16 characters, what the kernel's native_sandbox_grant looks for) is
  // let in anyway.
  service->setProperty("NativeSandboxGrantProcess", "videocodecd,mediaserverd,ptpd");
  // What the original kext copies from the device tree node /arm-io/vxd: the plug-in (H264H8.videodecoder, AppleD5500CheckPlatform, called by H264H8Register at
  // the start of videocodecd) reads this property of the first AppleD5500 service as a boolean and registers its H.265 decoder only when it is true.
  service->setProperty("HEVCSupported", gHevc);
  service->registerService();
  printf("rk3588-vdec: d5500: service \"AppleD5500\" registered\n");
}
