/*
 * VideoToolbox test of the rkvdec2 video decoder: the first user task that asks iPadOS's own decoder stack for H.264 pictures and compares what
 * comes back with the CRC-32 of each picture the host decoder produced (the .crc files of streams/).
 *
 * It runs as launchd's job com.apple.ptpd when boot-arg evb10-exec=2 (UniXNU-orangepi5 patches/xnu/opi5-launchd-test.patch: the kernel writes it to
 * /private/var/evb10/launchd from the file list Rk3588Vdec.cpp gives it, vdec_test_file, and union-mounts that directory over /usr/libexec, so it shadows the
 * sealed /usr/libexec/ptpd). VideoToolbox needs launchd's bootstrap port; as the init process (evb10-exec=1) it cannot create a decoder session. The output
 * goes to /dev/console, which is the serial line. It never exits after the last pass (launchd would restart it): it sleeps.
 * It runs 16 passes (the first right after boot, then after 60 s, then every 30 s) because the failures this test found appeared minutes after boot.
 *
 *   1. IOKit: is there an AppleD5500 service (what AppleD5500OpenConnection of H264H8.videodecoder looks for), and does IOServiceOpen(type 1) work?
 *   2. VTIsHardwareDecodeSupported(H.264).
 *   3. For each embedded stream: a VTDecompressionSession (hardware preferred, not required), the frames of the stream in decode order, each output
 *      picture's pixel format, size and CRC-32 (NV12, the visible rectangle, luma then chroma), compared as a set with the expected CRCs (the order
 *      of the output depends on the stream's reordering).
 *   4. The same with the hardware decoder required: with the service this must work; without it, VideoToolbox reports that it found none.
 *
 * Build: usertest/build.sh (clang -target arm64-apple-ios17.0, signed ad hoc; the cdhash goes into the owner trust bundle).
 */
#include <dlfcn.h>
#include <fcntl.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <IOKit/IOKitLib.h>
#include <VideoToolbox/VideoToolbox.h>

/* Exported by VideoToolbox on iOS too, but only declared for macOS in the SDK's headers. */
extern const CFStringRef kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder;
extern const CFStringRef kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder;
extern const CFStringRef kVTDecompressionPropertyKey_UsingHardwareAcceleratedVideoDecoder;

#include "vt_streams.inc"   /* struct VtStream { const char *name; const unsigned char *data; unsigned size; const unsigned *crcs; unsigned crcCount; }; kVtStreams[]; kVtStreamCount */

static int console = 2;

static void say(const char *format, ...)
{
   char line[640];
   va_list ap;
   va_start(ap, format);
   int n = vsnprintf(line, sizeof line - 1, format, ap);
   va_end(ap);
   if (n < 0)
      return;
   if (n > (int)sizeof line - 2)
      n = (int)sizeof line - 2;
   line[n++] = '\n';
   (void)write(console, line, (size_t)n);
}

static uint64_t now_ms(void)
{
   static mach_timebase_info_data_t tb;
   if (!tb.denom)
      mach_timebase_info(&tb);
   return mach_absolute_time() * tb.numer / tb.denom / 1000000;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n)
{
   static uint32_t table[256];
   static int ready;
   if (!ready) {
      for (uint32_t i = 0; i < 256; ++i) {
         uint32_t c = i;
         for (int k = 0; k < 8; ++k)
            c = c & 1 ? 0xedb88320u ^ (c >> 1) : c >> 1;
         table[i] = c;
      }
      ready = 1;
   }
   crc = ~crc;
   for (size_t i = 0; i < n; ++i)
      crc = table[(crc ^ p[i]) & 0xff] ^ (crc >> 8);
   return ~crc;
}

/* ---- Annex B -> access units ---- */

struct nal {
   const uint8_t *data;
   uint32_t size;
};

#define MAX_NALS 512
#define MAX_UNITS 128

struct unit {
   uint32_t first;   /* index of its first NAL in the list (slices only) */
   uint32_t count;
};

struct parsed {
   struct nal nals[MAX_NALS];
   uint32_t nalCount;
   const struct nal *sps, *pps;
   struct unit units[MAX_UNITS];
   uint32_t unitCount;
};

static int parse_annexb(const uint8_t *d, size_t n, struct parsed *out)
{
   memset(out, 0, sizeof *out);
   size_t i = 0;
   ssize_t start = -1;
   while (i + 3 <= n) {
      if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1) {
         if (start >= 0) {
            size_t end = i;
            while (end > (size_t)start && d[end - 1] == 0)
               --end;
            if (out->nalCount == MAX_NALS)
               return -1;
            out->nals[out->nalCount++] = (struct nal){d + start, (uint32_t)(end - start)};
         }
         start = (ssize_t)(i + 3);
         i += 3;
      } else {
         ++i;
      }
   }
   if (start >= 0 && (size_t)start < n) {
      if (out->nalCount == MAX_NALS)
         return -1;
      out->nals[out->nalCount++] = (struct nal){d + start, (uint32_t)(n - (size_t)start)};
   }
   for (uint32_t k = 0; k < out->nalCount; ++k) {
      const struct nal *nal = &out->nals[k];
      unsigned type = nal->data[0] & 0x1f;
      if (type == 7 && !out->sps)
         out->sps = nal;
      else if (type == 8 && !out->pps)
         out->pps = nal;
      else if (type == 1 || type == 5) {
         /* first_mb_in_slice == 0 (the first bit of the slice header is 1) starts a new picture */
         int firstSlice = nal->size > 1 && (nal->data[1] & 0x80);
         if (firstSlice || out->unitCount == 0) {
            if (out->unitCount == MAX_UNITS)
               return -1;
            out->units[out->unitCount++] = (struct unit){k, 1};
         } else {
            out->units[out->unitCount - 1].count++;
         }
      }
   }
   return out->sps && out->pps && out->unitCount ? 0 : -2;
}

/* ---- the decode callback ---- */

struct result {
   volatile uint32_t outputs;
   volatile uint32_t errors;
   uint32_t crc[MAX_UNITS];
   int64_t pts[MAX_UNITS];
   uint32_t width, height;
   OSType format;
   int surfaceBacked;
};

static struct result gResult;
static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;

static uint32_t picture_crc(CVPixelBufferRef pixels, uint32_t *w, uint32_t *h, OSType *fmt, int *iosurface)
{
   *w = (uint32_t)CVPixelBufferGetWidth(pixels);
   *h = (uint32_t)CVPixelBufferGetHeight(pixels);
   *fmt = CVPixelBufferGetPixelFormatType(pixels);
   *iosurface = CVPixelBufferGetIOSurface(pixels) != NULL;
   uint32_t crc = 0;
   if (CVPixelBufferLockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly) != kCVReturnSuccess)
      return 0xdeadbeef;
   if (CVPixelBufferGetPlaneCount(pixels) == 2) {
      const uint8_t *luma = CVPixelBufferGetBaseAddressOfPlane(pixels, 0);
      size_t lumaStride = CVPixelBufferGetBytesPerRowOfPlane(pixels, 0);
      const uint8_t *chroma = CVPixelBufferGetBaseAddressOfPlane(pixels, 1);
      size_t chromaStride = CVPixelBufferGetBytesPerRowOfPlane(pixels, 1);
      for (uint32_t y = 0; y < *h; ++y)
         crc = crc32_update(crc, luma + y * lumaStride, *w);
      for (uint32_t y = 0; y < *h / 2; ++y)
         crc = crc32_update(crc, chroma + y * chromaStride, *w);
   } else {
      crc = 0xbad0bad0;
   }
   CVPixelBufferUnlockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly);
   return crc;
}

static void decoded(void *refcon, void *frameRefcon, OSStatus status, VTDecodeInfoFlags flags, CVImageBufferRef image, CMTime pts, CMTime duration)
{
   (void)refcon;
   (void)duration;
   uintptr_t index = (uintptr_t)frameRefcon;
   if (status != noErr || !image) {
      say("vt:   frame %lu: status %d (info flags %#x)%s", (unsigned long)index, (int)status, (unsigned)flags, image ? "" : ", no image");
      __atomic_fetch_add(&gResult.errors, 1, __ATOMIC_SEQ_CST);
      __atomic_fetch_add(&gResult.outputs, 1, __ATOMIC_SEQ_CST);
      return;
   }
   uint32_t w, h;
   OSType fmt;
   int surface;
   uint32_t crc = picture_crc(image, &w, &h, &fmt, &surface);
   pthread_mutex_lock(&gLock);
   uint32_t slot = gResult.outputs;
   if (slot < MAX_UNITS) {
      gResult.crc[slot] = crc;
      gResult.pts[slot] = pts.value;
   }
   gResult.width = w;
   gResult.height = h;
   gResult.format = fmt;
   gResult.surfaceBacked = surface;
   pthread_mutex_unlock(&gLock);
   say("vt:   output %u: frame %lu pts %lld %ux%u format '%c%c%c%c' IOSurface %d crc %08x", slot, (unsigned long)index, (long long)pts.value, w, h,
       (char)(fmt >> 24), (char)(fmt >> 16), (char)(fmt >> 8), (char)fmt, surface, crc);
   __atomic_fetch_add(&gResult.outputs, 1, __ATOMIC_SEQ_CST);
}

/* ---- one stream through one session ---- */

static void cf_set_int(CFMutableDictionaryRef dict, CFStringRef key, int value)
{
   CFNumberRef number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &value);
   CFDictionarySetValue(dict, key, number);
   CFRelease(number);
}

static int run_stream(const struct VtStream *stream, int requireHardware)
{
   struct parsed *parsed = malloc(sizeof *parsed);
   if (!parsed || parse_annexb(stream->data, stream->size, parsed) != 0) {
      say("vt: %s: cannot split the stream into pictures", stream->name);
      free(parsed);
      return 1;
   }
   say("vt: %s (%s hardware): %u NAL units, %u pictures, %u expected CRCs", stream->name, requireHardware ? "requiring" : "preferring", parsed->nalCount,
       parsed->unitCount, stream->crcCount);

   const uint8_t *sets[2] = {parsed->sps->data, parsed->pps->data};
   size_t setSizes[2] = {parsed->sps->size, parsed->pps->size};
   CMVideoFormatDescriptionRef format = NULL;
   OSStatus status = CMVideoFormatDescriptionCreateFromH264ParameterSets(kCFAllocatorDefault, 2, sets, setSizes, 4, &format);
   if (status != noErr || !format) {
      say("vt: %s: CMVideoFormatDescriptionCreateFromH264ParameterSets: %d", stream->name, (int)status);
      free(parsed);
      return 1;
   }
   CMVideoDimensions dim = CMVideoFormatDescriptionGetDimensions(format);
   say("vt: %s: format description %dx%d", stream->name, dim.width, dim.height);

   CFMutableDictionaryRef spec = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
   CFDictionarySetValue(spec, kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder, kCFBooleanTrue);
   if (requireHardware)
      CFDictionarySetValue(spec, kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder, kCFBooleanTrue);
   CFMutableDictionaryRef attributes = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
   cf_set_int(attributes, kCVPixelBufferPixelFormatTypeKey, kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange);
   CFDictionaryRef surfaceProperties = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
   CFDictionarySetValue(attributes, kCVPixelBufferIOSurfacePropertiesKey, surfaceProperties);
   CFRelease(surfaceProperties);

   VTDecompressionOutputCallbackRecord callback = {decoded, NULL};
   VTDecompressionSessionRef session = NULL;
   uint64_t t0 = now_ms();
   /* the first session of the boot starts videocodecd, which was seen to be killed while it started (-12911, malfunction): ask again */
   for (unsigned attempt = 0; attempt < 3; ++attempt) {
      status = VTDecompressionSessionCreate(kCFAllocatorDefault, format, spec, attributes, &callback, &session);
      say("vt: %s: VTDecompressionSessionCreate: %d (%llu ms)", stream->name, (int)status, (unsigned long long)(now_ms() - t0));
      if (status != -12911)
         break;
      sleep(2);
   }
   CFRelease(spec);
   CFRelease(attributes);
   if (status != noErr || !session) {
      CFRelease(format);
      free(parsed);
      return requireHardware ? 0 : 1;   /* the required case is expected to be refused when there is no hardware decoder */
   }
   CFTypeRef hardware = NULL;
   status = VTSessionCopyProperty(session, kVTDecompressionPropertyKey_UsingHardwareAcceleratedVideoDecoder, kCFAllocatorDefault, &hardware);
   say("vt: %s: UsingHardwareAcceleratedVideoDecoder: %s (status %d)", stream->name, hardware ? (CFBooleanGetValue(hardware) ? "yes" : "no") : "unknown",
       (int)status);
   if (hardware)
      CFRelease(hardware);

   memset(&gResult, 0, sizeof gResult);
   int failures = 0;
   t0 = now_ms();
   for (uint32_t u = 0; u < parsed->unitCount; ++u) {
      const struct unit *unit = &parsed->units[u];
      size_t total = 0;
      for (uint32_t k = 0; k < unit->count; ++k)
         total += 4 + parsed->nals[unit->first + k].size;
      uint8_t *bytes = malloc(total);
      size_t at = 0;
      for (uint32_t k = 0; k < unit->count; ++k) {
         const struct nal *nal = &parsed->nals[unit->first + k];
         bytes[at++] = (uint8_t)(nal->size >> 24);
         bytes[at++] = (uint8_t)(nal->size >> 16);
         bytes[at++] = (uint8_t)(nal->size >> 8);
         bytes[at++] = (uint8_t)nal->size;
         memcpy(bytes + at, nal->data, nal->size);
         at += nal->size;
      }
      CMBlockBufferRef block = NULL;
      status = CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault, bytes, total, kCFAllocatorMalloc, NULL, 0, total, 0, &block);
      if (status != noErr) {
         say("vt: %s: CMBlockBufferCreateWithMemoryBlock: %d", stream->name, (int)status);
         free(bytes);
         ++failures;
         break;
      }
      CMSampleTimingInfo timing = {CMTimeMake(1, 30), CMTimeMake(u, 30), kCMTimeInvalid};
      CMSampleBufferRef sample = NULL;
      status = CMSampleBufferCreate(kCFAllocatorDefault, block, true, NULL, NULL, format, 1, 1, &timing, 1, &total, &sample);
      CFRelease(block);
      if (status != noErr || !sample) {
         say("vt: %s: CMSampleBufferCreate: %d", stream->name, (int)status);
         ++failures;
         break;
      }
      VTDecodeInfoFlags info = 0;
      status = VTDecompressionSessionDecodeFrame(session, sample, kVTDecodeFrame_1xRealTimePlayback, (void *)(uintptr_t)u, &info);
      CFRelease(sample);
      if (status != noErr) {
         say("vt: %s: picture %u: VTDecompressionSessionDecodeFrame: %d (info %#x)", stream->name, u, (int)status, (unsigned)info);
         ++failures;
         break;
      }
   }
   status = VTDecompressionSessionFinishDelayedFrames(session);
   say("vt: %s: FinishDelayedFrames: %d", stream->name, (int)status);
   for (int waited = 0; gResult.outputs < parsed->unitCount && waited < 300; ++waited)
      usleep(100 * 1000);
   status = VTDecompressionSessionWaitForAsynchronousFrames(session);
   say("vt: %s: WaitForAsynchronousFrames: %d, %u of %u pictures out in %llu ms", stream->name, (int)status, gResult.outputs, parsed->unitCount,
       (unsigned long long)(now_ms() - t0));

   /* The expected CRCs are in display order, the output order is the session's: compare as sets. */
   unsigned matched = 0;
   uint32_t seen = gResult.outputs < MAX_UNITS ? gResult.outputs : MAX_UNITS;
   unsigned used[MAX_UNITS] = {0};
   for (uint32_t o = 0; o < seen; ++o) {
      for (unsigned e = 0; e < stream->crcCount; ++e) {
         if (!used[e] && stream->crcs[e] == gResult.crc[o]) {
            used[e] = 1;
            ++matched;
            break;
         }
      }
   }
   int ok = !failures && !gResult.errors && gResult.outputs == stream->crcCount && matched == stream->crcCount;
   say("vt: %s: %u outputs, %u errors, %u of %u pictures match the host decoder: %s", stream->name, gResult.outputs, gResult.errors, matched,
       stream->crcCount, ok ? "PASS" : "FAIL");
   VTDecompressionSessionInvalidate(session);
   CFRelease(session);
   CFRelease(format);
   free(parsed);
   return ok ? 0 : 1;
}

/* ---- the kernel side ---- */

static void probe_service(void)
{
   CFMutableDictionaryRef match = IOServiceMatching("AppleD5500");
   io_iterator_t iterator = 0;
   kern_return_t kr = IOServiceGetMatchingServices(kIOMainPortDefault, match, &iterator);
   io_service_t service = kr == KERN_SUCCESS ? IOIteratorNext(iterator) : 0;
   say("vt: IOServiceGetMatchingServices(AppleD5500): %#x, service %#x", kr, service);
   if (service) {
      io_connect_t connect = 0;
      kr = IOServiceOpen(service, mach_task_self(), 1, &connect);
      say("vt: IOServiceOpen(AppleD5500, type 1): %#x, connection %#x", kr, connect);
      if (kr == KERN_SUCCESS)
         IOServiceClose(connect);
      IOObjectRelease(service);
   }
   if (iterator)
      IOObjectRelease(iterator);
}

int main(void)
{
   int fd = open("/dev/console", O_WRONLY);
   if (fd >= 0)
      console = fd;
   int init = getpid() == 1;
   say("vt: VideoToolbox test of the rkvdec2 decoder, pid %d uid %d (%s)", getpid(), getuid(), init ? "the init process" : "a job of launchd");
   /* As a job of launchd the test runs sixteen times, the first ones as the system comes up (services VideoToolbox needs may not be there at the first), the
    * rest while the system has been busy for a while: sessions that went wrong at random came out of the system's state of some minutes. */
   for (unsigned pass = 0; pass < (init ? 1u : 16u); ++pass) {
      if (pass)
         sleep(pass == 1 ? 60 : 30);
      say("vt: pass %u", pass);
      probe_service();
      say("vt: VTIsHardwareDecodeSupported(H.264): %d, (HEVC): %d", VTIsHardwareDecodeSupported(kCMVideoCodecType_H264),
          VTIsHardwareDecodeSupported(kCMVideoCodecType_HEVC));
      unsigned failed = 0;
      for (unsigned round = 0; round < 2; ++round)
         for (unsigned i = 0; i < kVtStreamCount; ++i)
            failed += run_stream(&kVtStreams[i], (int)round) != 0;
      say("vt: pass %u done, %u stream run(s) failed", pass, failed);
   }
   for (;;)
      sleep(1000);
}
