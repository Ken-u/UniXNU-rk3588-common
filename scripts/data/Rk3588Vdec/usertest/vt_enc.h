/*
 * The encoder part of the VideoToolbox test (included by vt_test.c after say() and cf_set_int()): one VTCompressionSession per codec with the hardware encoder
 * required, a few synthetic NV12 pictures, and a report of what comes out of it (status of every call, the compressed samples' sizes and first bytes, the
 * parameter sets). Its purpose at the logging stage of the kernel service (Rk3588VencAve2.cpp) is to make the plug-in (H264H9.videoencoder / H9.videoencoder)
 * open AppleAVE2Driver and send its calls, which the kernel prints; with the real service it checks the bit streams.
 *
 * Runs only when the kernel command line has rk3588-venc-ave2.
 */
#include <sys/sysctl.h>

extern const CFStringRef kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder;
extern const CFStringRef kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder;
extern const CFStringRef kVTCompressionPropertyKey_UsingHardwareAcceleratedVideoEncoder;
extern const CFStringRef kVTCompressionPropertyKey_MaxKeyFrameInterval;
extern const CFStringRef kVTCompressionPropertyKey_ExpectedFrameRate;

/* What a session asks of the encoder besides its size: the average bit rate, the longest run of pictures between key frames (0: not set) and the expected frame
 * rate (0: not set). The kernel service reads them from the plug-in's Prepare descriptor. */
struct enc_settings {
   int bitrate, keyint, fps;
   const char *tag;   /* prefix of the names of the data rows the session prints (tools/check-vt-enc.py --tag) */
   unsigned dump;     /* how many of its first samples are printed */
};

struct enc_state {
   unsigned frames, key, errors;
   size_t bytes;
   uint64_t firstUs;
   int described;
   uint32_t crc;
   int hevc;   /* the session's codec: the data rows of an H.265 session are named h265_... */
   const char *tag;
   unsigned dump;
};

static int enc_wanted(void)
{
   char args[1024] = {};
   size_t size = sizeof args - 1;
   if (sysctlbyname("kern.bootargs", args, &size, NULL, 0) != 0)
      return 0;
   return strstr(args, "rk3588-venc-ave2") != NULL;
}

static void hex_line(const char *what, const uint8_t *p, size_t n)
{
   char line[3 * 40 + 1] = {};
   for (size_t i = 0; i < n && i < 40; ++i)
      snprintf(line + 3 * i, 4, "%02x ", p[i]);
   say("vt-enc: %s: %s", what, line);
}

/* Bytes for the host: `vt-enc: data NAME OFFSET HEX` rows of 32 bytes (tools/check-vt-enc.py puts them back together and decodes them). */
static void hex_dump(const char *name, const uint8_t *p, size_t n)
{
   static const char digits[] = "0123456789abcdef";
   for (size_t off = 0; off < n; off += 32) {
      char line[65] = {};
      size_t k = 0;
      for (size_t i = 0; i < 32 && off + i < n; ++i) {
         line[k++] = digits[p[off + i] >> 4];
         line[k++] = digits[p[off + i] & 15];
      }
      say("vt-enc: data %s %06zx %s", name, off, line);
   }
}

static void enc_output(void *refcon, void *frameRefcon, OSStatus status, VTEncodeInfoFlags flags, CMSampleBufferRef sample)
{
   struct enc_state *st = refcon;
   (void)frameRefcon;
   if (status != noErr || !sample) {
      st->errors++;
      say("vt-enc: output callback: status %d flags %#x sample %p", (int)status, (unsigned)flags, (void *)sample);
      return;
   }
   CMBlockBufferRef block = CMSampleBufferGetDataBuffer(sample);
   size_t length = block ? CMBlockBufferGetDataLength(block) : 0;
   int key = 1;
   CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sample, false);
   if (attachments && CFArrayGetCount(attachments)) {
      CFDictionaryRef first = CFArrayGetValueAtIndex(attachments, 0);
      key = !CFDictionaryContainsKey(first, kCMSampleAttachmentKey_NotSync);
   }
   uint8_t head[40] = {};
   if (block && length)
      CMBlockBufferCopyDataBytes(block, 0, length < sizeof head ? length : sizeof head, head);
   if (!st->frames || st->frames < 3)
      say("vt-enc: sample %u: %zu bytes, %s, pts %lld flags %#x", st->frames, length, key ? "sync" : "not sync", (long long)CMSampleBufferGetPresentationTimeStamp(sample).value,
          (unsigned)flags);
   if (st->frames < 2)
      hex_line("sample head", head, length);
   if (st->frames < st->dump && block && length && length <= (1u << 20)) {
      uint8_t *all = malloc(length);
      if (all && CMBlockBufferCopyDataBytes(block, 0, length, all) == noErr) {
         char name[32];
         snprintf(name, sizeof name, "%s%ssample%u%s", st->tag, st->hevc ? "h265_" : "", st->frames, key ? "k" : "");
         hex_dump(name, all, length);
         hex_dump(name, all, length);   /* twice: the kernel's own log lines can cut into a row, the checker takes the union */
      }
      free(all);
   }
   if (!st->described) {
      st->described = 1;
      CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sample);
      if (format) {
         CMVideoDimensions dim = CMVideoFormatDescriptionGetDimensions(format);
         size_t count = 0;
         int nal = 0;
         const uint8_t *set = NULL;
         size_t setSize = 0;
         OSStatus ps = st->hevc ? CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format, 0, &set, &setSize, &count, &nal)
                                : CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, &set, &setSize, &count, &nal);
         say("vt-enc: format %dx%d, %s parameter sets: status %d, %zu sets, NAL length %d", dim.width, dim.height, st->hevc ? "H.265" : "H.264", (int)ps, count, nal);
         if (ps == noErr && set)
            hex_line("first parameter set", set, setSize);
         for (size_t k = 0; k < count && k < 4; ++k) {
            const uint8_t *one = NULL;
            size_t oneSize = 0;
            char name[24];
            OSStatus got = st->hevc ? CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format, k, &one, &oneSize, NULL, NULL)
                                    : CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, k, &one, &oneSize, NULL, NULL);
            if (got == noErr && one) {
               snprintf(name, sizeof name, "%s%sset%zu", st->tag, st->hevc ? "h265_" : "", k);
               hex_dump(name, one, oneSize);
            }
         }
      }
   }
   st->frames++;
   st->key += key;
   st->bytes += length;
}

static void fill_nv12(CVPixelBufferRef pixels, unsigned frame)
{
   CVPixelBufferLockBaseAddress(pixels, 0);
   size_t width = CVPixelBufferGetWidth(pixels), height = CVPixelBufferGetHeight(pixels);
   uint8_t *y = CVPixelBufferGetBaseAddressOfPlane(pixels, 0);
   size_t ys = CVPixelBufferGetBytesPerRowOfPlane(pixels, 0);
   for (size_t row = 0; row < height; ++row)
      for (size_t col = 0; col < width; ++col)
         y[row * ys + col] = (uint8_t)(16 + ((col + 2 * frame) * 3 / 2 + row) % 200);
   uint8_t *c = CVPixelBufferGetBaseAddressOfPlane(pixels, 1);
   size_t cs = CVPixelBufferGetBytesPerRowOfPlane(pixels, 1);
   for (size_t row = 0; row < height / 2; ++row)
      for (size_t col = 0; col < width / 2; ++col) {
         c[row * cs + 2 * col] = (uint8_t)(100 + (col + frame) % 50);
         c[row * cs + 2 * col + 1] = (uint8_t)(120 + (row + 2 * frame) % 40);
      }
   CVPixelBufferUnlockBaseAddress(pixels, 0);
}

static int run_encode(CMVideoCodecType codec, unsigned width, unsigned height, unsigned frames, int require, const struct enc_settings *settings)
{
   static const struct enc_settings defaults = {2000000, 0, 0, "", 8};
   const struct enc_settings *want = settings ? settings : &defaults;
   const char *name = codec == kCMVideoCodecType_HEVC ? "H.265" : "H.264";
   say("vt-enc: %s %ux%u, %u frames, hardware %s, bit rate %d key frame interval %d frame rate %d", name, width, height, frames, require ? "required" : "enabled", want->bitrate,
       want->keyint, want->fps);
   CFMutableDictionaryRef spec = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
   CFDictionarySetValue(spec, require ? kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder : kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder,
                        kCFBooleanTrue);
   struct enc_state st = {};
   st.hevc = codec == kCMVideoCodecType_HEVC;
   st.tag = want->tag;
   st.dump = want->dump;
   VTCompressionSessionRef session = NULL;
   uint64_t t0 = now_ms();
   OSStatus status = VTCompressionSessionCreate(kCFAllocatorDefault, (int32_t)width, (int32_t)height, codec, spec, NULL, NULL, enc_output, &st, &session);
   CFRelease(spec);
   say("vt-enc: VTCompressionSessionCreate: %d (%llu ms) session %p", (int)status, (unsigned long long)(now_ms() - t0), (void *)session);
   if (status != noErr || !session)
      return 1;
   CFTypeRef hw = NULL;
   OSStatus ps = VTSessionCopyProperty(session, kVTCompressionPropertyKey_UsingHardwareAcceleratedVideoEncoder, kCFAllocatorDefault, &hw);
   say("vt-enc: UsingHardwareAcceleratedVideoEncoder: status %d value %s", (int)ps, hw == kCFBooleanTrue ? "true" : hw == kCFBooleanFalse ? "false" : hw ? "?" : "none");
   if (hw)
      CFRelease(hw);
   int value = want->bitrate;
   CFNumberRef number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &value);
   say("vt-enc: set AverageBitRate: %d", (int)VTSessionSetProperty(session, kVTCompressionPropertyKey_AverageBitRate, number));
   CFRelease(number);
   if (want->keyint) {
      value = want->keyint;
      number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &value);
      say("vt-enc: set MaxKeyFrameInterval: %d", (int)VTSessionSetProperty(session, kVTCompressionPropertyKey_MaxKeyFrameInterval, number));
      CFRelease(number);
   }
   if (want->fps) {
      value = want->fps;
      number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &value);
      say("vt-enc: set ExpectedFrameRate: %d", (int)VTSessionSetProperty(session, kVTCompressionPropertyKey_ExpectedFrameRate, number));
      CFRelease(number);
   }
   say("vt-enc: set RealTime: %d", (int)VTSessionSetProperty(session, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue));
   say("vt-enc: set AllowFrameReordering: %d", (int)VTSessionSetProperty(session, kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse));
   t0 = now_ms();
   status = VTCompressionSessionPrepareToEncodeFrames(session);
   say("vt-enc: PrepareToEncodeFrames: %d (%llu ms)", (int)status, (unsigned long long)(now_ms() - t0));
   int failed = status != noErr;
   for (unsigned frame = 0; frame < frames && !failed; ++frame) {
      CFMutableDictionaryRef attrs = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
      CFDictionaryRef surface = CFDictionaryCreate(kCFAllocatorDefault, NULL, NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
      CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, surface);
      CFRelease(surface);
      CVPixelBufferRef pixels = NULL;
      CVReturn cv = CVPixelBufferCreate(kCFAllocatorDefault, width, height, kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange, attrs, &pixels);
      CFRelease(attrs);
      if (cv != kCVReturnSuccess || !pixels) {
         say("vt-enc: CVPixelBufferCreate: %d", (int)cv);
         failed = 1;
         break;
      }
      fill_nv12(pixels, frame);
      VTEncodeInfoFlags flags = 0;
      t0 = now_ms();
      status = VTCompressionSessionEncodeFrame(session, pixels, CMTimeMake(frame, 30), CMTimeMake(1, 30), NULL, NULL, &flags);
      if (frame < 2 || status != noErr)
         say("vt-enc: EncodeFrame %u: %d flags %#x (%llu ms)", frame, (int)status, (unsigned)flags, (unsigned long long)(now_ms() - t0));
      CFRelease(pixels);
      failed = status != noErr;
   }
   t0 = now_ms();
   status = VTCompressionSessionCompleteFrames(session, kCMTimeInvalid);
   say("vt-enc: CompleteFrames: %d (%llu ms)", (int)status, (unsigned long long)(now_ms() - t0));
   VTCompressionSessionInvalidate(session);
   CFRelease(session);
   say("vt-enc: %s done: %u of %u samples, %u sync, %zu bytes, %u errors", name, st.frames, frames, st.key, st.bytes, st.errors);
   return failed || st.frames != frames || st.errors;
}

static void probe_ave2(void)
{
   CFMutableDictionaryRef match = IOServiceMatching("AppleAVE2Driver");
   io_iterator_t iterator = 0;
   kern_return_t kr = IOServiceGetMatchingServices(kIOMainPortDefault, match, &iterator);
   io_service_t service = kr == KERN_SUCCESS ? IOIteratorNext(iterator) : 0;
   say("vt-enc: IOServiceGetMatchingServices(AppleAVE2Driver): %#x, service %#x", kr, service);
   if (service)
      IOObjectRelease(service);
   if (iterator)
      IOObjectRelease(iterator);
}

static void list_encoders(void)
{
   typedef OSStatus (*list_fn)(CFDictionaryRef, CFArrayRef *);
   list_fn copy = (list_fn)dlsym(RTLD_DEFAULT, "VTCopyVideoEncoderList");
   if (!copy) {
      say("vt-enc: VTCopyVideoEncoderList is not there");
      return;
   }
   CFArrayRef list = NULL;
   OSStatus status = copy(NULL, &list);
   say("vt-enc: VTCopyVideoEncoderList: %d, %ld encoders", (int)status, list ? (long)CFArrayGetCount(list) : -1L);
   for (CFIndex i = 0; list && i < CFArrayGetCount(list); ++i) {
      CFDictionaryRef d = CFArrayGetValueAtIndex(list, i);
      CFStringRef id = CFDictionaryGetValue(d, CFSTR("EncoderID"));
      CFStringRef codec = CFDictionaryGetValue(d, CFSTR("CodecName"));
      CFBooleanRef hw = CFDictionaryGetValue(d, CFSTR("HardwareAccelerated"));
      char a[96] = {}, b[96] = {};
      if (id)
         CFStringGetCString(id, a, sizeof a, kCFStringEncodingUTF8);
      if (codec)
         CFStringGetCString(codec, b, sizeof b, kCFStringEncodingUTF8);
      say("vt-enc:   encoder %s (%s) hardware %s", a, b, hw == kCFBooleanTrue ? "yes" : "no");
   }
   if (list)
      CFRelease(list);
}

static void encoder_pass(unsigned pass)
{
   say("vt-enc: pass %u", pass);
   probe_ave2();
   list_encoders();
   unsigned failed = 0;
   failed += run_encode(kCMVideoCodecType_H264, 640, 360, 5, 1, NULL) != 0;
   failed += run_encode(kCMVideoCodecType_HEVC, 640, 360, 5, 1, NULL) != 0;
   /* what the descriptors say when the settings change (which words are the bit rate, the key frame interval and the frame rate), and whether the pictures follow */
   static const struct enc_settings first = {1000000, 7, 25, "k_", 8}, second = {3000000, 12, 15, "r_", 6}, big = {6000000, 0, 0, "f_", 2}, odd = {1500000, 0, 0, "o_", 3};
   failed += run_encode(kCMVideoCodecType_H264, 640, 360, 16, 1, &first) != 0;   /* key frames at 0, 7 and 14 */
   failed += run_encode(kCMVideoCodecType_H264, 640, 360, 6, 1, &second) != 0;
   failed += run_encode(kCMVideoCodecType_H264, 1920, 1080, 4, 1, &big) != 0;
   failed += run_encode(kCMVideoCodecType_H264, 480, 270, 3, 1, &odd) != 0;      /* a size with neither a trace nor a multiple of 16 rows */
   /* the same for H.265 (the encoder core's second codec): the data rows of these sessions are named <tag>h265_... */
   failed += run_encode(kCMVideoCodecType_HEVC, 640, 360, 16, 1, &first) != 0;
   failed += run_encode(kCMVideoCodecType_HEVC, 640, 360, 6, 1, &second) != 0;
   failed += run_encode(kCMVideoCodecType_HEVC, 1920, 1080, 4, 1, &big) != 0;
   failed += run_encode(kCMVideoCodecType_HEVC, 480, 270, 3, 1, &odd) != 0;
   say("vt-enc: pass %u done, %u encoder run(s) failed", pass, failed);
}
