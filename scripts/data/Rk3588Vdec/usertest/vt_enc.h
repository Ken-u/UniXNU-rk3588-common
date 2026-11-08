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

struct enc_state {
   unsigned frames, key, errors;
   size_t bytes;
   uint64_t firstUs;
   int described;
   uint32_t crc;
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
   if (!st->described) {
      st->described = 1;
      CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sample);
      if (format) {
         CMVideoDimensions dim = CMVideoFormatDescriptionGetDimensions(format);
         size_t count = 0;
         int nal = 0;
         const uint8_t *set = NULL;
         size_t setSize = 0;
         OSStatus ps = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, &set, &setSize, &count, &nal);
         say("vt-enc: format %dx%d, H.264 parameter sets: status %d, %zu sets, NAL length %d", dim.width, dim.height, (int)ps, count, nal);
         if (ps == noErr && set)
            hex_line("first parameter set", set, setSize);
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

static int run_encode(CMVideoCodecType codec, unsigned width, unsigned height, unsigned frames, int require)
{
   const char *name = codec == kCMVideoCodecType_HEVC ? "H.265" : "H.264";
   say("vt-enc: %s %ux%u, %u frames, hardware %s", name, width, height, frames, require ? "required" : "enabled");
   CFMutableDictionaryRef spec = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
   CFDictionarySetValue(spec, require ? kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder : kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder,
                        kCFBooleanTrue);
   struct enc_state st = {};
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
   int value = 2000000;
   CFNumberRef number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &value);
   say("vt-enc: set AverageBitRate: %d", (int)VTSessionSetProperty(session, kVTCompressionPropertyKey_AverageBitRate, number));
   CFRelease(number);
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
   failed += run_encode(kCMVideoCodecType_H264, 640, 360, 5, 1) != 0;
   failed += run_encode(kCMVideoCodecType_HEVC, 640, 360, 5, 1) != 0;
   say("vt-enc: pass %u done, %u encoder run(s) failed", pass, failed);
}
