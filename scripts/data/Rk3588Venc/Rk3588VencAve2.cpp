// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The video encoder service of iPadOS: an IOService of class AppleAVE2Driver (what the VideoToolbox encoder plug-ins H264H9.videoencoder and H9.videoencoder
// look for with IOServiceMatching("AppleAVE2Driver")) with a user client AppleAVE2UserClient (the name the platform sandbox matches).
//
// THIS IS THE LOGGING STAGE. The original kernel extension's user client ABI was only read as names (nine methods, IO_Open ... IO_Process), no argument was
// decoded, so this service answers every selector with success and zeroes and prints what the plug-in sent: the selector, the scalars, the structure (hex),
// the size of the structure and scalars it wants back, whether the call is asynchronous, the memory the plug-in hands over or asks for. Run the VideoToolbox
// test (Rk3588Vdec/usertest/vt_test.c, "encoder" part) with boot argument rk3588-venc-ave2=1 and read the "ave2:" lines of the serial log.
//
// Boot argument rk3588-venc-ave2=N: 0 off (default), 1 register the service and log, 2 log every call in full (the default stops the hex dumps after the
// first calls of each selector).
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSNumber.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

#include "Rk3588Venc.h"

extern "C" void proc_selfname(char *buf, int size);

namespace {

int gLog = 1;
// The device type the original kernel extension reports to the plug-in in the first reply (index into the 28-entry platform table that both the plug-in and
// the kernel extension carry; entries 8 and 9 are the same family and the same function tables, 9 is the A10X's). Boot argument rk3588-venc-ave2-type.
uint32_t gDeviceType = 9;
uint32_t gSessions = 0;
bool gEncode = false;        // rk3588-venc-ave2-encode=1: the frames are encoded by the block (a trace played with the application's pictures), not made up
bool gSurfaceOwner = true;   // create the buffers for the client's task (rk3588-venc-ave2-owner=0: for nobody, as the first runs did)
constexpr unsigned kSelectors = 64;

void hexLines(const char *what, const void *data, size_t length) {
  const uint8_t *p = static_cast<const uint8_t *>(data);
  const size_t shown = length > 512 ? 512 : length;
  for (size_t offset = 0; offset < shown; offset += 32) {
    char line[3 * 32 + 1] = {};
    for (size_t i = 0; i < 32 && offset + i < shown; ++i) snprintf(line + 3 * i, 4, "%02x ", p[offset + i]);
    printf("rk3588-venc: ave2:   %s %04lx: %s\n", what, static_cast<unsigned long>(offset), line);
  }
  if (shown < length) printf("rk3588-venc: ave2:   %s ... %lu more bytes\n", what, static_cast<unsigned long>(length - shown));
}

// A big structure is mostly zeros: print the 16-byte rows that are not (at most maxRows of them) and say how many bytes were non-zero.
void nonZeroRows(const char *what, const uint8_t *data, size_t length, unsigned maxRows) {
  unsigned rows = 0, skipped = 0;
  size_t nonZero = 0;
  for (size_t offset = 0; offset < length; offset += 16) {
    bool any = false;
    for (size_t i = 0; i < 16 && offset + i < length; ++i)
      if (data[offset + i]) {
        any = true;
        ++nonZero;
      }
    if (!any) continue;
    if (rows >= maxRows) {
      ++skipped;
      continue;
    }
    ++rows;
    char line[3 * 16 + 1] = {};
    for (size_t i = 0; i < 16 && offset + i < length; ++i) snprintf(line + 3 * i, 4, "%02x ", data[offset + i]);
    printf("rk3588-venc: ave2:   %s %06lx: %s\n", what, static_cast<unsigned long>(offset), line);
  }
  printf("rk3588-venc: ave2:   %s: %lu non-zero bytes of %lu%s\n", what, static_cast<unsigned long>(nonZero), static_cast<unsigned long>(length),
         skipped ? " (more rows not shown)" : "");
}

// Read bytes of the caller's address space (the plug-in's own state, to see what its notification handler will find).
bool peekUser(task_t task, uint64_t address, void *out, size_t length) {
  IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange(address, length, kIODirectionOut, task);
  if (!md) return false;
  bool ok = false;
  if (md->prepare(kIODirectionOut) == kIOReturnSuccess) {
    ok = md->readBytes(0, out, length) == length;
    md->complete(kIODirectionOut);
  }
  md->release();
  return ok;
}

// ---- IOSurfaces.
//
// The plug-in keeps its request records in an IOSurface it creates itself ("AVE2" allocation tag, 32 records of 0x10630 bytes) and says which one in the open
// call (word at +0x14 of the 1032 bytes: its id); the kernel side maps it and answers by writing into the records (the notification only carries the number of
// the record). The buffers the plug-in works in are IOSurfaces too, created by the kernel and named in the answer to Start (ids in the record).
// The IOSurface kext keeps its surfaces in a table indexed by id inside the IOCoreSurfaceRoot service; offsets of the original 21H461 kext (the same the
// decoder's Surface class uses): surface +16 id, +48 the memory descriptor; root +208 the table, +216 its entry count, +312 the lock (an IOLock *).
// (plus, read the way the decoder's Surface class does: +88 width, +96 height, +104 bytes per row, +128 pixel format, +152 plane count, +384 the plane array of
// 56-byte elements: height at +8, where the plane starts at +16, bytes per row at +32)
constexpr size_t kRootTable = 208, kRootCount = 216, kRootLock = 312, kSurfId = 16, kSurfMemory = 48, kSurfWidth = 88, kSurfHeight = 96, kSurfStride = 104,
                 kSurfFormat = 128, kSurfPlanes = 152, kSurfPlaneArray = 384, kSurfPlaneCount = 392, kPlaneBytes = 56;
// IOSurfaceRoot::createSurface(root, task = 0, properties) of the original kext: in the kernel cache file it is at 0xfffffff005baaa24 ("mov x3, #0; b ...", a
// thin wrapper), the original display extension calls it through the GOT slot of its stub. The original kexts are linked into our kernel 0x1000000 lower
// (the "slide" of kernel.json: the display bindings read 0xfffffff004b9ead0 for the getter at 0xfffffff005b9ead0 of the file), so the function is at
// 0xfffffff004baaa24 at run time (the slot at 0xfffffff005f339b8 holds that); the first two words are checked before the call.
// (The first try read the slot at its file address 0xfffffff006f339b8, which is another place in this kernel: it read 0, run ave11.)
constexpr uintptr_t kCreateSurface = 0xfffffff004baaa24ull;
constexpr uint32_t kCreateSurfaceWords[2] = {0xd2800003u, 0x17ffffeau};

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

struct SurfaceMap {
  OSObject *object = nullptr;               // retained
  IOMemoryDescriptor *memory = nullptr;     // retained, prepared
  IOMemoryMap *map = nullptr;
  uint8_t *kva = nullptr;
  uint64_t length = 0;
  uint32_t id = 0;
  uint32_t format = 0, planes = 0;
  uint64_t width = 0, height = 0, stride = 0;
  struct Plane {
    uint64_t stride, offset, height;
  } plane[2] = {};

  bool valid() const { return kva != nullptr; }
  void release() {
    if (map) map->release();
    if (memory) {
      (void)memory->complete();
      memory->release();
    }
    if (object) object->release();
    *this = SurfaceMap();
  }
  // Looks the surface up by id, takes a reference and maps its memory into the kernel.
  bool open(uint32_t surfaceId) {
    *this = SurfaceMap();
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
    planes = field<uint32_t>(surface, kSurfPlanes);
    plane[0] = {stride, 0, height};
    plane[1] = {stride, stride * height, height / 2};
    const uint8_t *planeArray = field<const uint8_t *>(surface, kSurfPlaneArray);
    const uint32_t planeCount = field<uint32_t>(surface, kSurfPlaneCount);
    if (planes == 2 && planeArray && planeCount >= 2 && planeCount <= 4) {
      const uint64_t p0Stride = field<uint64_t>(planeArray, 32), p1Stride = field<uint64_t>(planeArray, kPlaneBytes + 32);
      if (p0Stride) plane[0].stride = p0Stride;
      if (p1Stride) plane[1].stride = p1Stride;
      const uint64_t p0Height = field<uint64_t>(planeArray, 8), p1Height = field<uint64_t>(planeArray, kPlaneBytes + 8);
      if (p0Height) plane[0].height = p0Height;
      if (p1Height) plane[1].height = p1Height;
      for (unsigned k = 0; k < 2; ++k) {   // where a plane starts: the word at +16 when it is a plausible offset (the decoder's boards showed the first plane need not start at 0)
        const uint64_t v = field<uint64_t>(planeArray, k * kPlaneBytes + 16);
        if (v < length && (k == 0 || v >= plane[0].offset + plane[0].stride * plane[0].height)) plane[k].offset = v;
      }
    }
    return true;
  }
};

void setNumber(OSDictionary *dictionary, const char *key, uint64_t value) {
  OSNumber *number = OSNumber::withNumber(value, 64);
  if (number) {
    dictionary->setObject(key, number);
    number->release();
  }
}

// A data buffer of `size` bytes as an IOSurface the user processes can look up by id (global). Returns the surface (retained, 0 on failure) and its id.
OSObject *createDataSurface(task_t task, uint64_t size, uint32_t *id) {
  *id = 0;
  IOService *root = surfaceRoot();
  typedef OSObject *(*CreateSurface)(IOService *, task_t, OSDictionary *);
  const uint32_t *code = reinterpret_cast<const uint32_t *>(kCreateSurface);
  if (!root || code[0] != kCreateSurfaceWords[0] || code[1] != kCreateSurfaceWords[1]) {
    printf("rk3588-venc: ave2: no IOSurface creator (root %p, words %08x %08x at %p)\n", static_cast<void *>(root), root ? code[0] : 0, root ? code[1] : 0,
           reinterpret_cast<const void *>(kCreateSurface));
    return nullptr;
  }
  CreateSurface create = reinterpret_cast<CreateSurface>(kCreateSurface);
  OSDictionary *properties = OSDictionary::withCapacity(8);
  if (!properties) return nullptr;
  setNumber(properties, "IOSurfaceWidth", size);
  setNumber(properties, "IOSurfaceHeight", 1);
  setNumber(properties, "IOSurfaceBytesPerElement", 1);
  setNumber(properties, "IOSurfaceBytesPerRow", size);
  setNumber(properties, "IOSurfaceAllocSize", size);
  setNumber(properties, "IOSurfacePixelFormat", 0);
  properties->setObject("IOSurfaceIsGlobal", kOSBooleanTrue);
  OSObject *surface = create(root, task, properties);
  properties->release();
  if (surface) *id = field<uint32_t>(surface, kSurfId);
  return surface;
}

// ---- H.264 parameter sets (made up for now: the plug-in parses them to learn the stream's shape).
struct BitWriter {
  uint8_t *bytes;
  size_t bits = 0;
  explicit BitWriter(uint8_t *to) : bytes(to) {}
  void put(uint32_t value, unsigned count) {
    for (unsigned i = count; i-- > 0;) {
      if (!(bits & 7)) bytes[bits >> 3] = 0;
      if ((value >> i) & 1) bytes[bits >> 3] |= static_cast<uint8_t>(0x80 >> (bits & 7));
      ++bits;
    }
  }
  void ue(uint32_t value) {
    const uint32_t v = value + 1;
    unsigned length = 0;
    while ((v >> length) > 1) ++length;
    put(0, length);
    put(v, length + 1);
  }
  void se(int32_t value) { ue(value > 0 ? static_cast<uint32_t>(2 * value - 1) : static_cast<uint32_t>(-2 * value)); }
  void trailing() {
    put(1, 1);
    while (bits & 7) put(0, 1);
  }
  size_t length() const { return bits >> 3; }
};

// Annex B SPS + PPS for a progressive frame of the given coded size (Main profile, CAVLC, one reference frame, no B frames); returns the length.
size_t makeParameterSets(uint8_t *out, uint32_t width, uint32_t height) {
  size_t n = 0;
  static const uint8_t startCode[4] = {0, 0, 0, 1};
  memcpy(out + n, startCode, 4);
  n += 4;
  out[n++] = 0x67;
  BitWriter sps(out + n);
  sps.put(77, 8);      // profile_idc: Main
  sps.put(0, 8);       // constraint flags
  sps.put(31, 8);      // level_idc 3.1
  sps.ue(0);           // seq_parameter_set_id
  sps.ue(0);           // log2_max_frame_num_minus4
  sps.ue(2);           // pic_order_cnt_type: 2 (output order is decoding order)
  sps.ue(1);           // max_num_ref_frames
  sps.put(0, 1);       // gaps_in_frame_num_value_allowed_flag
  sps.ue(width / 16 - 1);
  sps.ue(height / 16 - 1);
  sps.put(1, 1);       // frame_mbs_only_flag
  sps.put(1, 1);       // direct_8x8_inference_flag
  sps.put(0, 1);       // frame_cropping_flag
  sps.put(0, 1);       // vui_parameters_present_flag
  sps.trailing();
  n += sps.length();
  memcpy(out + n, startCode, 4);
  n += 4;
  out[n++] = 0x68;
  BitWriter pps(out + n);
  pps.ue(0);           // pic_parameter_set_id
  pps.ue(0);           // seq_parameter_set_id
  pps.put(0, 1);       // entropy_coding_mode_flag: CAVLC
  pps.put(0, 1);       // bottom_field_pic_order_in_frame_present_flag
  pps.ue(0);           // num_slice_groups_minus1
  pps.ue(0);           // num_ref_idx_l0_default_active_minus1
  pps.ue(0);           // num_ref_idx_l1_default_active_minus1
  pps.put(0, 1);       // weighted_pred_flag
  pps.put(0, 2);       // weighted_bipred_idc
  pps.se(0);           // pic_init_qp_minus26
  pps.se(0);           // pic_init_qs_minus26
  pps.se(0);           // chroma_qp_index_offset
  pps.put(1, 1);       // deblocking_filter_control_present_flag
  pps.put(0, 1);       // constrained_intra_pred_flag
  pps.put(0, 1);       // redundant_pic_cnt_present_flag
  pps.trailing();
  n += pps.length();
  return n;
}

}  // namespace

class AppleAVE2Driver : public IOService {
  OSDeclareDefaultStructors(AppleAVE2Driver)
 public:
  using IOService::newUserClient;
  IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) override;
};
OSDefineMetaClassAndStructors(AppleAVE2Driver, IOService)

class AppleAVE2UserClient : public IOUserClient {
  OSDeclareDefaultStructors(AppleAVE2UserClient)
  char ownerName[17] = {};
  unsigned calls[kSelectors] = {};
  unsigned total = 0;
  io_user_reference_t asyncReference[8] = {};   // the plug-in's own reference of SetCallback (port, function, context): where the frame notifications go
  bool haveCallback = false;
  task_t ownerTask = nullptr;
  void notify(uint64_t counter, uint64_t second, const char *what);
  uint64_t context = 0;   // the plug-in's frame receiver (refcon of the callback)
  SurfaceMap ring;        // the plug-in's record ring, mapped in the kernel
  static constexpr unsigned kBuffers = 4;
  OSObject *buffers[3][kBuffers] = {};   // the data surfaces created for Start (kept until the connection closes)
  uint32_t bufferIds[3][kBuffers] = {};
  bool buffersMade = false;
  OSObject *headerSurface = nullptr;   // where the plug-in finds the stream headers (SPS, PPS) of the first frame: its id is in word +0x170 of the Start answer
  uint32_t headerId = 0;
  unsigned yuvDumps = 0;
  unsigned receiverDumps = 0;
  uint32_t yuvNotified[32] = {};   // number + 1 of the YUV request answered in each ring position
  void fakeFrame(uint8_t *record);
  bool encodeFrame(uint8_t *record);   // the block encodes the application's picture (gEncode); false: nothing was written, make one up
  Rk3588VencStream *encoder = nullptr;
  Rk3588VencPicture picture = {};
  unsigned encoded = 0;
  uint32_t codec = 1;   // 1 = H.264, 2 = H.265: word +0x10 of the Open call's structure is 0 for the H.264 plug-in and 1 for the H.265 one (the same word of the Prepare / Start
                        // descriptor is the client number the Open answer gave: a first reading took it for the codec and only got the test program's two sessions right)
  void dumpReceiver();
  bool receiverDumped = false;
  unsigned walkRecords(const char *when, bool dump, uint64_t *numbers, uint32_t *kinds, unsigned room);
  void completePending(const char *when, bool dump);
  void answerStart(uint8_t *record);

 public:
  using IOUserClient::initWithTask;
  using IOUserClient::clientMemoryForType;
  bool initWithTask(task_t owningTask, void *securityToken, UInt32 type) override;
  IOReturn clientClose() override;
  IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) override;
  IOReturn registerNotificationPort(mach_port_t port, UInt32 type, UInt32 refCon) override;
  IOReturn setProperties(OSObject *properties) override;
  IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *arguments, IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) override;
};
OSDefineMetaClassAndStructors(AppleAVE2UserClient, IOUserClient)

IOReturn AppleAVE2Driver::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) {
  if (!handler) return kIOReturnBadArgument;
  AppleAVE2UserClient *client = OSTypeAlloc(AppleAVE2UserClient);
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

bool AppleAVE2UserClient::initWithTask(task_t owningTask, void *securityToken, UInt32 type) {
  if (!IOUserClient::initWithTask(owningTask, securityToken, type)) return false;
  ownerTask = owningTask;
  proc_selfname(ownerName, sizeof ownerName);
  printf("rk3588-venc: ave2: %s opens the service (connection type %u)\n", ownerName, static_cast<unsigned>(type));
  return true;
}

IOReturn AppleAVE2UserClient::clientClose() {
  printf("rk3588-venc: ave2: %s closes the connection after %u calls\n", ownerName, total);
  if (haveCallback) releaseAsyncReference64(asyncReference);
  haveCallback = false;
  ring.release();
  if (encoder) rk3588_venc_stream_close(encoder);
  encoder = nullptr;
  if (headerSurface) headerSurface->release();
  headerSurface = nullptr;
  for (auto &set : buffers)
    for (auto &surface : set) {
      if (surface) surface->release();
      surface = nullptr;
    }
  terminate();
  return kIOReturnSuccess;
}

IOReturn AppleAVE2UserClient::clientMemoryForType(UInt32 type, IOOptionBits *, IOMemoryDescriptor **) {
  printf("rk3588-venc: ave2: %s asks for client memory of type %u\n", ownerName, static_cast<unsigned>(type));
  return kIOReturnUnsupported;
}

IOReturn AppleAVE2UserClient::registerNotificationPort(mach_port_t port, UInt32 type, UInt32 refCon) {
  printf("rk3588-venc: ave2: %s registers a notification port %#x (type %u, refcon %#x)\n", ownerName, static_cast<unsigned>(reinterpret_cast<uintptr_t>(port)), static_cast<unsigned>(type),
         static_cast<unsigned>(refCon));
  return kIOReturnSuccess;
}

IOReturn AppleAVE2UserClient::setProperties(OSObject *properties) {
  printf("rk3588-venc: ave2: %s sets properties (%s)\n", ownerName, properties ? properties->getMetaClass()->getClassName() : "none");
  return kIOReturnSuccess;
}

// What the plug-in's frame receiver will find: it takes the first word of a notification as a sequence number, looks up record (number % count) of its own
// ring (0x10630 bytes each; byte 0 = in use, word 1 = number, word at 0xc0 = what the request was: 0x4568 for Prepare), and refuses a record that is not in use.
// Calls `visit` for every record in use and returns how many there were.
unsigned AppleAVE2UserClient::walkRecords(const char *when, bool dump, uint64_t *numbers, uint32_t *kinds, unsigned room) {
  constexpr size_t kRecord = 0x10630;
  uint32_t count = 32;
  if (ring.valid()) {
    // the plug-in's ring, mapped in the kernel by its IOSurface id (the open call): 32 records laid out from the start of the memory
    if (ring.length / kRecord < count) count = static_cast<uint32_t>(ring.length / kRecord);
    unsigned used = 0;
    for (uint32_t i = 0; i < count; ++i) {
      uint8_t *record = ring.kva + static_cast<size_t>(i) * kRecord;
      if (!record[0]) continue;
      const uint32_t number = field<uint32_t>(record, 4), kind = field<uint32_t>(record, 0xc0);
      printf("rk3588-venc: ave2:   ring (%s): record %u: in use, number %u, kind %#x, frame state %u, slot %u, status %#x, stamp %#llx\n", when, i, number, kind, field<uint32_t>(record, 0x5b34),
             field<uint32_t>(record, 0x46c), field<uint32_t>(record, 0x2e4), static_cast<unsigned long long>(field<uint64_t>(record, 0x6b90)));
      if (dump) nonZeroRows("record", record, kRecord, 60);
      if (used < room) {
        numbers[used] = number;
        kinds[used] = kind;
      }
      ++used;
    }
    return used < room ? used : room;
  }
  if (!context) return 0;
  uint8_t a[0x20] = {};
  if (!peekUser(ownerTask, context + 0xc680, a, sizeof a)) {
    printf("rk3588-venc: ave2:   peek (%s): cannot read the receiver at %#llx\n", when, static_cast<unsigned long long>(context));
    return 0;
  }
  // [receiver + 0xc688] points at the ring's own descriptor: word 0 = number of records, +8 = where they are
  uint64_t pool = 0;
  memcpy(&pool, a + 8, sizeof pool);
  uint8_t desc[0x20] = {};
  if (!pool || !peekUser(ownerTask, pool, desc, sizeof desc)) {
    printf("rk3588-venc: ave2:   peek (%s): no ring at %#llx\n", when, static_cast<unsigned long long>(pool));
    return 0;
  }
  uint64_t base = 0;
  memcpy(&count, desc, sizeof count);
  memcpy(&base, desc + 8, sizeof base);
  if (!base || !count || count > 64) return 0;
  unsigned used = 0;
  for (uint32_t i = 0; i < count; ++i) {
    uint8_t head[0x10] = {};
    uint32_t kind = 0, number = 0;
    const uint64_t address = base + static_cast<uint64_t>(i) * kRecord;
    if (!peekUser(ownerTask, address, head, sizeof head) || !peekUser(ownerTask, address + 0xc0, &kind, sizeof kind)) continue;
    memcpy(&number, head + 4, sizeof number);
    if (!head[0]) continue;
    printf("rk3588-venc: ave2:   peek (%s): record %u at %#llx: in use, number %u, kind %#x\n", when, i, static_cast<unsigned long long>(address), number, kind);
    if (used < room) {
      numbers[used] = number;
      kinds[used] = kind;
    }
    ++used;
  }
  return used < room ? used : room;
}

// The answer to Start, written into the plug-in's record the way the original extension does (the plug-in's Start handler reads it when the notification
// arrives): at +0x170 a count, at +0x174 / +0x1ec / +0x264 thirty IOSurface ids each (the plug-in looks them up, locks them and keeps their base address; the
// first set also their size). Which set is for what is read from the plug-in's use of them: set 1 (+0x1ec) are blocks of 0x2cab8 bytes the plug-in fills with
// the per-frame parameters and the kernel reads, set 0 (+0x174) are the buffers it copies the coded frame out of, set 2 (+0x264) is not read yet.
void AppleAVE2UserClient::answerStart(uint8_t *record) {
  static const uint64_t sizes[3] = {0x100000, 0x30000, 0x100000};
  if (!buffersMade) {
    for (unsigned set = 0; set < 3; ++set)
      for (unsigned i = 0; i < kBuffers; ++i) {
        buffers[set][i] = createDataSurface(gSurfaceOwner ? ownerTask : nullptr, sizes[set], &bufferIds[set][i]);
        printf("rk3588-venc: ave2:   buffer %u.%u: %llu bytes, surface id %u%s\n", set, i, static_cast<unsigned long long>(sizes[set]), bufferIds[set][i], buffers[set][i] ? "" : " (failed)");
      }
    buffersMade = true;
  }
  if (!headerSurface) {
    headerSurface = createDataSurface(gSurfaceOwner ? ownerTask : nullptr, 0x1000, &headerId);
    printf("rk3588-venc: ave2:   header surface: surface id %u%s\n", headerId, headerSurface ? "" : " (failed)");
  }
  // Word +0x170 is not a count (the first reading of the plug-in took it for one): the plug-in's code that sends the stream headers looks the surface of that id up.
  memcpy(record + 0x170, &headerId, sizeof headerId);
  static const size_t offsets[3] = {0x174, 0x1ec, 0x264};
  for (unsigned set = 0; set < 3; ++set)
    for (unsigned i = 0; i < kBuffers; ++i) memcpy(record + offsets[set] + 4 * i, &bufferIds[set][i], sizeof(uint32_t));
}

// What the hardware would have left for a finished frame, made up: one slice. The plug-in's frame-done code takes from the slot's parameter block (set 1) the
// number of slices (+0xf0), a 12-byte filler entry per frame (+0xf4) and an array of 0x220-byte slice records from +0x180 (word 0 = the slice's size including its
// header, byte +0x20c = the header's size; the first record with size 0 ends the list), and the bytes themselves from the slot's output buffer (set 0). Here
// the header is a start code and the slice a few made-up bytes: enough for the plug-in to get as far as handing a sample to VideoToolbox, not a decodable picture.
void AppleAVE2UserClient::fakeFrame(uint8_t *record) {
  const uint32_t slot = field<uint32_t>(record, 0x46c);
  if (slot >= kBuffers || !bufferIds[0][slot] || !bufferIds[1][slot]) {
    printf("rk3588-venc: ave2:   fake frame: no buffers for slot %u\n", slot);
    return;
  }
  static const uint8_t slice[] = {0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00, 0x33, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc};
  constexpr uint32_t kHeader = 4;
  const uint32_t size = sizeof slice;
  SurfaceMap out, block;
  if (!out.open(bufferIds[0][slot]) || !block.open(bufferIds[1][slot])) {
    printf("rk3588-venc: ave2:   fake frame: cannot map the buffers of slot %u\n", slot);
    out.release();
    block.release();
    return;
  }
  memcpy(out.kva, slice, sizeof slice);
  // The stream headers the plug-in's header sender reads from the header surface: length in bits at +0x98 of the parameter block; the coded size of the
  // picture is in the request (+0xbf8 / +0xbfc: width and height).
  uint32_t headerBits = 0;
  if (headerSurface) {
    SurfaceMap header;
    if (header.open(headerId)) {
      uint32_t width = field<uint32_t>(record, 0xbf8), height = field<uint32_t>(record, 0xbfc);
      if (!width || !height || (width & 15) || (height & 15)) {
        width = 640;
        height = 368;
      }
      const size_t bytes = makeParameterSets(header.kva, width, height);
      headerBits = static_cast<uint32_t>(bytes * 8);
      printf("rk3588-venc: ave2:   headers for %ux%u: %zu bytes\n", width, height, bytes);
      hexLines("headers", header.kva, bytes);
      header.release();
    }
  }
  memcpy(block.kva + 0x98, &headerBits, sizeof headerBits);
  // With the plug-in's settings word 0 clear (as it is here) the same count is read from the request record itself, at +0x5528 (host emulation of the handler:
  // `tmp/media/ave_emu.py`; the bits the sender found in the block's +0x98 when that word is set).
  memcpy(record + 0x5528, &headerBits, sizeof headerBits);
  const uint32_t slices = 1;
  // The 12-byte entry per frame is not a slice's place: its eight bytes are the prefix of a filler-data NAL unit (last byte = their count) and the word after
  // them the number of 0xff filler bytes to add after the slice. Zero = no filler (a size here made the plug-in copy a negative number of bytes).
  const uint64_t fillerPrefix = 0;
  const uint32_t fillerBytes = 0;
  memcpy(block.kva + 0xf0, &slices, sizeof slices);
  memcpy(block.kva + 0xf4, &fillerPrefix, sizeof fillerPrefix);
  memcpy(block.kva + 0xfc, &fillerBytes, sizeof fillerBytes);
  memcpy(block.kva + 0x180, &size, sizeof size);
  block.kva[0x38c] = kHeader;
  printf("rk3588-venc: ave2:   fake frame for slot %u: one slice of %u bytes\n", slot, size);
  out.release();
  block.release();
}

namespace {

// The Annex B NAL units of a stream: where each starts (at its start code, a zero byte in front of a three-byte one counted into it), where its header byte is, its type.
struct Nal {
  uint32_t start, header, end;
  uint8_t type;
};
unsigned splitNals(const uint8_t *p, uint32_t length, Nal *out, unsigned room, bool hevc) {
  unsigned n = 0;
  for (uint32_t i = 0; i + 3 < length;) {
    if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1) {
      if (n && n <= room) out[n - 1].end = i > out[n - 1].header + 1 && p[i - 1] == 0 ? i - 1 : i;
      if (n < room) {
        out[n].start = i > 0 && p[i - 1] == 0 && (n == 0 || i - 1 > out[n - 1].header) ? i - 1 : i;
        out[n].header = i + 3;
        out[n].type = hevc ? (p[i + 3] >> 1) & 0x3f : p[i + 3] & 0x1f;   // H.265: six bits after the forbidden zero bit of a two-byte header
        out[n].end = length;
      }
      ++n;
      i += 3;
    } else {
      ++i;
    }
  }
  return n < room ? n : room;
}

}  // namespace

// What the block wrote for the application's picture, in the places the plug-in's frame-done code reads (see fakeFrame for the layout): the picture is copied
// from the application's NV12 pixel buffer (the IOSurface whose id is in the request at +0x134) into the planar source picture of the trace, the next job of the
// trace runs, and its slice NAL units go into the slot's output buffer with a slice record each; the trace's parameter sets go to the header surface.
bool AppleAVE2UserClient::encodeFrame(uint8_t *record) {
  const uint32_t slot = field<uint32_t>(record, 0x46c), pixelsId = field<uint32_t>(record, 0x134);
  if (slot >= kBuffers || !bufferIds[0][slot] || !bufferIds[1][slot] || !headerSurface) return false;
  SurfaceMap pixels;
  if (!pixels.open(pixelsId)) {
    printf("rk3588-venc: ave2:   encode: the pixel buffer (surface %u) cannot be mapped\n", pixelsId);
    return false;
  }
  const uint32_t width = static_cast<uint32_t>(pixels.width), height = static_cast<uint32_t>(pixels.height);
  const bool nv12 = pixels.planes == 2 && (pixels.format == 0x34323076u || pixels.format == 0x34323066u);   // '420v', '420f'
  if (!encoded)
    printf("rk3588-venc: ave2:   encode: pixel buffer surface %u: %ux%u format %#x planes %u, plane 0 offset %llu stride %llu, plane 1 offset %llu stride %llu, %llu bytes\n", pixelsId, width,
           height, pixels.format, pixels.planes, static_cast<unsigned long long>(pixels.plane[0].offset), static_cast<unsigned long long>(pixels.plane[0].stride),
           static_cast<unsigned long long>(pixels.plane[1].offset), static_cast<unsigned long long>(pixels.plane[1].stride), static_cast<unsigned long long>(pixels.length));
  if (!nv12 || (width & 1) || (height & 1)) {
    printf("rk3588-venc: ave2:   encode: the pixel buffer is not NV12 (format %#x, %u planes)\n", pixels.format, pixels.planes);
    pixels.release();
    return false;
  }
  if (!encoder) {
    const char *name = nullptr;
    encoder = rk3588_venc_stream_open(width, height, codec == 2, &picture, &name);
    if (!encoder) {
      printf("rk3588-venc: ave2:   encode: no encoder for %ux%u %s (no trace of that size, or the block is not ready)\n", width, height, codec == 2 ? "H.265" : "H.264");
      pixels.release();
      return false;
    }
    printf("rk3588-venc: ave2:   encode: trace %s, source picture %ux%u strides %u/%u\n", name, picture.width, picture.height, picture.yStride, picture.uStride);
  }
  if (picture.width != width || picture.height != height) {
    printf("rk3588-venc: ave2:   encode: the picture is %ux%u, the trace's %ux%u\n", width, height, picture.width, picture.height);
    pixels.release();
    return false;
  }
  // NV12 -> planar 4:2:0
  const uint8_t *luma = pixels.kva + pixels.plane[0].offset, *chroma = pixels.kva + pixels.plane[1].offset;
  if (pixels.plane[0].offset + pixels.plane[0].stride * height > pixels.length || pixels.plane[1].offset + pixels.plane[1].stride * (height / 2) > pixels.length) {
    printf("rk3588-venc: ave2:   encode: the planes do not fit the surface\n");
    pixels.release();
    return false;
  }
  for (uint32_t row = 0; row < height; ++row) memcpy(picture.y + static_cast<size_t>(row) * picture.yStride, luma + row * pixels.plane[0].stride, width);
  for (uint32_t row = 0; row < height / 2; ++row) {
    const uint8_t *in = chroma + row * pixels.plane[1].stride;
    uint8_t *u = picture.u + static_cast<size_t>(row) * picture.uStride, *v = picture.v + static_cast<size_t>(row) * picture.uStride;
    for (uint32_t col = 0; col < width / 2; ++col) {
      u[col] = in[2 * col];
      v[col] = in[2 * col + 1];
    }
  }
  pixels.release();
  Rk3588VencResult result;
  const bool ok = rk3588_venc_stream_encode(encoder, &result);
  if (!ok || !result.stream || !result.length) return false;
  Nal nals[32];
  const bool hevc = codec == 2;
  const unsigned count = splitNals(result.stream, result.length, nals, 32, hevc);
  SurfaceMap out, block, header;
  if (!out.open(bufferIds[0][slot]) || !block.open(bufferIds[1][slot]) || !header.open(headerId)) {
    printf("rk3588-venc: ave2:   encode: cannot map the buffers of slot %u\n", slot);
    out.release();
    block.release();
    header.release();
    return false;
  }
  // The parameter sets: the trace's (what MPP wrote in front of the first job); a stream that carries its own wins.
  const uint8_t *sets = result.headers;
  uint32_t setBytes = result.headerLength;
  uint8_t own[512];
  uint32_t ownBytes = 0;
  uint32_t slices = 0, written = 0;
  const uint8_t kStartCode[4] = {0, 0, 0, 1};
  uint32_t sliceSize[31] = {};
  bool idr = false;
  for (unsigned i = 0; i < count && slices < 31; ++i) {
    const uint32_t size = nals[i].end - nals[i].start, startSize = nals[i].header - nals[i].start;
    const bool parameterSet = hevc ? nals[i].type >= 32 && nals[i].type <= 34 : nals[i].type == 7 || nals[i].type == 8;   // H.265: VPS, SPS, PPS
    if (parameterSet) {
      if (ownBytes + 4 + size - startSize <= sizeof own) {
        memcpy(own + ownBytes, kStartCode, 4);
        memcpy(own + ownBytes + 4, result.stream + nals[i].header, size - startSize);
        ownBytes += 4 + size - startSize;
      }
      continue;
    }
    const bool slice = hevc ? nals[i].type <= 9 || (nals[i].type >= 16 && nals[i].type <= 21) : nals[i].type == 1 || nals[i].type == 5;
    if (!slice || nals[i].start < result.hwOffset) continue;   // bytes in front of where the block began writing are the recording's CPU-written head (stale here)
    if (written + size > out.length) break;
    memcpy(out.kva + written, result.stream + nals[i].start, size);
    written += size;
    sliceSize[slices] = size;
    idr |= hevc ? nals[i].type >= 16 && nals[i].type <= 21 : nals[i].type == 5;
    ++slices;
  }
  if (ownBytes) {
    sets = own;
    setBytes = ownBytes;
  }
  if (!slices || !sets || !setBytes || setBytes > header.length) {
    printf("rk3588-venc: ave2:   encode: job %u gave %u NAL units, %u slices, %u bytes of parameter sets\n", result.job, count, slices, setBytes);
    out.release();
    block.release();
    header.release();
    return false;
  }
  memcpy(header.kva, sets, setBytes);
  const uint32_t headerBits = setBytes * 8;
  memcpy(block.kva + 0x98, &headerBits, sizeof headerBits);
  memcpy(record + 0x5528, &headerBits, sizeof headerBits);
  memcpy(block.kva + 0xf0, &slices, sizeof slices);
  const uint64_t fillerPrefix = 0;
  const uint32_t fillerBytes = 0;
  memcpy(block.kva + 0xf4, &fillerPrefix, sizeof fillerPrefix);
  memcpy(block.kva + 0xfc, &fillerBytes, sizeof fillerBytes);
  for (uint32_t i = 0; i <= slices && i < 32; ++i) {
    uint8_t *entry = block.kva + 0x180 + static_cast<size_t>(i) * 0x220;
    const uint32_t size = i < slices ? sliceSize[i] : 0;
    memcpy(entry, &size, sizeof size);
    // Byte +0x20c is how many bytes the plug-in leaves off the end of the slice (it copies size - that from the buffer, replaces the start code with the NAL's length
    // and appends 00 00 03): with the NAL's own header size here the last four bytes of every slice were lost (board run ave29, host emulation of the handler), so 0.
    entry[0x20c] = 0;
  }
  // The frame type the plug-in logs and builds its sample's attachments from, at +0x5b34 of the record (found by running its handler on the host with 0..3 there):
  // 3 is an IDR picture (a sync sample, an I frame), 0 an I picture that is not an IDR, 1 a P picture, 2 a B picture.
  const uint32_t frameType = idr ? 3 : 1;
  memcpy(record + 0x5b34, &frameType, sizeof frameType);
  if (encoded < 8 || (encoded & 31) == 0)
    printf("rk3588-venc: ave2:   encode: frame %u: job %u, %u bytes from the block in %u us: %u NAL units, %u slices (%u bytes), %u bytes of parameter sets\n", encoded, result.job, result.length,
           result.runUs, count, slices, written, setBytes);
  ++encoded;
  out.release();
  block.release();
  header.release();
  return true;
}

// The whole frame receiver of the plug-in (the object its notification handler works on), read in pieces and printed as the rows that are not zero: this is what
// the handler's code can be run against on the host. A piece that cannot be read ends the dump.
void AppleAVE2UserClient::dumpReceiver() {
  if (!context) return;
  constexpr size_t kPiece = 0x1000, kMax = 0x60000;
  uint8_t *piece = static_cast<uint8_t *>(IOMallocZero(kPiece));
  if (!piece) return;
  printf("rk3588-venc: ave2:   receiver dump at %#llx\n", static_cast<unsigned long long>(context));
  size_t read = 0;
  for (size_t offset = 0; offset < kMax; offset += kPiece) {
    if (!peekUser(ownerTask, context + offset, piece, kPiece)) break;
    read = offset + kPiece;
    for (size_t row = 0; row < kPiece; row += 16) {
      bool any = false;
      for (size_t i = 0; i < 16; ++i) any |= piece[row + i] != 0;
      if (!any) continue;
      char line[3 * 16 + 1] = {};
      for (size_t i = 0; i < 16; ++i) snprintf(line + 3 * i, 4, "%02x ", piece[row + i]);
      printf("rk3588-venc: ave2:   rcv %06lx: %s\n", static_cast<unsigned long>(offset + row), line);
    }
  }
  printf("rk3588-venc: ave2:   receiver dump: %zu bytes read\n", read);
  IOFree(piece, kPiece);
}

// Answer every request the plug-in has outstanding, in the order of its ring: the plug-in's own bookkeeping says what each one was.
void AppleAVE2UserClient::completePending(const char *when, bool dump) {
  uint64_t numbers[32] = {};
  uint32_t kinds[32] = {};
  const unsigned n = walkRecords(when, dump, numbers, kinds, 32);
  for (unsigned i = 0; i < n; ++i) {
    if (kinds[i] == 0x4567 && ring.valid()) {
      uint8_t *record = ring.kva + static_cast<size_t>(numbers[i] % 32) * 0x10630;
      answerStart(record);
    }
    if (kinds[i] == 0x4569 && ring.valid() && yuvDumps < 2) {
      // The first frame requests: the whole record, then every parameter block (set 1) the plug-in has written into.
      ++yuvDumps;
      uint8_t *record = ring.kva + static_cast<size_t>(numbers[i] % 32) * 0x10630;
      printf("rk3588-venc: ave2:   YUV record number %llu\n", static_cast<unsigned long long>(numbers[i]));
      nonZeroRows("yuv", record, 0x10630, 500);
      for (unsigned b = 0; b < kBuffers; ++b) {
        SurfaceMap block;
        if (bufferIds[1][b] && block.open(bufferIds[1][b])) {
          printf("rk3588-venc: ave2:   parameter block %u (surface %u, %llu bytes)\n", b, bufferIds[1][b], static_cast<unsigned long long>(block.length));
          nonZeroRows("param", block.kva, block.length < 0x2cab8 ? block.length : 0x2cab8, 300);
          block.release();
        }
      }
    }
    if (kinds[i] == 0x4569 && ring.valid()) {
      // A YUV request is answered once: the "frame" it asked for is made up (fakeFrame) before the notification.
      const unsigned position = static_cast<unsigned>(numbers[i] % 32);
      if (yuvNotified[position] == numbers[i] + 1) continue;
      yuvNotified[position] = static_cast<uint32_t>(numbers[i] + 1);
      uint8_t *yuvRecord = ring.kva + static_cast<size_t>(position) * 0x10630;
      if (!gEncode || !encodeFrame(yuvRecord)) fakeFrame(yuvRecord);
      if (!receiverDumped) {
        receiverDumped = true;
        dumpReceiver();
        uint8_t *record = ring.kva + static_cast<size_t>(position) * 0x10630;
        nonZeroRows("yuv record", record, 0x10630, 600);
      }
    }
    notify(numbers[i], 0, "request done");
  }
  if (n) {
    IOSleep(300);
    walkRecords("after", false, numbers, kinds, 32);
    bool yuvPending = false;
    for (unsigned i = 0; i < n; ++i) yuvPending |= kinds[i] == 0x4569;
    if (context && yuvPending && receiverDumps < 2) {
      // What the receiver's notification handler has left in the plug-in's own state: the flags it tests before it takes the YUV path, the per-slot tables
      // Start fills, and the table the frame-done code fills from the parameter block.
      ++receiverDumps;
      static const struct { size_t offset, length; const char *name; } regions[] = {
          {0x4dbe0, 0x20, "flags"}, {0x64b0, 0x10, "+64b8"},  {0xc630, 0xa0, "+c63d"},     {0x23f80, 0x20, "settings"}, {0x24280, 0x60, "settings+300"},
          {0x6588, 0x20, "out base"}, {0x6768, 0x10, "out size"}, {0x67e0, 0x20, "param"}, {0x69c0, 0x20, "third"},     {0x6ba0, 0x30, "table"},
          {0xc5a0, 0x40, "slice"}, {0x6678, 0x20, "out ref"}, {0x68d0, 0x20, "param ref"}};
      for (const auto &region : regions) {
        uint8_t bytes[0xa0] = {};
        if (!peekUser(ownerTask, context + region.offset, bytes, region.length)) {
          printf("rk3588-venc: ave2:   receiver +%#zx (%s): unreadable\n", region.offset, region.name);
          continue;
        }
        char name[48];
        snprintf(name, sizeof name, "receiver +%#zx %s", region.offset, region.name);
        hexLines(name, bytes, region.length);
      }
    }
  }
}

// A notification the way the original extension sends one: the plug-in's frame receiver (FrameDone(refcon, result, args, numArgs)) takes args[0] as the
// number of the record of its own ring that the request was filed under (the call itself does not carry it: the two sides count in step).
void AppleAVE2UserClient::notify(uint64_t counter, uint64_t second, const char *what) {
  if (!haveCallback) {
    printf("rk3588-venc: ave2: nobody listens: %s (counter %llu) not sent\n", what, static_cast<unsigned long long>(counter));
    return;
  }
  io_user_reference_t args[3] = {counter, second, 0};
  const IOReturn kr = sendAsyncResult64(asyncReference, kIOReturnSuccess, args, 3);
  printf("rk3588-venc: ave2:   -> notification %s: counter %llu, second %llu: %#x\n", what, static_cast<unsigned long long>(counter), static_cast<unsigned long long>(second), kr);
}

IOReturn AppleAVE2UserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *, OSObject *, void *) {
  if (!args) return kIOReturnBadArgument;
  const unsigned index = selector < kSelectors ? selector : kSelectors - 1;
  const unsigned call = ++calls[index];
  ++total;
  const bool full = gLog >= 2 || call <= 3;
  printf("rk3588-venc: ave2: %s: selector %u call %u%s: scalars in %u, struct in %u%s, scalars out %u, struct out %u%s%s\n", ownerName, selector, call,
         args->asyncWakePort ? " (asynchronous)" : "", args->scalarInputCount, args->structureInputSize, args->structureInputDescriptor ? " + descriptor" : "",
         args->scalarOutputCount, args->structureOutputSize, args->structureOutputDescriptor ? " + descriptor" : "",
         args->structureVariableOutputData ? " + variable output" : "");
  if (full) {
    for (uint32_t i = 0; i < args->scalarInputCount && i < 16; ++i) printf("rk3588-venc: ave2:   scalar %u = %#llx\n", i, static_cast<unsigned long long>(args->scalarInput[i]));
    if (args->structureInput && args->structureInputSize) {
      // The open call's structure is 1032 bytes with pointers and sizes past the first 512: print the non-zero rows of all of it.
      if (args->structureInputSize > 512) nonZeroRows("in ", static_cast<const uint8_t *>(args->structureInput), args->structureInputSize, 100);
      else hexLines("in ", args->structureInput, args->structureInputSize);
    }
    if (args->structureInputDescriptor) {
      IOMemoryDescriptor *descriptor = args->structureInputDescriptor;
      const uint64_t length = descriptor->getLength();
      printf("rk3588-venc: ave2:   input descriptor of %llu bytes\n", static_cast<unsigned long long>(length));
      // The descriptor belongs to the caller's task: wire it before reading (an unwired read panics: "IOGMD: not wired for the IODMACommand").
      if (length && length <= (1u << 20) && descriptor->prepare(kIODirectionOut) == kIOReturnSuccess) {
        uint8_t *copy = static_cast<uint8_t *>(IOMallocZero(static_cast<size_t>(length)));
        if (copy) {
          const uint64_t n = descriptor->readBytes(0, copy, length);
          nonZeroRows("descriptor", copy, static_cast<size_t>(n), 400);
          IOFree(copy, static_cast<size_t>(length));
        }
        descriptor->complete(kIODirectionOut);
      }
    }
    if (args->structureOutputDescriptor) {
      printf("rk3588-venc: ave2:   output descriptor of %llu bytes\n", static_cast<unsigned long long>(args->structureOutputDescriptor->getLength()));
    }
  }
  if (selector == 2 && args->asyncWakePort && args->structureInputSize >= 40) {
    // The plug-in's call carries a zeroed reference; the function to call and its context are in the structure (five 64-bit words: two it keeps for
    // itself, a flags word, the frame receiver's callback at +0x18 and the receiver at +0x20): build the reference the way the original extension does.
    const uint32_t *words = static_cast<const uint32_t *>(args->structureInput);
    const uint64_t callback = words[6] | (static_cast<uint64_t>(words[7]) << 32);
    const uint64_t context_ = words[8] | (static_cast<uint64_t>(words[9]) << 32);
    if (haveCallback) releaseAsyncReference64(asyncReference);
    bzero(asyncReference, sizeof asyncReference);
    setAsyncReference64(asyncReference, args->asyncWakePort, static_cast<mach_vm_address_t>(callback), static_cast<io_user_reference_t>(context_), ownerTask);
    haveCallback = true;
    context = static_cast<uint64_t>(context_);
    printf("rk3588-venc: ave2:   async reference %llx %llx %llx (call's own: %llx %llx, %u entries)\n", static_cast<unsigned long long>(asyncReference[0]),
           static_cast<unsigned long long>(asyncReference[1]), static_cast<unsigned long long>(asyncReference[2]), static_cast<unsigned long long>(args->asyncReference[0]),
           static_cast<unsigned long long>(args->asyncReference[1]), args->asyncReferenceCount);
  }
  for (uint32_t i = 0; i < args->scalarOutputCount; ++i) args->scalarOutput[i] = 0;
  if (args->structureOutput && args->structureOutputSize) bzero(args->structureOutput, args->structureOutputSize);
  if (selector == 0 && args->structureOutput && args->structureOutputSize >= 8) {
    // IO_Open: out[0] = the device type (the plug-in checks it against its platform table), out[1] = this client's number.
    const uint32_t reply[2] = {gDeviceType, ++gSessions};
    memcpy(args->structureOutput, reply, sizeof reply);
    printf("rk3588-venc: ave2:   -> device type %u, client %u\n", reply[0], reply[1]);
    if (args->structureInputSize >= 0x18) {
      const uint32_t ringId = field<uint32_t>(args->structureInput, 0x14);
      codec = field<uint32_t>(args->structureInput, 0x10) == 1 ? 2 : 1;
      printf("rk3588-venc: ave2:   codec %s\n", codec == 2 ? "H.265" : "H.264");
      ring.release();
      const bool ok = ring.open(ringId);
      printf("rk3588-venc: ave2:   record ring = IOSurface %u: %s (%llu bytes)\n", ringId, ok ? "mapped" : "not found", static_cast<unsigned long long>(ring.length));
    }
  }
  if (selector == 4 && args->structureOutput && args->structureOutputSize >= sizeof(uint32_t)) {
    // Start's 4-byte answer is how many frames the encoder keeps in flight: the plug-in sizes its two request queues ("Connection", a ring of packets) to it
    // (Connection::connectionSize, at most 16) and EncodeFrame waits for space in the first one. Zero (what the first runs returned) is a queue that is full from
    // the start: the first EncodeFrame then waited its 120 s and failed with -12912 (runs ave12..ave19: the waiting thread was inside Connection::writePacketBlock).
    const uint32_t frames = kBuffers;
    memcpy(args->structureOutput, &frames, sizeof frames);
    printf("rk3588-venc: ave2:   -> frames in flight %u\n", frames);
  }
  if (selector >= 3 && selector <= 8) completePending("request", call <= 2);
  return kIOReturnSuccess;
}

extern "C" void rk3588_venc_ave2_serve(void) {
  uint32_t value = 1;
  if (PE_parse_boot_argn("rk3588-venc-ave2", &value, sizeof value)) gLog = static_cast<int>(value);
  if (gLog <= 0) return;
  if (PE_parse_boot_argn("rk3588-venc-ave2-type", &value, sizeof value)) gDeviceType = value;
  if (PE_parse_boot_argn("rk3588-venc-ave2-owner", &value, sizeof value)) gSurfaceOwner = value != 0;
  if (PE_parse_boot_argn("rk3588-venc-ave2-encode", &value, sizeof value)) gEncode = value != 0;
  AppleAVE2Driver *service = OSTypeAlloc(AppleAVE2Driver);
  if (!service || !service->init() || !service->attach(IOService::getResourceService()) || !service->start(IOService::getResourceService())) {
    printf("rk3588-venc: ave2: could not make the service\n");
    return;
  }
  // Who may open a user client (the kernel's native_sandbox_grant compares process names up to 16 characters): the daemons that host VideoToolbox's encoders
  // and the test program (it runs as launchd's job com.apple.ptpd).
  service->setProperty("NativeSandboxGrantProcess", "videocodecd,mediaserverd,ptpd,mediaplaybackd,avconferenced,cameracaptured,assetsd");
  service->registerService();
  printf("rk3588-venc: ave2: service \"AppleAVE2Driver\" registered (logging stage)\n");
}
