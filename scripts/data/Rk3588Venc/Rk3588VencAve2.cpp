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
constexpr size_t kRootTable = 208, kRootCount = 216, kRootLock = 312, kSurfId = 16, kSurfMemory = 48;
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
OSObject *createDataSurface(uint64_t size, uint32_t *id) {
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
  OSObject *surface = create(root, nullptr, properties);
  properties->release();
  if (surface) *id = field<uint32_t>(surface, kSurfId);
  return surface;
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
  unsigned yuvDumps = 0;
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
      printf("rk3588-venc: ave2:   ring (%s): record %u: in use, number %u, kind %#x\n", when, i, number, kind);
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
        buffers[set][i] = createDataSurface(sizes[set], &bufferIds[set][i]);
        printf("rk3588-venc: ave2:   buffer %u.%u: %llu bytes, surface id %u%s\n", set, i, static_cast<unsigned long long>(sizes[set]), bufferIds[set][i], buffers[set][i] ? "" : " (failed)");
      }
    buffersMade = true;
  }
  const uint32_t count = kBuffers;
  memcpy(record + 0x170, &count, sizeof count);
  static const size_t offsets[3] = {0x174, 0x1ec, 0x264};
  for (unsigned set = 0; set < 3; ++set)
    for (unsigned i = 0; i < kBuffers; ++i) memcpy(record + offsets[set] + 4 * i, &bufferIds[set][i], sizeof(uint32_t));
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
    notify(numbers[i], 0, "request done");
  }
  if (n) {
    IOSleep(300);
    walkRecords("after", false, numbers, kinds, 32);
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
