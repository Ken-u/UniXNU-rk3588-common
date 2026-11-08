#!/usr/bin/env python3
"""Run the H264 encoder plug-in's frame-done handler (H264VideoEncoderFrameReceiver::FrameDone) on the host.

The plug-in's code is /tmp/ave/h264h9.bin (the image's text as it is in the shared cache, offset = address - image base).  The receiver object and the record
it will look at come from a board log (the stub's `rcv` / `yuv record` rows); everything else the code touches is zero until we say otherwise.  Calls out of
the image (into CoreFoundation, IOSurface, ...) land in a page of `ret` instructions: they are logged with their arguments and answered from a table.

usage: ave_emu.py LOGFILE [number] [--fake]    (--fake: seed what the stub's fakeFrame writes)    (run from a directory that has no dis.py / inspect.py shadowing the stdlib)
"""
import os, re, struct, sys
from unicorn import *
from unicorn.arm64_const import *

BIN = os.environ.get('AVE_PLUGIN_IMAGE', '/tmp/ave/h264h9.bin')  # the plug-in's text from the shared cache (not in the repository)
BASE = 0x211187000
FRAMEDONE = BASE + 0x34ce4
RECV = 0x300000000
RECV_SIZE = 0x60000
STACK = 0x7f0000000
STACK_SIZE = 0x100000
SENTINEL = 0x7e0000000
ISLAND = BASE + 0x3579000
ISLAND_SIZE = 0x2000
STUBS = (0x8b490, 0x8bec0)

NAMES = {
    0x3579de0: 'CFAllocatorAllocate', 0x3579f90: 'CFRelease', 0x357a050: 'CMBlockBufferGetDataPointer', 0x357a17c: 'CVPixelBufferLockBaseAddress',
    0x357a1f4: 'FigFormatDescriptionRelease', 0x357a26c: 'IOSurfaceGetAllocSize', 0x357a278: 'IOSurfaceGetBaseAddress', 0x357a308: 'IOSurfaceLock',
    0x357a314: 'IOSurfaceLookup', 0x357a32c: 'IOSurfaceUnlock', 0x357a338: 'VTEncoderSessionCreateCMBlockBuffer',
    0x3579e64: 'CFDataCreateMutable', 0x3579ea0: 'CFDataSetLength', 0x3579e88: 'CFDataGetMutableBytePtr', 0x3579ec4: 'CFDictionaryCreate',
    0x3579edc: 'CFDictionaryCreateMutable', 0x3579eac: 'CFDictionaryAddValue', 0x357a350: 'VTEncoderSessionCreateVideoFormatDescription',
    0x357a4d0: '__stack_chk_fail', 0x357a584: 'gettimeofday', 0x357a35c: 'VTEncoderSessionEmitEncodedFrame', 0x357a074: 'CMSampleBufferCreate',
    0x357a080: 'CMSampleBufferGetSampleAttachmentsArray', 0x357a098: 'CMTimeCompare', 0x3579e28: 'CFArrayGetValueAtIndex', 0x357a4e8: '__platform_bzero',
    0x8bd28: 'stub_8bd28', 0x8bd88: 'stub_8bd88', 0x8bd94: 'stub_8bd94', 0x8bc08: 'free',
    0x357a38c: 'VTMotionEstimationProcessorSessionCreateMotionVectorPixelBuffer', 0x357a500: 'clock_gettime_nsec_np', 0x357a590: 'kdebug_trace',
    0x357a5c0: 'malloc_type_malloc', 0x357a71c: 'setpriority', 0x357a7a0: 'syslog', 0x8bc80: 'memcpy',
}


def parse_log(path):
    mem = {}
    rec = {}
    for line in open(path, errors='replace'):
        m = re.search(r'ave2:\s+rcv ([0-9a-f]{6}): ((?:[0-9a-f]{2} ){16})', line)
        if m:
            mem[int(m.group(1), 16)] = bytes.fromhex(m.group(2).replace(' ', ''))
        m = re.search(r'ave2:\s+yuv record ([0-9a-f]{6}): ((?:[0-9a-f]{2} ){16})', line)
        if m:
            rec[int(m.group(1), 16)] = bytes.fromhex(m.group(2).replace(' ', ''))
    return mem, rec


class Emu:
    def __init__(self, logfile, number=2):
        self.mem, self.rec = parse_log(logfile)
        self.uc = Uc(UC_ARCH_ARM64, UC_MODE_ARM)
        uc = self.uc
        data = open(BIN, 'rb').read()
        size = (len(data) + 0xfff) & ~0xfff
        uc.mem_map(BASE, size)
        uc.mem_write(BASE, data)
        uc.mem_write(BASE + STUBS[0], struct.pack('<I', 0xd65f03c0) * ((STUBS[1] - STUBS[0]) // 4))
        self.text_end = BASE + size
        uc.mem_map(STACK, STACK_SIZE)
        uc.mem_map(SENTINEL, 0x1000)
        uc.mem_write(SENTINEL, struct.pack('<I', 0xd65f03c0))
        uc.mem_map(RECV, RECV_SIZE)
        for off, row in self.mem.items():
            if off + 16 <= RECV_SIZE:
                uc.mem_write(RECV + off, row)
        uc.mem_map(ISLAND, ISLAND_SIZE)
        uc.mem_write(ISLAND, struct.pack('<I', 0xd65f03c0) * (ISLAND_SIZE // 4))
        self.pages = set()
        self.calls = []
        self.writes = []
        self.trace = []
        self.prev = None
        self.number = number
        self.surf_base = 0x500000000
        self.next_obj = 0x800000000
        self.lookup_ids = {}
        uc.hook_add(UC_HOOK_MEM_READ_UNMAPPED | UC_HOOK_MEM_WRITE_UNMAPPED, self.unmapped)
        uc.hook_add(UC_HOOK_CODE, self.code, begin=ISLAND, end=ISLAND + ISLAND_SIZE)
        uc.hook_add(UC_HOOK_CODE, self.code, begin=BASE + STUBS[0], end=BASE + STUBS[1])
        uc.hook_add(UC_HOOK_CODE, self.jumps, begin=BASE, end=self.text_end)
        uc.hook_add(UC_HOOK_MEM_WRITE, self.write, begin=RECV, end=RECV + RECV_SIZE)
        self.setup_ring()

    def u64(self, addr):
        return struct.unpack('<Q', self.uc.mem_read(addr, 8))[0]

    def u32(self, addr):
        return struct.unpack('<I', self.uc.mem_read(addr, 4))[0]

    def put(self, addr, data):
        self.ensure(addr, len(data))
        self.uc.mem_write(addr, data)

    def ensure(self, addr, length):
        a = addr & ~0xfff
        while a < addr + length:
            if a not in self.pages and not self.mapped(a):
                self.uc.mem_map(a, 0x1000)
                self.pages.add(a)
            a += 0x1000

    def mapped(self, a):
        for lo, hi, _ in self.uc.mem_regions():
            if lo <= a <= hi:
                return True
        return False

    def unmapped(self, uc, access, address, size, value, user):
        self.ensure(address, size)
        return True

    def setup_ring(self):
        # [receiver + 0xc688] -> {u32 count, u64 base}; the base holds 32 records of 0x10630 bytes: the one under test comes from the log
        desc_ptr = self.u64(RECV + 0xc688)
        ring = 0x400000000
        self.put(desc_ptr, struct.pack('<IIQ', 32, 0, ring))
        self.ring = ring
        self.put(ring, b'\0' * (32 * 0x10630))
        rec = ring + (self.number % 32) * 0x10630
        for off, row in self.rec.items():
            self.uc.mem_write(rec + off, row)
        self.record = rec

    def write(self, uc, access, address, size, value, user):
        self.writes.append((address - RECV, size, value & ((1 << (8 * size)) - 1)))

    def jumps(self, uc, address, size, user):
        if self.prev is not None and address != self.prev + 4:
            self.trace.append((self.prev - BASE, address - BASE))
        self.prev = address

    def code(self, uc, address, size, user):
        off = address - BASE
        lr = uc.reg_read(UC_ARM64_REG_X30)
        args = [uc.reg_read(r) for r in (UC_ARM64_REG_X0, UC_ARM64_REG_X1, UC_ARM64_REG_X2, UC_ARM64_REG_X3, UC_ARM64_REG_X4, UC_ARM64_REG_X5)]
        name = NAMES.get(off, '?')
        ret = self.external(off, name, args)
        self.calls.append((lr - BASE, off, name, args, ret))
        if ret is not None:
            uc.reg_write(UC_ARM64_REG_X0, ret)

    def cstr(self, addr, limit=200):
        try:
            out = bytearray()
            while len(out) < limit:
                b = self.uc.mem_read(addr + len(out), 1)[0]
                if not b:
                    break
                out.append(b)
            return out.decode('latin1')
        except UcError:
            return '?'

    databuf = {}
    usec = 0

    def scratch(self, size=0x10000):
        p = self.next_obj
        self.next_obj += (size + 0xfff) & ~0xfff
        self.ensure(p, size)
        return p

    def external(self, off, name, a):
        if name == 'IOSurfaceLookup':
            return 0x600000000 + a[0]
        if name == 'IOSurfaceGetBaseAddress':
            return self.surf_base + (a[0] - 0x600000000) * 0x100000
        if name == 'IOSurfaceGetAllocSize':
            return 0x100000
        if name in ('IOSurfaceLock', 'IOSurfaceUnlock', 'CVPixelBufferLockBaseAddress', 'kdebug_trace', 'CFRelease', 'FigFormatDescriptionRelease', 'setpriority'):
            return 0
        if name == 'clock_gettime_nsec_np':
            return 0x1000000
        if name == 'syslog':
            print('   SYSLOG(%d): %s' % (a[0], self.cstr(a[1])))
            return 0
        if name == 'VTEncoderSessionCreateCMBlockBuffer':
            self.put(a[2], struct.pack('<Q', self.scratch()))
            return 0
        if name == 'CMBlockBufferGetDataPointer':
            self.put(a[4], struct.pack('<Q', self.scratch()))
            return 0
        if name in ('CFAllocatorAllocate', 'malloc_type_malloc'):
            return self.scratch(a[1] if name == 'CFAllocatorAllocate' else a[0])
        if name in ('CFDataCreateMutable', 'CFDictionaryCreate', 'CFDictionaryCreateMutable', 'CFArrayCreate'):
            return self.scratch(0x1000)
        if name == 'CFDataGetMutableBytePtr':
            return self.databuf.setdefault(a[0], self.scratch(0x1000))
        if name == 'VTEncoderSessionCreateVideoFormatDescription':
            self.put(a[5], struct.pack('<Q', self.scratch(0x1000)))
            return 0
        if name == 'gettimeofday':
            self.usec += 50000
            self.put(a[0], struct.pack('<QI', 1000 + self.usec // 1000000, self.usec % 1000000))
            return 0
        if name == 'VTEncoderSessionEmitEncodedFrame':
            print('   >>> VTEncoderSessionEmitEncodedFrame(session %#x, sample %#x, flags..)' % (a[0], a[1]))
            return 0
        if name == 'CMSampleBufferCreate':
            print('   >>> CMSampleBufferCreate args', ['%#x' % x for x in a])
            return 0
        if name == '__stack_chk_fail':
            print('   STACK CHECK FAILED')
            return 0
        if name == 'memcpy':
            n = a[2]
            if n > 0x400000:
                print('   memcpy too large: %#x bytes %#x <- %#x' % (n, a[0], a[1]))
                return a[0]
            self.ensure(a[0], n)
            self.ensure(a[1], n)
            self.uc.mem_write(a[0], bytes(self.uc.mem_read(a[1], n)))
            return a[0]
        return 0

    def fake_frame(self, slot=0, header_id=21):
        """what the stub's fakeFrame leaves behind: one slice, headers, in the slot's buffers"""
        block = self.u64(RECV + 0x67e0 + 8 * slot)
        out = self.u64(RECV + 0x6588 + 8 * slot)
        slice_ = bytes([0, 0, 0, 1, 0x65, 0x88, 0x84, 0, 0x33, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc])
        headers = bytes.fromhex('00000001674d001fda0280be40000000016 8ce3c80'.replace(' ', ''))
        self.put(out, slice_)
        self.put(block + 0x98, struct.pack('<I', len(headers) * 8))
        self.put(self.record + 0x5528, struct.pack('<I', len(headers) * 8))
        self.put(block + 0xf0, struct.pack('<I', 1))
        self.put(block + 0xf4, struct.pack('<QI', 0, 0))
        self.put(block + 0x180, struct.pack('<I', len(slice_)))
        self.put(block + 0x38c, bytes([4]))
        self.put(self.surf_base + header_id * 0x100000, headers)
        print('fake frame: block %#x out %#x' % (block, out))

    def run(self, limit=2000000):
        uc = self.uc
        args = 0x700000000
        self.put(args, struct.pack('<QQQ', self.number, 0, 0))
        uc.reg_write(UC_ARM64_REG_SP, STACK + STACK_SIZE - 0x1000)
        uc.reg_write(UC_ARM64_REG_X29, STACK + STACK_SIZE - 0x1000)
        uc.reg_write(UC_ARM64_REG_X30, SENTINEL)
        uc.reg_write(UC_ARM64_REG_X0, RECV)
        uc.reg_write(UC_ARM64_REG_X1, 0)
        uc.reg_write(UC_ARM64_REG_X2, args)
        uc.reg_write(UC_ARM64_REG_X3, 3)
        try:
            uc.emu_start(FRAMEDONE, SENTINEL, count=limit)
        except UcError as e:
            print('stopped:', e, 'pc', hex(uc.reg_read(UC_ARM64_REG_PC) - BASE))
        print('pc at end', hex(uc.reg_read(UC_ARM64_REG_PC) - BASE), 'x0', hex(uc.reg_read(UC_ARM64_REG_X0)))


if __name__ == '__main__':
    emu = Emu(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 2)
    if '--fake' in sys.argv:
        emu.fake_frame()
    emu.run()
    print('jumps:', len(emu.trace))
    for a, b in emu.trace:
        print('  %x -> %x' % (a, b))
    print('external calls:')
    for lr, off, name, args, ret in emu.calls:
        print('  from %x: %x %s(%s) -> %s' % (lr - 4, off, name, ', '.join('%#x' % x for x in args), ret if ret is None else '%#x' % ret))
    print('writes into the receiver:')
    for off, size, value in emu.writes[:80]:
        print('  +%#x (%d) = %#x' % (off, size, value))
    rec = emu.record
    print('record +0x00..: in use', emu.uc.mem_read(rec, 1)[0])
