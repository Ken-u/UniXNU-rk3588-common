#!/usr/bin/env python3
"""Which words of its record does the H264 encoder plug-in's frame-done handler read, and which slot's buffers does it take the frame from?

Runs ave_emu.py's emulation of H264VideoEncoderFrameReceiver::FrameDone (unicorn, the plug-in's text from the shared cache in /tmp/ave/h264h9.bin, the receiver
and the record from the `rcv` / `yuv record` rows of a board log), seeds a made-up finished frame in the buffers of slot FAKESLOT (what the stub's fakeFrame
writes), sets 32-bit words of the record, and prints the offsets of the record that the handler reads (those from 0x200 to 0x6000) and whether it got as far as
handing a sample to VideoToolbox.

usage: [FAKESLOT=N] ave_slot.py LOGFILE [OFFSET=VALUE ...]      OFFSET in hex, VALUE a number; e.g. FAKESLOT=3 ave_slot.py run.log 3a8=3

Result (2026-11-09, log of run sp3): the handler reads +0x3a4 and +0x3a8 and nothing at +0x46c; with +0x3a8 = 3 and the frame in slot 3 it emits the sample,
with +0x3a8 = 3 and the frame in slot 0 (or the 3 at +0x46c) it stops after reading the empty buffers of the wrong slot. The slot word is +0x3a8.
(Run it from a directory that has no dis.py / inspect.py shadowing the standard library.)
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ave_emu
from unicorn import UC_HOOK_MEM_READ

if len(sys.argv) < 2:
    sys.exit(__doc__)
words = {}
for arg in sys.argv[2:]:
    key, value = arg.split('=')
    words[int(key, 16)] = int(value, 0)

emu = ave_emu.Emu(sys.argv[1], 2)
emu.fake_frame(int(os.environ.get('FAKESLOT', '0')))
record = emu.record
for offset, value in words.items():
    emu.uc.mem_write(record + offset, struct.pack('<I', value))
reads = set()


def on_read(uc, access, address, size, value, user):
    reads.add(address - record)


emu.uc.hook_add(UC_HOOK_MEM_READ, on_read, begin=record, end=record + 0x10630)
emu.run()
print('record offsets read (0x200 to 0x6000): ' + ' '.join('%#x' % o for o in sorted(reads) if 0x200 <= o < 0x6000))
emitted = [c for c in emu.calls if c[2] == 'VTEncoderSessionEmitEncodedFrame']
print('VTEncoderSessionEmitEncodedFrame called: %s' % ('yes' if emitted else 'no'))
