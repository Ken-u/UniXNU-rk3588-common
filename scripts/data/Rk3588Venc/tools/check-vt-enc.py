#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""Decode what VideoToolbox's hardware H.264 (or, with --codec h265, H.265) encoder delivered on the board and compare it with the pictures the test program gave it.

    check-vt-enc.py SERIAL.log --decoder PATH-TO-avcdec|hevcdec [--codec h264|h265] [--width 640] [--height 360] [--frames 5] [--min-psnr 30] [--tag k_] [--keep DIR]

The test program (Rk3588Vdec/usertest/vt_enc.h) prints every compressed sample of the first H.264 session and the session's parameter sets as rows
    vt-enc: data NAME OFFSET HEX            (NAME: set0, set1, sample0, sample1k ... ; "k" = the sample was marked sync; an H.265 session's names start with h265_, its sets are VPS, SPS, PPS)
The samples are in the length-prefixed form of the MP4 world (4-byte NAL lengths); this puts them back into an Annex B stream with the parameter sets in front, has
libavc's avcdec decode it and compares picture N with the picture the test generated for frame N (fill_nv12 of vt_enc.h, rebuilt here) as planar 4:2:0: PSNR of
luma and both chroma planes. The encoder is lossy (a trace's fixed quantiser), so this is a check against garbage and the wrong picture, not against equality.
"""
import argparse
import math
import re
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path


def psnr(a, b):
    mse = sum((x - y) ** 2 for x, y in zip(a, b)) / len(a)
    return 99.0 if mse == 0 else 10 * math.log10(255 * 255 / mse)


def source_picture(width, height, frame):
    """fill_nv12() of vt_enc.h, as planar 4:2:0 (Y, then Cb, then Cr)."""
    y = bytearray(width * height)
    for row in range(height):
        base = row * width
        for col in range(width):
            y[base + col] = 16 + ((col + 2 * frame) * 3 // 2 + row) % 200
    cb = bytearray((width // 2) * (height // 2))
    cr = bytearray((width // 2) * (height // 2))
    for row in range(height // 2):
        for col in range(width // 2):
            cb[row * (width // 2) + col] = 100 + (col + frame) % 50
            cr[row * (width // 2) + col] = 120 + (row + 2 * frame) % 40
    return bytes(y) + bytes(cb) + bytes(cr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', type=Path)
    ap.add_argument('--decoder', required=True, type=Path, help='libavc avcdec, or libhevc hevcdec with --codec h265')
    ap.add_argument('--codec', choices=('h264', 'h265'), default='h264')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=360)
    ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--min-psnr', type=float, default=30.0)
    ap.add_argument('--tag', default='', help='the session to check: the prefix of its data rows (the settings struct of the session in vt_enc.h)')
    ap.add_argument('--keep', type=Path)
    args = ap.parse_args()

    row_re = re.compile(r'vt-enc: data (\w+) ([0-9a-f]{6}) ([0-9a-f]+)\s*$')
    # Every sample is dumped several times (two passes, and the dump itself is printed twice), and the serial line can lose
    # or mix characters when two threads print at once: for each row the copy that most of the copies agree on is used.
    votes = {}
    for raw in args.log.read_text(errors='replace').splitlines():
        m = row_re.search(raw)
        if m and len(m.group(3)) % 2 == 0:
            votes.setdefault((m.group(1), int(m.group(2), 16)), Counter())[bytes.fromhex(m.group(3))] += 1
    chunks = {}
    damaged = 0
    for (name, offset), copies in votes.items():
        best, n = copies.most_common(1)[0]
        damaged += sum(copies.values()) - n
        chunks.setdefault(name, {})[offset] = best
    if damaged:
        print(f'{damaged} damaged copies of data rows in the log outvoted')

    def blob(name):
        have = chunks[name]
        data = bytearray()
        for offset in sorted(have):
            if offset != len(data):
                sys.exit('%s: the dump has a gap at %#x' % (name, offset))
            data += have[offset]
        return bytes(data)

    prefix = args.tag + ('h265_' if args.codec == 'h265' else '')
    samples = sorted((int(re.match(prefix + r'sample(\d+)', n).group(1)), n) for n in chunks if n.startswith(prefix + 'sample'))
    if len(samples) < args.frames:
        sys.exit('the log has %d samples, %d expected' % (len(samples), args.frames))
    start = b'\x00\x00\x00\x01'
    stream = bytearray()
    for name in (prefix + 'set0', prefix + 'set1', prefix + 'set2'):
        if name in chunks:
            stream += start + blob(name)
    for index, name in samples[:args.frames]:
        data = blob(name)
        at = 0
        types = []
        while at + 4 <= len(data):
            n = int.from_bytes(data[at:at + 4], 'big')
            nal = data[at + 4:at + 4 + n]
            if n == 0 or at + 4 + n > len(data):
                sys.exit('%s: bad NAL length %d at %d of %d' % (name, n, at, len(data)))
            types.append((nal[0] >> 1) & 0x3f if args.codec == 'h265' else nal[0] & 0x1f)
            stream += start + nal
            at += 4 + n
        print('%s: %d bytes, NAL types %s' % (name, len(data), types))

    with tempfile.TemporaryDirectory() as tmp:
        work = (args.keep or Path(tmp)).resolve()
        work.mkdir(parents=True, exist_ok=True)
        extension = '.265' if args.codec == 'h265' else '.h264'
        (work / ('vt' + extension)).write_bytes(stream)
        text = '--input %s\n--save_output 1\n--num_frames -1\n--output %s\n--chroma_format YUV_420P\n--share_display_buf 0\n--num_cores 1\n' \
               '--loopback 0\n--display 0\n--arch X86_GENERIC\n--soc GENERIC\n' % (work / ('vt' + extension), work / 'vt.dec.yuv')
        text += '--max_wd 4096\n--max_ht 2304\n--max_level 62\n' if args.codec == 'h265' else '--fps 30\n'
        (work / 'vt.dec.cfg').write_text(text)
        subprocess.run([str(args.decoder)] + (['-c'] if args.codec == 'h265' else ['--config']) + [str(work / 'vt.dec.cfg')], stdout=subprocess.DEVNULL, check=True)
        decoded = (work / 'vt.dec.yuv').read_bytes()

    frame = args.width * args.height * 3 // 2
    count = len(decoded) // frame
    print('%d bytes of stream, %d pictures decoded (%d expected)' % (len(stream), count, args.frames))
    bad = count != args.frames
    y = args.width * args.height
    c = y // 4
    for i in range(min(count, args.frames)):
        d, s = decoded[i * frame:(i + 1) * frame], source_picture(args.width, args.height, i)
        values = (psnr(d[:y], s[:y]), psnr(d[y:y + c], s[y:y + c]), psnr(d[y + c:], s[y + c:]))
        ok = values[0] >= args.min_psnr and min(values) >= args.min_psnr - 5
        bad = bad or not ok
        print('picture %d: PSNR Y %.2f U %.2f V %.2f dB: %s' % (i, values[0], values[1], values[2], 'ok' if ok else 'BAD'))
    print('vt-enc: %s' % ('ALL OK' if not bad else 'FAILED'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
