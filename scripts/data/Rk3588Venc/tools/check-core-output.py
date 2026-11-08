#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""Decode what the encoder core's self-test (rk3588-venc=6, rk3588-venc-core-dump=1) printed and compare it with the pictures that were encoded.

    check-core-output.py SERIAL.log --decoder PATH-TO-avcdec [--hevc-decoder PATH-TO-hevcdec] [--name NAME ...] [--min-psnr 36] [--keep DIR]

The serial log has, for every recipe NAME, the line
    rk3588-venc: core NAME: H.264|H.265 WIDTHxHEIGHT, N pictures, ...   (older logs: without the codec, H.264)
the parameter sets and every picture as printed by the kernel (32 bytes to a line, "pic 65535" is the parameter sets that venc_core_header gave, the pictures are numbered from 0)
    rk3588-venc: core NAME pic N OFFSET HEX
and a summary line per picture
    rk3588-venc: core NAME pic N: B bytes ...
The stream handed to libavc (avcdec; libhevc's hevcdec for an H.265 recipe) is the parameter sets followed by the pictures. Every decoded picture is compared with the picture the kernel made (the pattern of
fillPicture in Rk3588VencCore.cpp, planar 4:2:0, picture number F): PSNR of luma and of both chroma planes. A lossy encoder cannot be checked for equality; a block that
wrote garbage, or the wrong picture, is far below what the quantiser allows. The floor for the luma is MIN_PSNR at QP 28 and 0.8 dB lower for every QP above it (the test
pattern is all edges and gradients: at the QP 49 a bit rate control may reach on a small picture the decoder gives 25 dB), the chroma may be 5 dB below that.
"""
import argparse
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import numpy
except ImportError:   # pure Python is slow at 1080p but right
    numpy = None


def pattern(width, height, frame):
    """The source picture of fillPicture(), planar 4:2:0, tightly packed (the decoder's output has no padding)."""
    if numpy is not None:
        y, x = numpy.mgrid[0:height, 0:width]
        luma = (x + 2 * y + 7 * frame + (((x >> 4) & (y >> 4)) * 9)) & 255
        cy, cx = numpy.mgrid[0:height // 2, 0:width // 2]
        u = 112 + ((3 * cx + 5 * frame) & 31)
        v = 112 + ((3 * cy + 7 * frame) & 31)
        return luma.astype(numpy.uint8).tobytes() + u.astype(numpy.uint8).tobytes() + v.astype(numpy.uint8).tobytes()
    luma = bytearray(width * height)
    for y in range(height):
        for x in range(width):
            luma[y * width + x] = (x + 2 * y + 7 * frame + (((x >> 4) & (y >> 4)) * 9)) & 255
    u = bytearray((width // 2) * (height // 2))
    v = bytearray(len(u))
    for y in range(height // 2):
        for x in range(width // 2):
            u[y * (width // 2) + x] = 112 + ((3 * x + 5 * frame) & 31)
            v[y * (width // 2) + x] = 112 + ((3 * y + 7 * frame) & 31)
    return bytes(luma + u + v)


def psnr(a, b):
    if numpy is not None:
        d = numpy.frombuffer(a, dtype=numpy.uint8).astype(numpy.int32) - numpy.frombuffer(b, dtype=numpy.uint8).astype(numpy.int32)
        mse = float((d * d).mean())
    else:
        mse = sum((x - y) ** 2 for x, y in zip(a, b)) / len(a)
    return 99.0 if mse == 0 else 10 * math.log10(255 * 255 / mse)


def collect(log, name):
    """(codec, width, height, pictures expected, {picture: {offset: bytes}}, {picture: byte count of the summary line}, {picture: QP}) of one recipe."""
    size = re.compile(r'rk3588-venc: core %s: (?:(H\.26[45]) )?(\d+)x(\d+), (\d+) pictures' % re.escape(name))
    line = re.compile(r'rk3588-venc: core %s pic (\d+) ([0-9a-f]{6}) ([0-9a-f]+)\s*$' % re.escape(name))
    summary = re.compile(r'rk3588-venc: core %s pic (\d+): (\d+) bytes.* qp (-?\d+)' % re.escape(name))
    codec = width = height = count = None
    chunks, lengths, qps = {}, {}, {}
    for raw in log.read_text(errors='replace').splitlines():
        m = size.search(raw)
        if m and width is None:
            codec, width, height, count = 'h265' if m.group(1) == 'H.265' else 'h264', int(m.group(2)), int(m.group(3)), int(m.group(4))
            continue
        m = line.search(raw)
        if m:
            chunks.setdefault(int(m.group(1)), {}).setdefault(int(m.group(2), 16), bytes.fromhex(m.group(3)))
            continue
        m = summary.search(raw)
        if m:
            lengths.setdefault(int(m.group(1)), int(m.group(2)))
            qps.setdefault(int(m.group(1)), int(m.group(3)))
    return codec, width, height, count, chunks, lengths, qps


def check(log, name, decoders, min_psnr, keep):
    codec, width, height, count, chunks, lengths, qps = collect(log, name)
    if width is None:
        print('%s: not in the log' % name)
        return False
    decoder = decoders[codec]
    if decoder is None:
        print('%s: %s, but no decoder for it was given' % (name, codec))
        return False
    stream = bytearray()
    for picture in [65535] + list(range(count)):
        have = chunks.get(picture, {})
        data = bytearray()
        for offset in sorted(have):
            if offset != len(data):
                print('%s picture %d: the dump has a gap at %#x' % (name, picture, offset))
                return False
            data += have[offset]
        want = lengths.get(picture) if picture != 65535 else None
        if picture != 65535 and (want is None or len(data) != want):
            print('%s picture %d: %d bytes dumped, %s written' % (name, picture, len(data), want))
            return False
        if picture == 65535 and not data:
            print('%s: no parameter sets in the log' % name)
            return False
        stream += data
    with tempfile.TemporaryDirectory() as tmp:
        work = (keep or Path(tmp)).resolve()
        work.mkdir(parents=True, exist_ok=True)
        extension = '.265' if codec == 'h265' else '.h264'
        (work / (name + extension)).write_bytes(stream)
        config = work / (name + '.dec.cfg')
        text = ('--input %s\n--save_output 1\n--num_frames -1\n--output %s\n--chroma_format YUV_420P\n--share_display_buf 0\n--num_cores 1\n--loopback 0\n'
                '--display 0\n--arch X86_GENERIC\n--soc GENERIC\n' % (work / (name + extension), work / (name + '.dec.yuv')))
        text += '--max_wd 4096\n--max_ht 2304\n--max_level 62\n' if codec == 'h265' else '--fps 30\n'
        config.write_text(text)
        subprocess.run([str(decoder), '-c' if codec == 'h265' else '--config', str(config)], stdout=subprocess.DEVNULL, check=True)
        decoded = (work / (name + '.dec.yuv')).read_bytes()
    frame = width * height * 3 // 2
    pictures = len(decoded) // frame
    print('%s: %s %dx%d, %d bytes of stream, %d pictures decoded (%d expected)' % (name, codec, width, height, len(stream), pictures, count))
    bad = pictures != count or len(decoded) % frame
    for i in range(min(pictures, count)):
        d, s = decoded[i * frame:(i + 1) * frame], pattern(width, height, i)
        y = width * height
        c = (width // 2) * (height // 2)
        values = (psnr(d[:y], s[:y]), psnr(d[y:y + c], s[y:y + c]), psnr(d[y + c:], s[y + c:]))
        qp = qps[i]
        if codec == 'h265':
            # MPP's H.265 front end reports the quantiser sum of the picture's 8x8 blocks divided by the number of blocks in whole 64x64 units (hal_h265e_vepu580.c: quality_real =
            # qp_sum / mb8_num): a picture that does not fill its units reports less than it used (176x144: 9 units, 0.69 of them filled, the picture coded at QP 38 says 26: counted back here)
            qp = int(round(qp * ((width + 63) // 64 * 64) * ((height + 63) // 64 * 64) / (((width + 7) // 8 * 8) * ((height + 7) // 8 * 8))))
        floor = min_psnr - 0.8 * max(0, qp - 28)
        ok = values[0] >= floor and min(values) >= floor - 5
        bad = bad or not ok
        print('  picture %d: %d bytes, qp %d%s, PSNR Y %.2f U %.2f V %.2f dB (floor %.1f): %s' % (i, lengths[i], qps[i], ' (%d with the partial units counted)' % qp if qp != qps[i] else '', values[0], values[1], values[2], floor, 'ok' if ok else 'BAD'))
    print('%s: %s' % (name, 'OK' if not bad else 'FAILED'))
    return not bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', type=Path)
    ap.add_argument('--decoder', required=True, type=Path, help='libavc avcdec (H.264 recipes)')
    ap.add_argument('--hevc-decoder', type=Path, help='libhevc hevcdec (H.265 recipes)')
    ap.add_argument('--name', action='append', help='a recipe of the log (default: all that have a size line)')
    ap.add_argument('--min-psnr', type=float, default=36.0, help='luma floor at QP 28 (lower by 0.8 dB per QP above)')
    ap.add_argument('--keep', type=Path, help='keep the reassembled bit streams and the decoder output in this directory')
    args = ap.parse_args()
    names = args.name or list(dict.fromkeys(re.findall(r'rk3588-venc: core (\S+): (?:H\.26[45] )?\d+x\d+, \d+ pictures', args.log.read_text(errors='replace'))))
    if not names:
        sys.exit('no recipe in %s' % args.log)
    decoders = {'h264': args.decoder, 'h265': args.hevc_decoder}
    results = [check(args.log, name, decoders, args.min_psnr, args.keep) for name in names]
    print('%d of %d recipes OK' % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
