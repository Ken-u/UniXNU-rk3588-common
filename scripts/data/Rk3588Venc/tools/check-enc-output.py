#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""Decode what the RKVENC0 self-test printed and compare it with the pictures that were encoded.

    check-enc-output.py SERIAL.log TRACE.vet SOURCE.yuv WIDTH HEIGHT --decoder PATH [--codec h264|h265] [--name NAME] [--min-psnr 35] [--keep DIR]

The serial log has, for the trace NAME (default: the trace file's name), a line per job
    rk3588-venc: NAME: job N: ok status 0x1 stream L bytes in T us, crc 0x... of M bytes
and the bytes the block wrote, 32 to a line,
    rk3588-venc: stream NAME job N OFFSET HEX
The SPS and PPS are not the block's (the encoder library writes them, in the CPU-written start of the stream buffer, which the block then overwrites): they are
taken from the trace (the head of the stream buffer of job 0, and for the jobs whose address register says the block starts after a parameter set that MPP put
there, those bytes). The bit stream handed to the decoder is the first, then for every job its prefix and the L bytes the block wrote.
libavc (avcdec, --codec h264) or libhevc (hevcdec, --codec h265) decodes it; every decoded picture is compared with the picture of the same index in SOURCE.yuv (planar 4:2:0, WIDTH x HEIGHT): PSNR of luma
and of both chroma planes. A lossy encoder cannot be checked for equality; a block that wrote garbage, or the wrong picture, is far below 30 dB.
"""
import argparse
import math
import re
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


def parse_trace(path):
    """(width, height, [(prefix, extradata)] per job): for every job the CPU-written bytes of the stream buffer in front of where the block starts writing
    (a parameter set that MPP put before the slice: the packet starts with them), and for job 0 the head of the buffer when the block starts at offset 0
    (the SPS and PPS MPP wrote for the header request: the extradata of the stream)."""
    data = path.read_bytes()
    if data[:4] != b'VET1':
        sys.exit('%s is not a VET1 trace' % path)
    width, height, nbuf, njob = struct.unpack_from('<4I', data, 4)
    sizes = struct.unpack_from('<%dI' % nbuf, data, 20)
    at = 20 + 4 * nbuf
    model = {b + 1: bytearray(sizes[b]) for b in range(nbuf)}
    extent = {b + 1: 0 for b in range(nbuf)}   # how far into each buffer the CPU has written
    jobs = []
    for job in range(njob):
        nreg, = struct.unpack_from('<I', data, at)
        at += 4 + 8 * nreg
        nrel, = struct.unpack_from('<I', data, at)
        at += 4 + 12 * nrel
        nup, = struct.unpack_from('<I', data, at)
        at += 4
        for _ in range(nup):
            buf, offset, length = struct.unpack_from('<III', data, at)
            at += 12
            model[buf][offset:offset + length] = data[at:at + length]
            extent[buf] = max(extent[buf], offset + length)
            at += (length + 3) & ~3
        out_buf, out_offset, out_len = struct.unpack_from('<III', data, at)
        at += 12
        prefix = bytes(model[out_buf][:out_offset])
        extradata = bytes(model[out_buf][:extent[out_buf]]) if job == 0 and out_offset == 0 else b''
        jobs.append((prefix, extradata))
    return width, height, jobs


def psnr(a, b):
    mse = sum((x - y) ** 2 for x, y in zip(a, b)) / len(a)
    return 99.0 if mse == 0 else 10 * math.log10(255 * 255 / mse)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', type=Path)
    ap.add_argument('trace', type=Path)
    ap.add_argument('source', type=Path)
    ap.add_argument('width', type=int)
    ap.add_argument('height', type=int)
    ap.add_argument('--decoder', '--avcdec', dest='decoder', required=True, type=Path, help='libavc avcdec or libhevc hevcdec')
    ap.add_argument('--codec', choices=('h264', 'h265'), default='h264')
    ap.add_argument('--name')
    ap.add_argument('--min-psnr', type=float, default=35.0)
    ap.add_argument('--keep', type=Path, help='keep the reassembled bit stream and the decoder output in this directory')
    args = ap.parse_args()
    name = args.name or args.trace.stem

    _, _, trace_jobs = parse_trace(args.trace)
    njob = len(trace_jobs)
    lens, chunks = {}, {}
    job_re = re.compile(r'rk3588-venc: %s: job (\d+): (ok|FAILED) status (0x[0-9a-f]+) stream (\d+) bytes' % re.escape(name))
    line_re = re.compile(r'rk3588-venc: stream %s job (\d+) ([0-9a-f]{6}) ([0-9a-f]+)\s*$' % re.escape(name))
    for raw in args.log.read_text(errors='replace').splitlines():
        m = job_re.search(raw)
        if m:
            if m.group(2) != 'ok':
                sys.exit('job %s failed (status %s)' % (m.group(1), m.group(3)))
            lens.setdefault(int(m.group(1)), int(m.group(4)))   # the first pass's
            continue
        m = line_re.search(raw)
        if m:
            job, offset = int(m.group(1)), int(m.group(2), 16)
            chunks.setdefault(job, {}).setdefault(offset, bytes.fromhex(m.group(3)))
    if sorted(lens) != list(range(njob)):
        sys.exit('the log has jobs %s of %d' % (sorted(lens), njob))

    stream = bytearray(trace_jobs[0][1])
    for job in range(njob):
        have = chunks.get(job, {})
        data = bytearray()
        for offset in sorted(have):
            if offset != len(data):
                sys.exit('job %d: the dump has a gap at %#x' % (job, offset))
            data += have[offset]
        if len(data) < lens[job]:
            sys.exit('job %d: %d bytes dumped, %d written' % (job, len(data), lens[job]))
        print('job %d: %d bytes from the block, starting %s' % (job, lens[job], data[:8].hex()))
        stream += trace_jobs[job][0] + data[:lens[job]]

    with tempfile.TemporaryDirectory() as tmp:
        work = (args.keep or Path(tmp)).resolve()
        work.mkdir(parents=True, exist_ok=True)
        extension = '.h264' if args.codec == 'h264' else '.265'
        (work / (name + extension)).write_bytes(stream)
        config = work / (name + '.dec.cfg')
        text = '--input %s\n--save_output 1\n--num_frames -1\n--output %s\n--chroma_format YUV_420P\n--share_display_buf 0\n--num_cores 1\n' \
               '--loopback 0\n--display 0\n--arch X86_GENERIC\n--soc GENERIC\n' % (work / (name + extension), work / (name + '.dec.yuv'))
        text += '--fps 30\n' if args.codec == 'h264' else '--max_wd 4096\n--max_ht 2304\n--max_level 62\n'
        config.write_text(text)
        subprocess.run([str(args.decoder)] + (['--config', str(config)] if args.codec == 'h264' else ['-c', str(config)]), stdout=subprocess.DEVNULL, check=True)
        decoded = (work / (name + '.dec.yuv')).read_bytes()

    frame = args.width * args.height * 3 // 2
    source = args.source.read_bytes()
    count = len(decoded) // frame
    print('%d bytes of stream, %d pictures decoded (%d expected)' % (len(stream), count, njob))
    bad = count != njob
    for i in range(min(count, njob)):
        d, s = decoded[i * frame:(i + 1) * frame], source[i * frame:(i + 1) * frame]
        y = args.width * args.height
        c = y // 4
        values = (psnr(d[:y], s[:y]), psnr(d[y:y + c], s[y:y + c]), psnr(d[y + c:], s[y + c:]))
        ok = values[0] >= args.min_psnr and min(values) >= args.min_psnr - 5
        bad = bad or not ok
        print('picture %d: PSNR Y %.2f U %.2f V %.2f dB: %s' % (i, values[0], values[1], values[2], 'ok' if ok else 'BAD'))
    print('%s: %s' % (name, 'ALL OK' if not bad else 'FAILED'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
