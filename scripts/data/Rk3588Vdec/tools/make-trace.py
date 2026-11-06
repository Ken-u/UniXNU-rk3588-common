#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""Assemble the decode-job recording of tools/trace-shim.c into the compact trace the board replays.

Input : the directory the shim wrote (jobNNN.json, bufB_jobNNN.bin), the reference decode as I420 (libavc's output, bit-exact for H.264), width, height.
Output: <out>.vdt (binary) and <out>.inc (the same bytes as a C array).

Binary layout (all words little endian):
  header  'VDT1', width, height, nBuf, nJob
  sizes   nBuf words: the size of buffer id 1..nBuf (0 = not used)
  per job:
    regs[360]                                   the register image as MPP wrote it (address registers still hold fd | offset << 10: do not use them)
    nRun, then nRun x { first, count }          the register ranges MPP wrote (the board writes these, in this order, and only these)
    nReloc, then nReloc x { reg, buf, offset }  address register `reg` = bus address of buffer `buf` + offset
    nUpload, then nUpload x { buf, offset, len } followed by the bytes (padded to 4): CPU-written content that changed since the job before
    outBuf, outOffset, outLen, outCrc           the decoded frame (NV12, luma plane then interleaved chroma) and the CRC-32 it must have
"""
import argparse, glob, json, os, struct, sys, zlib

NREGS = 360
PAGE = 4096


def i420_to_nv12(frame, w, h):
    y = frame[:w * h]
    u = frame[w * h:w * h + w * h // 4]
    v = frame[w * h + w * h // 4:]
    uv = bytearray(w * h // 2)
    uv[0::2] = u
    uv[1::2] = v
    return bytes(y) + bytes(uv)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trace_dir')
    ap.add_argument('--ref', required=True, help='reference decode, I420 frames in output order')
    ap.add_argument('--width', type=int, required=True)
    ap.add_argument('--height', type=int, required=True)
    ap.add_argument('--jobs', type=int, required=True, help='number of jobs to keep (= frames, no B frames)')
    ap.add_argument('--out', required=True, help='output path without extension')
    ap.add_argument('--mpp-out', help='the pictures MPP handed out (NV12, display order) with the shim\'s job marker in their first eight bytes; without it the '
                                      'jobs are taken to be in display order (no B pictures)')
    a = ap.parse_args()

    w, h = a.width, a.height
    frame_size = w * h * 3 // 2
    ref = open(a.ref, 'rb').read()
    if len(ref) < a.jobs * frame_size:
        sys.exit('reference too short: %d bytes for %d frames of %d' % (len(ref), a.jobs, frame_size))

    jobs = []
    for k in range(a.jobs):
        jobs.append(json.load(open(os.path.join(a.trace_dir, 'job%03d.json' % k))))

    # display position of each job's picture (the reference decoder writes display order, the jobs run in decoding order)
    position = {k: k for k in range(a.jobs)}
    if a.mpp_out:
        pics = open(a.mpp_out, 'rb').read()
        position = {}
        for d in range(len(pics) // frame_size):
            m = pics[d * frame_size:d * frame_size + 8]
            if m[:2] != b'JK' or m[6:8] != b'\xa5\x5a':
                sys.exit('picture %d of %s carries no job marker' % (d, a.mpp_out))
            k = m[2] | m[3] << 8
            if (m[4] | m[5] << 8) != (~k & 0xffff):
                sys.exit('picture %d of %s: damaged marker' % (d, a.mpp_out))
            position[k] = d
        missing = [k for k in range(a.jobs) if k not in position]
        if missing:
            sys.exit('no picture came out for job(s) %s' % missing)

    sizes = {}
    for j in jobs:
        for r in j['relocs']:
            sizes[r['buf']] = r['size']
    nbuf = max(sizes)

    model = {b: bytearray(sizes[b]) for b in sizes}   # what the board's copy of each buffer holds from the CPU's writes
    out = bytearray()
    out += b'VDT1' + struct.pack('<4I', w, h, nbuf, a.jobs)
    for b in range(1, nbuf + 1):
        out += struct.pack('<I', sizes.get(b, 0))

    stats = []
    for k, j in enumerate(jobs):
        regs = j['regs']
        assert len(regs) == NREGS
        out += struct.pack('<%dI' % NREGS, *regs)
        written = sorted(set(j['set']))
        runs = []
        for r in written:
            if runs and runs[-1][0] + runs[-1][1] == r:
                runs[-1][1] += 1
            else:
                runs.append([r, 1])
        out += struct.pack('<I', len(runs))
        for first, count in runs:
            out += struct.pack('<2I', first, count)
        relocs = j['relocs']
        out += struct.pack('<I', len(relocs))
        for r in relocs:
            out += struct.pack('<3I', r['reg'], r['buf'], r['off'])
        # uploads: the pages of each referenced buffer that differ from what the board already has
        uploads = []
        for b in sorted(set(r['buf'] for r in relocs)):
            path = os.path.join(a.trace_dir, 'buf%d_job%03d.bin' % (b, k))
            if not os.path.exists(path):
                continue
            data = bytearray(open(path, 'rb').read())
            assert len(data) == sizes[b], (path, len(data), sizes[b])
            for q in range(k):   # the markers of the jobs before this one (the shim writes a job's marker after it has dumped that job's buffers)
                r130 = [r for r in jobs[q]['relocs'] if r['reg'] == 130][0]
                if r130['buf'] == b:
                    data[r130['off']:r130['off'] + 8] = bytes(8)
            data = bytes(data)
            m = model[b]
            p = 0
            while p < len(data):
                if data[p:p + PAGE] != m[p:p + PAGE]:
                    q = p
                    while q < len(data) and data[q:q + PAGE] != m[q:q + PAGE]:
                        q += PAGE
                    chunk = data[p:q]
                    # trim trailing zero pages that equal the model anyway (already excluded) and keep the chunk
                    uploads.append((b, p, chunk))
                    m[p:q] = chunk
                    p = q
                else:
                    p += PAGE
        out += struct.pack('<I', len(uploads))
        up_bytes = 0
        for b, off, chunk in uploads:
            out += struct.pack('<3I', b, off, len(chunk)) + chunk + b'\0' * (-len(chunk) % 4)
            up_bytes += len(chunk)
        rel130 = [r for r in relocs if r['reg'] == 130][0]
        d = position[k]
        nv12 = i420_to_nv12(ref[d * frame_size:(d + 1) * frame_size], w, h)
        out += struct.pack('<4I', rel130['buf'], rel130['off'], len(nv12), zlib.crc32(nv12) & 0xffffffff)
        stats.append((k, d, len(relocs), len(uploads), up_bytes, hex(zlib.crc32(nv12) & 0xffffffff)))

    open(a.out + '.vdt', 'wb').write(out)
    with open(a.out + '.inc', 'w') as f:
        f.write('// generated by make-trace.py: %d jobs, %d x %d, %d bytes\n' % (a.jobs, w, h, len(out)))
        f.write('static const unsigned char kVdecTrace[] = {\n')
        for i in range(0, len(out), 24):
            f.write(','.join('0x%02x' % c for c in out[i:i + 24]) + ',\n')
        f.write('};\n')
    for s in stats:
        print('job %d (display position %d): %d relocs, %d upload chunks (%d bytes), expected crc %s' % s)
    print('wrote %s.vdt (%d bytes) and %s.inc' % (a.out, len(out), a.out))


main()
