#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""Checks a decoder's I420 output against the decoded picture hash SEI messages inside an H.265 stream.

The libhevc encoder writes, after each picture, a suffix SEI (NAL type 40, payload 132) with a checksum of its own reconstruction (--sei_hash_flags 3).
The decoder that gives the reference pictures of a trace (libhevc's hevcdec) does not check it, so this does: the checksums of every output frame must be
exactly the checksums in the stream (as sets: the SEI messages are in decoding order, the output is in display order). A trace whose reference pictures
fail this is not recorded.

usage: check-hevc-sei.py STREAM.265 DECODED.yuv WIDTH HEIGHT
"""
import sys
import numpy as np


def nals(data):
    """Annex B NAL units without start codes."""
    pos = []
    i = 0
    while True:
        i = data.find(b'\x00\x00\x01', i)
        if i < 0:
            break
        pos.append(i + 3)
        i += 3
    for n, p in enumerate(pos):
        end = pos[n + 1] - 3 if n + 1 < len(pos) else len(data)
        yield data[p:end].rstrip(b'\x00') if n + 1 < len(pos) else data[p:end]


def unescape(b):
    out = bytearray()
    zeros = 0
    for c in b:
        if zeros >= 2 and c == 3:
            zeros = 0
            continue
        out.append(c)
        zeros = zeros + 1 if c == 0 else 0
    return bytes(out)


def hashes_in_stream(data):
    found = []
    for nal in nals(data):
        if len(nal) < 3 or (nal[0] >> 1) & 0x3f != 40:   # suffix SEI
            continue
        rbsp = unescape(nal[2:])
        i = 0
        while i + 2 <= len(rbsp) and rbsp[i] != 0x80:
            ptype = 0
            while rbsp[i] == 0xff:
                ptype += 255
                i += 1
            ptype += rbsp[i]
            i += 1
            psize = 0
            while rbsp[i] == 0xff:
                psize += 255
                i += 1
            psize += rbsp[i]
            i += 1
            payload = rbsp[i:i + psize]
            i += psize
            if ptype == 132:
                kind = payload[0]
                if kind != 2:
                    sys.exit('the stream carries picture hash type %d; only the checksum (2) is checked here' % kind)
                found.append(tuple(int.from_bytes(payload[1 + 4 * c:5 + 4 * c], 'big') for c in range(3)))
    return found


def checksum(plane):
    h, w = plane.shape
    y = np.arange(h, dtype=np.uint32)[:, None]
    x = np.arange(w, dtype=np.uint32)[None, :]
    mask = (x & 0xff) ^ (y & 0xff) ^ (x >> 8) ^ (y >> 8)
    return int(((plane.astype(np.uint32) & 0xff) ^ mask).sum(dtype=np.uint64)) & 0xffffffff


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    stream = open(sys.argv[1], 'rb').read()
    raw = np.fromfile(sys.argv[2], dtype=np.uint8)
    w, h = int(sys.argv[3]), int(sys.argv[4])
    frame = w * h * 3 // 2
    if len(raw) % frame:
        sys.exit('the decoded file is not a whole number of frames')
    want = sorted(hashes_in_stream(stream))
    got = []
    for f in range(len(raw) // frame):
        b = raw[f * frame:(f + 1) * frame]
        got.append((checksum(b[:w * h].reshape(h, w)), checksum(b[w * h:w * h * 5 // 4].reshape(h // 2, w // 2)),
                    checksum(b[w * h * 5 // 4:].reshape(h // 2, w // 2))))
    got.sort()
    if not want:
        sys.exit('no picture hash SEI in the stream (encode with --sei_enable_flags 1 --sei_hash_flags 3)')
    if want != got:
        print('stream: %d hashes, decoder output: %d frames' % (len(want), len(got)))
        sys.exit('the decoder output does not match the stream\'s picture hashes')
    print('%d frames: every checksum equals the stream\'s decoded picture hash' % len(got))


main()
