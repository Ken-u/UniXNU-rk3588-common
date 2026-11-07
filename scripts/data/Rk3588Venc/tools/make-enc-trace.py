#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""Pack the encode jobs recorded by tools/trace-shim-enc.c into a VET1 trace (the layout is in VencTrace.h).

    make-enc-trace.py RECORDING_DIR WIDTH HEIGHT OUT.vet [--bsbs-reg 175]

RECORDING_DIR holds jobNNN.json (registers, relocations) and bufB_jobNNN.bin (buffer B as the CPU had written it, when it changed) of one recording. The
page model of the buffers starts out zero, like the one the player has; a job uploads the pages of the dumps that differ from it.
"""
import argparse
import json
import struct
import sys
from pathlib import Path

PAGE = 4096


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('directory', type=Path)
    ap.add_argument('width', type=int)
    ap.add_argument('height', type=int)
    ap.add_argument('out', type=Path)
    ap.add_argument('--bsbs-reg', type=int, default=175, help='the register that holds where the block starts writing the bit stream (adr_bsbs)')
    args = ap.parse_args()

    files = sorted(args.directory.glob('job*.json'))
    if not files:
        sys.exit('no jobs in %s' % args.directory)
    jobs = [json.loads(f.read_text()) for f in files]

    sizes = {}
    for job in jobs:
        for r in job['relocs']:
            sizes[r['buf']] = max(sizes.get(r['buf'], 0), r['size'])
    nbuf = max(sizes)
    model = {b: bytearray(sizes[b]) for b in sizes}

    out = bytearray(b'VET1')
    out += struct.pack('<4I', args.width, args.height, nbuf, len(jobs))
    for b in range(1, nbuf + 1):
        out += struct.pack('<I', sizes.get(b, 0))

    for index, (f, job) in enumerate(zip(files, jobs)):
        regs = sorted((int(i), int(v) & 0xffffffff) for i, v in job['regs'])
        out += struct.pack('<I', len(regs))
        for i, v in regs:
            out += struct.pack('<II', i, v)
        relocs = job['relocs']
        out += struct.pack('<I', len(relocs))
        for r in relocs:
            out += struct.pack('<III', r['reg'], r['buf'], r['off'])

        uploads = []
        for dump in sorted(args.directory.glob('buf*_job%03d.bin' % job['job'])):
            b = int(dump.name[3:dump.name.index('_')])
            data = dump.read_bytes()
            if b not in model or len(data) != len(model[b]):
                sys.exit('%s: buffer %d has %d bytes, the relocations say %d' % (dump.name, b, len(data), len(model.get(b, b''))))
            # runs of pages that differ from the model; the ends of a run are trimmed to the bytes that differ (a header in an otherwise zero page is 37 bytes,
            # not 4096), so that a run is as long as the CPU's write
            old_data = bytes(model[b])
            runs, start = [], None
            for page in range(0, len(data) + PAGE, PAGE):
                differs = page < len(data) and data[page:page + PAGE] != old_data[page:page + PAGE]
                if differs and start is None:
                    start = page
                elif not differs and start is not None:
                    runs.append((start, min(page, len(data))))
                    start = None
            for begin, end in runs:
                first = next(i for i in range(begin, end) if data[i] != old_data[i])
                last = max(i for i in range(begin, end) if data[i] != old_data[i]) + 1
                uploads.append((b, first, data[first:last]))
            model[b][:] = data
        out += struct.pack('<I', len(uploads))
        for b, offset, data in uploads:
            out += struct.pack('<III', b, offset, len(data)) + data + b'\0' * (-len(data) % 4)

        start = next((r for r in relocs if r['reg'] == args.bsbs_reg), None)
        if start is None:
            sys.exit('%s: no relocation for register %d' % (f.name, args.bsbs_reg))
        out += struct.pack('<III', start['buf'], start['off'], sizes[start['buf']] - start['off'])
        print('job %d: %d registers, %d relocations, %d upload runs (%d bytes), stream buffer %d from +%d' %
              (index, len(regs), len(relocs), len(uploads), sum(len(u[2]) for u in uploads), start['buf'], start['off']))

    args.out.write_bytes(out)
    print('%s: %d bytes' % (args.out, len(out)))


if __name__ == '__main__':
    main()
