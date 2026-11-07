#!/usr/bin/env python3
"""Medians of the per-frame profile the AppleD5500 service prints and of the timing lines of usertest/vt_test.c.

usage: profile-summary.py LOG...      (serial captures of a rk3588-vdec=7 run, with or without the "[ time]" prefix)

Per log: for each picture size, the median microseconds a frame spends in the service (queue wait, prepare, core, finish; inside the core the block's
start-to-interrupt time, the job runner and the picture callback split into flush, copy, flush and probe), then per stream the test tool's timing
(first and last picture, copy out of the surface, CRC) and the time WaitForAsynchronousFrames took, and the PASS / FAIL / mpp_buf_slot line counts.
Lines of older tool builds (one "CRC" figure, no separate copy) are read too; their copy column is 0.
"""
import re, sys, statistics as st

def med(v):
    return st.median(v) if v else float('nan')

for path in sys.argv[1:]:
    size = None
    groups = {}
    pending = None
    for line in open(path, errors='replace'):
        m = re.search(r'CreateDecoder: stream from the SPS: .*?(\d+)x(\d+) macroblocks', line)
        if m:
            size = '%dx%d' % (int(m.group(1)) * 16, int(m.group(2)) * 16)
        m = re.search(r'profile of (\d+) frames, microseconds per frame: queue (\d+), prepare (\d+), core (\d+), finish (\d+)', line)
        if m:
            pending = [int(x) for x in m.groups()]
            continue
        m = re.search(r'profile, inside core: block (\d+) start to interrupt, (\d+) in the job runner \((\d+) jobs\); picture callback (\d+) = flush (\d+) \+ copy (\d+) \+ flush (\d+) \+ probe (\d+); rest (\d+)', line)
        if m and pending is not None:
            g = groups.setdefault(size, [])
            g.append(pending + [int(x) for x in m.groups()])
            pending = None
    print(path)
    names = ['frames', 'queue', 'prepare', 'core', 'finish', 'block', 'runner', 'jobs', 'callback', 'flushpre', 'copy', 'flushpost', 'probe', 'rest']
    for size, rows in sorted(groups.items()):
        print('  %s: %d decoders' % (size, len(rows)))
        print('    median us/frame: ' + ', '.join('%s %d' % (n, med([r[i] for r in rows])) for i, n in enumerate(names) if n not in ('frames', 'jobs')))
        print('    max core %d, min core %d' % (max(r[3] for r in rows), min(r[3] for r in rows)))
    # tool side
    t = {}
    for line in open(path, errors='replace'):
        m = re.search(r'vt: (\w+): timing: first picture (\d+) us after the first frame went in, last (\d+) us; in the callbacks: (?:copy out of the surface (\d+) us, CRC of the copy|CRC) (\d+) us, log line (\d+) us', line)
        if m:
            name = m.group(1)
            if m.group(4) is None:
                t.setdefault(name, []).append((int(m.group(2)), int(m.group(3)), 0, int(m.group(5)), int(m.group(6))))
            else:
                t.setdefault(name, []).append(tuple(int(m.group(i)) for i in (2, 3, 4, 5, 6)))
    for name, rows in sorted(t.items()):
        print('  tool %-14s n=%3d  first %6d  last %7d  copy %6d  crc %7d  say %5d  (medians, us)' % (
            name, len(rows), med([r[0] for r in rows]), med([r[1] for r in rows]), med([r[2] for r in rows]), med([r[3] for r in rows]), med([r[4] for r in rows])))
    w = {}
    for line in open(path, errors='replace'):
        m = re.search(r'vt: (\w+): WaitForAsynchronousFrames: (-?\d+), (\d+) of (\d+) pictures out in (\d+) ms', line)
        if m:
            w.setdefault(m.group(1), []).append(int(m.group(5)))
    for name, rows in sorted(w.items()):
        print('  wait %-14s n=%3d  median %5d ms  max %5d ms' % (name, len(rows), med(rows), max(rows)))
    print('  PASS %d FAIL %d buf_slot lines %d' % (
        sum(1 for l in open(path, errors='replace') if re.search(r'vt: .*PASS', l)),
        sum(1 for l in open(path, errors='replace') if re.search(r'vt: .*FAIL', l)),
        sum(1 for l in open(path, errors='replace') if 'on_used' in l or 'buf_slot' in l)))
