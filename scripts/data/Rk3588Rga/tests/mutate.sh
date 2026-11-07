#!/bin/sh
# Breaks the driver headers in a number of ways, one at a time, and requires tests/test-rga.cpp to fail each time (a test that passes a broken driver
# proves nothing). Works on copies in a temporary directory. Run from anywhere; prints one line per mutant and exits non-zero if one survives.
set -e
here=$(cd "$(dirname "$0")" && pwd)
work=$(mktemp -d "${TMPDIR:-/tmp}/rga-mutants.XXXXXX")
trap 'rm -rf "$work"' EXIT
exec /usr/bin/env python3 - "$here" "$work" <<'PY'
import os, shutil, subprocess, sys

here, work = sys.argv[1], sys.argv[2]
root = os.path.dirname(here)

# (file, text to find, replacement, what it breaks)
MUTANTS = [
    ('RkIommu.h', '((pa & 0xf00000000ull) >> 24)', '((pa & 0xf00000000ull) >> 25)', 'address bits 35:32 land in the wrong bits of a table word'),
    ('RkIommu.h', '((pa & 0xf000000000ull) >> 32)', '((pa & 0xf000000000ull) >> 33)', 'address bits 39:36 land in the wrong bits of a table word'),
    ('RkIommu.h', '(writable ? 4u : 0u)', '(writable ? 0u : 0u)', 'no page is writable'),
    ('RkIommu.h', '(writable ? 4u : 0u)', '4u', 'every page is writable'),
    ('RkIommu.h', 'if (isValid(word)) return false;\n      word = makePte', 'word = makePte', 'a mapped page can be mapped over'),
    ('RkIommu.h', 'if (pt) pt[((iova >> 12) + i) & (kEntries - 1)] = 0;', 'if (pt) (void)pt;', 'unmap leaves the words'),
    ('RkIommu.h', 'dte = makeDte(bus_ + used_ * kPageSize);', 'dte = makeDte(bus_ + (used_ + 1) * kPageSize);', 'a directory word points one page past its table'),
    ('RkIommu.h', '  command(r, kCmdZapCache);\n  command(r, kCmdDisableStall);', '  command(r, kCmdDisableStall);', 'invalidate does not zap while translating'),
    ('RkIommu.h', '  if (!(rd(kRegStatus) & kStatusPaging)) {\n    command(r, kCmdZapCache);\n    return true;\n  }\n  command(r, kCmdEnableStall);',
     '  if (!(rd(kRegStatus) & kStatusPaging)) {\n    command(r, kCmdZapCache);\n    return true;\n  }\n  command(r, kCmdZapCache);\n  command(r, kCmdEnableStall);', 'invalidate zaps before it has stalled'),
    ('RkIommu.h', '  if (rd(kRegStatus) & kStatusPaging) {\n    command(r, kCmdEnableStall);\n    if (!detail::waitFor(r, 1000, [&] { return (rd(kRegStatus) & kStatusStallActive) != 0; })) return false;\n  }\n  r.write(r.ctx, kRegDteAddr, 0xcafebabeu);',
     '  r.write(r.ctx, kRegDteAddr, 0xcafebabeu);', 'enable does not stall a translating unit'),
    ('RkIommu.h', 'command(r, kCmdForceReset);\n  if (!detail::waitFor(r, 1000, [&] { return rd(kRegDteAddr) == 0; })) return false;', 'command(r, kCmdZapCache);', 'enable does not reset the unit'),
    ('RkIommu.h', 'r.write(r.ctx, kRegIntMask, kIntPageFault | kIntBusError);', 'r.write(r.ctx, kRegIntMask, 0);', 'the fault interrupt bits are left masked'),
    ('RkIommu.h', 'r.write(r.ctx, kRegAutoGating, rd(kRegAutoGating) | kAutoGatingNoFetchLimit);', '', 'the auto-gating workaround is skipped'),
    ('Rga3.h', 'constexpr uint32_t strideWords(uint32_t bytes) { return align16(bytes) >> 2; }', 'constexpr uint32_t strideWords(uint32_t bytes) { return align16(bytes) >> 1; }', 'row pitch in the wrong unit'),
    ('Rga3.h', 'reg(kWin0ActOff) = c.srcX | (c.srcRow << 16);', 'reg(kWin0ActOff) = c.srcRow | (c.srcX << 16);', 'crop offsets swapped'),
    ('Rga3.h', 'reg(kWin0UBase) = c.srcUv;', 'reg(kWin0UBase) = c.srcY;', 'source chroma plane address'),
    ('Rga3.h', 'reg(kWrUBase) = c.dstUv;', 'reg(kWrUBase) = c.dstY;', 'destination chroma plane address'),
    ('Rga3.h', 'reg(kWin0DstSize) = c.width | (c.height << 16);', 'reg(kWin0DstSize) = (c.width + 2) | (c.height << 16);', 'window 0 scales by a little'),
    ('Rga3.h', 'reg(kWin0SrcSize) = align16(c.width + c.srcX) | (align16(c.height + c.srcRow) << 16);', 'reg(kWin0SrcSize) = (c.width + c.srcX) | ((c.height + c.srcRow) << 16);', 'source extent not aligned'),
    ('Rga3.h', '(1u << 20) | (1u << 22);\n  reg(kWin0YBase)', '(1u << 21) | (1u << 22);\n  reg(kWin0YBase)', 'horizontal bypass replaced by upscale'),
    ('Rga3.h', 'reg(kWrCtrl) = (0u << 4) | (1u << 8) | (1u << 2) | (1u << 10) | (0xfu << 13);', 'reg(kWrCtrl) = (0u << 4) | (1u << 8) | (1u << 2) | (1u << 10);', 'the outstanding-request limit is missing'),
    ('Rga3.h', 'reg(kWrPlVirStride) = strideWords(c.dstStride);', 'reg(kWrPlVirStride) = strideWords(c.srcStride);', 'destination chroma pitch taken from the source'),
    ('Rga3.h', 'reg(kOvlpBotCtrl) = 0xffu << 16;', '', 'bottom layer alpha missing'),
    ('Rga3.h', 'r.write(r.ctx, kIntEn, kIntFrameDone | kIntCmdLineFinish | kIntErrors);', 'r.write(r.ctx, kIntEn, kIntCmdLineFinish);', 'frame-done not enabled'),
    ('Rga3.h', 'r.write(r.ctx, kSysCtrl, sys | kSysStart);', 'r.write(r.ctx, kSysCtrl, sys | kSysStart | kSysCmdMode);', 'master mode'),
    ('Rga3.h', 'return res.done && !(res.intRaw & kIntErrors);', 'return res.done;', 'a job with error bits counts as done'),
    ('Rga3.h', 'if ((c.srcX | c.srcRow | c.width | c.height) & 1) return "an odd offset or size";', '', 'odd sizes are accepted'),
    ('Rga3.h', 'if (c.width < kMinWidth || c.height < kMinHeight) return "smaller than the block\'s minimum";', '', 'sizes under the minimum are accepted'),
    ('Rga3.h', 'if ((c.srcStride | c.dstStride) & (kStrideAlign - 1)) return "a row pitch is not a multiple of 16";', '', 'unaligned pitches are accepted'),
    ('Rga3.h', 'if (c.srcX + c.width > c.srcStride || c.width > c.dstStride) return "a row does not fit in its pitch";', '', 'rows wider than their pitch are accepted'),
    ('Rga3.h', '  r.write(r.ctx, kSysCtrl, kSysCclkReset | kSysAclkReset);', '  r.write(r.ctx, kSysCtrl, 0);', 'the soft reset does not reset'),
    ('Rga3.h', 'if (r.nowUs(r.ctx) - t0 > timeoutUs) break;', '', 'the job runner never gives up'),
]

import concurrent.futures


def one(item):
    number, (name, old, new, what) = item
    dst = os.path.join(work, 'm%d' % number)
    os.makedirs(os.path.join(dst, 'tests'))
    for header in ('Rga3.h', 'RkIommu.h'):
        shutil.copy(os.path.join(root, header), dst)
    shutil.copy(os.path.join(here, 'test-rga.cpp'), os.path.join(dst, 'tests'))
    path = os.path.join(dst, name)
    text = open(path).read()
    if old not in text:
        return False, 'mutant %2d: the text to break is not in %s: %s' % (number, name, what)
    open(path, 'w').write(text.replace(old, new, 1))
    exe = os.path.join(dst, 'test')
    build = subprocess.run(['c++', '-std=c++17', '-O1', '-g', '-fsanitize=address,undefined', '-w', '-I' + dst, os.path.join(dst, 'tests', 'test-rga.cpp'), '-o', exe],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if build.returncode:
        return False, 'mutant %2d: does not build (%s): %s' % (number, what, build.stdout.strip().splitlines()[-1] if build.stdout.strip() else '')
    try:
        run = subprocess.run([exe], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return True, 'mutant %2d killed (it hangs): %s' % (number, what)
    if run.returncode == 0:
        return False, 'mutant %2d SURVIVED: %s' % (number, what)
    return True, 'mutant %2d killed: %s' % (number, what)


failures = 0
with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, os.cpu_count() or 1)) as pool:
    for ok, line in pool.map(one, enumerate(MUTANTS, 1)):
        print(line, flush=True)
        failures += 0 if ok else 1
print('%d mutants, %d not killed' % (len(MUTANTS), failures))
sys.exit(1 if failures else 0)
PY
