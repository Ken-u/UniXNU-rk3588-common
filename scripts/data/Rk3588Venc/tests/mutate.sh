#!/bin/bash
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Breaks VencTrace.h nine ways, one at a time, and requires test-venc to fail each time (prints "MUTATION SURVIVED" and exits 1 otherwise). The player is the
# part of the encoder test that decides whether the hardware gets the sequence the vendor driver gives it, so the test has to notice each departure.
here=$(cd "$(dirname "$0")/.." && pwd)
survived=0
t=$(mktemp -d)
mut() {
  name=$1; shift
  mkdir -p $t/m; cp $here/VencTrace.h $t/m/VencTrace.h
  python3 - "$t/m/VencTrace.h" "$@" <<'PY'
import sys
p=sys.argv[1]; old=sys.argv[2]; new=sys.argv[3]
s=open(p).read()
assert old in s, old
open(p,'w').write(s.replace(old,new,1))
PY
  c++ -std=c++17 -O1 -fsanitize=address,undefined -I$t/m $here/tests/test-venc.cpp -o $t/tv 2>$t/err || { echo "$name: does not compile"; cat $t/err | head -5; return; }
  if $t/tv $here/traces/*.vet > $t/out 2>&1; then echo "MUTATION SURVIVED: $name"; survived=1; else echo "caught: $name ($(tail -1 $t/out))"; fi
}
mut "no counter clear" '    hw_.write32(hw_.ctx, kRegHwCounter, 0x2);' ''
mut "start inside the loop" 'if (index != kRegStart) hw_.write32' 'if (true) hw_.write32'
mut "no uploads" 'memcpy(hw_.dmaVirt + trace_->bufOffset(u.buf) + u.offset, u.data, u.len);' ''
mut "relocation +4" 'trace_->bufOffset(r.buf) + r.offset);' 'trace_->bufOffset(r.buf) + r.offset + 4);'
mut "status never cleared" 'if (st) hw_.write32(hw_.ctx, kRegIntClr, st);' ''
mut "start value 0" 'set[kRegStart] ? regs[kRegStart] : 0' '0'
mut "errors ignored" 'if (st & (kStaEncDone | kStaErrorMask)) {' 'if (st & kStaEncDone) {'
mut "no forced clear" '    hw_.write32(hw_.ctx, kRegClear, 0x2);
    hw_.delayUs(hw_.ctx, 5);
    hw_.write32(hw_.ctx, kRegClear, 0);
    hw_.write32(hw_.ctx, kRegHwCounter' '    hw_.write32(hw_.ctx, kRegHwCounter'
mut "stream address wrong" 'hw_.dmaVirt + trace_->bufOffset(job.outBuf) + job.outOffset' 'hw_.dmaVirt + trace_->bufOffset(job.outBuf) + job.outOffset + 1'
rm -rf $t
exit $survived
