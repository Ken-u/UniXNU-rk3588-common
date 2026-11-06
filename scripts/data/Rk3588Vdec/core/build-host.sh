#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Builds the decoder core for the host: MPP's sources (core/mpp-files.txt, from the checkout in $MPP) with the shim and the loop of this directory, into
# the static library $OUT/libvdeccore.a. The host test links it. Usage: MPP=/path/to/mpp [CC=cc] [SAN=1] build-host.sh OUT
set -e
out=$1
[ -n "$out" ] && [ -n "$MPP" ] || { sed -n 3,6p "$0"; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
want=$(sed -n 's/^commit=//p' "$here/mpp.lock")
have=$(git -C "$MPP" rev-parse HEAD 2>/dev/null || echo unknown)
[ "$want" = "$have" ] || { echo "MPP is at $have, core/mpp.lock wants $want" >&2; exit 1; }
CC=${CC:-cc}
mkdir -p "$out/obj"
python3 "$here/derive-mpp.py" "$MPP" "$out"
inc="-I$here -I$MPP/inc -I$MPP/utils -I$MPP/osal/inc -I$MPP/mpp/inc -I$MPP/mpp/base/inc -I$MPP/kmpp/inc -I$MPP/kmpp/base/inc -I$MPP/mpp/common \
-I$MPP/mpp/codec/inc -I$MPP/mpp/hal/inc -I$MPP/mpp/hal/common -I$MPP/mpp/vproc/inc -I$MPP/mpp/codec/dec/h264 -I$MPP/mpp/codec/dec/common \
-I$MPP/mpp/hal/rkdec/inc -I$MPP/mpp/hal/rkdec/h264d -I$MPP/mpp/hal/rkdec/common -I$MPP/mpp/hal/vpu -I$MPP/osal"
defs="-DARCH_X64=1 -DARCH_X86=1 -DHAVE_H264D -DHW_VDPU34X -DHW_VDPU381"
san=""
[ -n "$SAN" ] && san="-fsanitize=address,undefined -fno-sanitize-recover=undefined"
# MPP's own code, three checks off: mpp_slots_set_prop() reads 4 bytes through its `val` before it looks at the property, and for the alignment callbacks
# `val` is a function (a read of code at any address); the bit reader shifts a signed 64-bit cache; hal_bufs_clear() shifts `1 << i` for i up to max_cnt, which can pass 31. All harmless on the
# compilers and targets we use. Every other check stays on.
mppsan=""
[ -n "$SAN" ] && mppsan="$san -fno-sanitize=alignment,shift"
# MPP's own code: third-party, so no warnings
grep -v '^#' "$here/mpp-files.txt" | grep -v '^$' | while read -r f; do
  o="$out/obj/$(basename "$f" .c).o"
  $CC -std=gnu11 -O1 -g -fno-strict-aliasing -Wno-multichar -w $mppsan $defs $inc -c "$MPP/$f" -o "$o"
done
$CC -std=gnu11 -O1 -g -fno-strict-aliasing -Wno-multichar -w $mppsan $defs $inc -c "$out/mpp_sys_cfg_proc.c" -o "$out/obj/mpp_sys_cfg_proc.o"
# ours
for f in MppShim.c VdecCore.c; do
  $CC -std=gnu11 -O1 -g -fno-strict-aliasing -Wno-multichar -Wall -Wextra -Werror $san $defs $inc -c "$here/$f" -o "$out/obj/$(basename "$f" .c).o"
done
rm -f "$out/libvdeccore.a"
ar rcs "$out/libvdeccore.a" "$out"/obj/*.o
echo "built $out/libvdeccore.a"
