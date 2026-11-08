#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Builds the encoder core for the host: MPP's sources (core/mpp-files.txt, from the checkout in $MPP or the pinned download) with the shim and the loop of this directory, into
# the static library $OUT/libvenccore.a. The host test links it (whole: MPP's modules set themselves up from constructors). Usage: [MPP=/path/to/mpp] [CC=cc] [SAN=1] [OURS_ONLY=1] build-host.sh OUT
set -e
out=$1
[ -n "$out" ] || { sed -n 3,6p "$0"; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
MPP=$(python3 "$here/fetch_mpp.py" "${MPP_CACHE:-}")   # $MPP if set, else a download (checked against mpp.lock either way)
CC=${CC:-cc}
mkdir -p "$out/obj"
inc="-I$here"
for d in $(grep -v '^#' "$here/mpp-includes.txt" | grep -v '^$'); do inc="$inc -I$MPP/$d"; done
defs="-DARCH_X64=1 -DARCH_X86=1 -DHAVE_H264E=1 -DHAVE_H265E=1 -DHW_VEPU580 -DMPP_ENC_IMPL_C=\"$MPP/mpp/codec/mpp_enc_impl.c\""
san=""
[ -n "$SAN" ] && san="-fsanitize=address,undefined -fno-sanitize-recover=undefined"
mppsan=""
[ -n "$SAN" ] && mppsan="$san -fno-sanitize=alignment,shift"
# MPP's own code: third-party, so no warnings (OURS_ONLY=1: only our two files, the objects of MPP's from an earlier build are kept)
[ -n "$OURS_ONLY" ] || grep -v '^#' "$here/mpp-files.txt" | sed 's/#.*//' | grep -v '^ *$' | while read -r f; do
  o="$out/obj/$(basename "$f" .c).o"
  $CC -std=gnu11 -O1 -g -fno-strict-aliasing -Wno-multichar -w $mppsan $defs $inc -include rk_type.h -c "$MPP/$f" -o "$o"
done
# ours: VencCore.c includes MPP's controller, so it is compiled as third party code too (the part that is ours is checked by the kernel build and by review)
$CC -std=gnu11 -O1 -g -fno-strict-aliasing -Wno-multichar -w -Werror=implicit-function-declaration -Werror=incompatible-pointer-types $mppsan $defs $inc -include rk_type.h -c "$here/VencCore.c" -o "$out/obj/VencCore.o"
$CC -std=gnu11 -O1 -g -fno-strict-aliasing -Wno-multichar -w -Werror=implicit-function-declaration -Werror=incompatible-pointer-types $mppsan $defs $inc -include rk_type.h -c "$here/EncShim.c" -o "$out/obj/EncShim.o"
rm -f "$out/libvenccore.a"
ar rcs "$out/libvenccore.a" "$out"/obj/*.o
echo "built $out/libvenccore.a"
