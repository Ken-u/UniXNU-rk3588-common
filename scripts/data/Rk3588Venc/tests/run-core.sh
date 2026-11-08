#!/bin/sh
# Host test of the encoder core (MPP's H.264 syntax writers, rate control and vepu580 register generator, unmodified, behind core/EncShim.c) against the
# recordings of MPP itself (traces/e*_h264*.vet). MPP at the commit in core/mpp.lock is downloaded on first use (tree hash checked); MPP=/path/to/checkout uses a local one.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TMPDIR:-/tmp}/venc-core-host
mkdir -p "$out"
[ -n "$NOBUILD" ] || SAN=1 "$here/../core/build-host.sh" "$out"
# whole archive: MPP's modules set themselves up from constructors, which only run from objects that are linked in
c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I"$here/.." "$here/test-venc-core.cpp" -Wl,--whole-archive "$out/libvenccore.a" -Wl,--no-whole-archive -lpthread -lm -o "$out/test-venc-core"
VENC_TRACES="$here/../traces" "$out/test-venc-core" "$@"
