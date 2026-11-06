#!/bin/sh
# Host test of the decoder core (MPP's H.264 parser + vdpu34x register generator) against the traces recorded from MPP on the board.
# Needs MPP checked out at the commit in core/mpp.lock: MPP=/path/to/mpp tests/run-core.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TMPDIR:-/tmp}/vdec-core-host
mkdir -p "$out"
SAN=1 "$here/../core/build-host.sh" "$out"
c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I"$here/.." "$here/test-core.cpp" "$out/libvdeccore.a" -lpthread -o "$out/test-core"
args=
for s in "$here"/../streams/*.h264; do
  n=$(basename "$s" .h264)
  args="$args $s $here/../traces/mpp-nt/$n.vdt $here/../streams/$n.crc"
done
# shellcheck disable=SC2086
"$out/test-core" "$@" $args
