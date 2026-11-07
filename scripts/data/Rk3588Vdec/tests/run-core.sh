#!/bin/sh
# Host test of the decoder core (MPP's H.264 parser + vdpu34x register generator) against the traces recorded from MPP on the board.
# MPP at the commit in core/mpp.lock is downloaded on first use (tree hash checked); MPP=/path/to/checkout uses a local one (also checked).
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TMPDIR:-/tmp}/vdec-core-host
mkdir -p "$out"
SAN=1 "$here/../core/build-host.sh" "$out"
c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I"$here/.." "$here/test-core.cpp" "$out/libvdeccore.a" -lpthread -o "$out/test-core"
args=
for s in "$here"/../streams/*.h264; do
  n=$(basename "$s" .h264)
  crc="$here/../streams/$n.crc"
  [ -f "$here/../streams/$n.coded.crc" ] && crc="$here/../streams/$n.coded.crc"   # a cropped stream: the job chain is checked against the whole coded pictures
  args="$args $s $here/../traces/mpp-nt/$n.vdt $crc"
done
# shellcheck disable=SC2086
"$out/test-core" "$@" $args
