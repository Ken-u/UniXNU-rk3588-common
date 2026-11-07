#!/bin/sh
# Host test of the AppleD5500 service's stream handling (D5500Stream.h): extradata records, framing of access units, one decoded picture per call.
# Uses the decoder core of core/ (MPP at the commit in core/mpp.lock: downloaded on first use, or MPP=/path/to/checkout) behind a block that accepts every job.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TMPDIR:-/tmp}/vdec-core-host
mkdir -p "$out"
SAN=1 "$here/../core/build-host.sh" "$out"
c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I"$here/.." "$here/test-d5500.cpp" "$out/libvdeccore.a" -lpthread -o "$out/test-d5500"
"$out/test-d5500" "$@" "$here"/../streams/*.h264 "$here"/../streams/*.h265
