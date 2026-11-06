#!/bin/sh
# Host test of the trace parser and player (sanitizers on). Run from anywhere.
set -e
here=$(cd "$(dirname "$0")" && pwd)
c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I"$here/.." "$here/test-vdec.cpp" -o "${TMPDIR:-/tmp}/test-vdec"
"${TMPDIR:-/tmp}/test-vdec" "$here"/../traces/*.vdt
