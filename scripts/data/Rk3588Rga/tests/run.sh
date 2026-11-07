#!/bin/sh
# Host test of the page tables, the IOMMU register sequences and the RGA3 copy driver against a strict hardware model (sanitizers on). Run from anywhere.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TMPDIR:-/tmp}/test-rga
c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I"$here/.." "$here/test-rga.cpp" -o "$out"
"$out"
