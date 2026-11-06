#!/bin/sh
# Build and run the host test of the xHCI driver against the controller model (needs a C++17 compiler; the sanitizers are what make it worth running).
set -e
cd "$(dirname "$0")"
${CXX:-c++} -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I.. test-xhci.cpp -o "${TMPDIR:-/tmp}/test-xhci"
"${TMPDIR:-/tmp}/test-xhci"
