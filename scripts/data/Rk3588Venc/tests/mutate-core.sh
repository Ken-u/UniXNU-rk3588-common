#!/bin/sh
# Negative control of the encoder core test: with one setting changed from the recording's (VENC_PERTURB: the quantiser, the bit rate, the key frame interval) the comparison
# against the recordings must fail. Needs run-core.sh to have built the test.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TMPDIR:-/tmp}/venc-core-host
if VENC_PERTURB=1 VENC_TRACES="$here/../traces" "$out/test-venc-core" > "$out/perturbed.log" 2>&1; then
  echo "mutate-core: the test PASSED with a perturbed setting: it does not look at what it should"
  exit 1
fi
echo "mutate-core: the perturbed run fails as it should ($(grep -c MISMATCH "$out/perturbed.log") of 6 recordings differ)"
