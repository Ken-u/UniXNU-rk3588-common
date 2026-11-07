#!/bin/bash
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Record an encode trace: Rockchip's unmodified MPP (rockchip-linux/mpp, built for the host with BUILD_TEST=ON: test/mpi_enc_test) encodes a synthetic video
# under tools/trace-shim-enc.c, which records every job the vepu580 register generator submits; tools/make-enc-trace.py packs the recording into
# traces/NAME.vet. The source video is Rk3588Vdec/tools/gen-yuv.py's (deterministic): check-enc-output.py compares the block's stream with it.
#
#   MPI_ENC_TEST=.../test/mpi_enc_test [MPP_LIB=.../mpp] [PYTHON_NUMPY=/usr/bin/python3] [CC=cc] \
#     make-enc-trace.sh NAME WIDTH HEIGHT FRAMES WORKDIR [-- mpi_enc_test arguments]
#
# Without arguments after `--` the encoder is asked for H.264 (-t 7), YUV 4:2:0 planar input (-f 4), a fixed QP of 28 (-rc 2 -qc 28:28:28:28:28). H.265 is
# `-- -t 16777220 -f 4 -rc 2 -qc 28:28:28:28:28`. The files left in WORKDIR: NAME.yuv (the source), NAME.fake.stream (what MPP made of the shim's fake
# answers: not a real stream), NAME.rec/ (the recording), and the trace is written to ../traces/NAME.vet.
set -e
if [ $# -lt 5 ]; then echo "usage: MPI_ENC_TEST=... $0 NAME WIDTH HEIGHT FRAMES WORKDIR [-- mpi_enc_test arguments] (see the head of this file)"; exit 2; fi
name=$1; width=$2; height=$3; frames=$4; work=$5; shift 5
[ "${1:-}" = "--" ] && shift
here=$(cd "$(dirname "$0")" && pwd)
: "${MPI_ENC_TEST:?set MPI_ENC_TEST to the host build of test/mpi_enc_test of MPP}"
mpp_lib=${MPP_LIB:-$(dirname "$MPI_ENC_TEST")/../mpp}
mkdir -p "$work"
work=$(cd "$work" && pwd)
args=("$@")
[ ${#args[@]} -eq 0 ] && args=(-t 7 -f 4 -rc 2 -qc 28:28:28:28:28)

${CC:-cc} -shared -fPIC -O2 -o "$work/trace-shim-enc.so" "$here/trace-shim-enc.c" -ldl -lpthread
"${PYTHON_NUMPY:-/usr/bin/python3}" "$here/../../Rk3588Vdec/tools/gen-yuv.py" "$width" "$height" "$((frames + 1))" "$work/$name.yuv"
rm -rf "$work/$name.rec"
(cd "$work" && LD_PRELOAD="$work/trace-shim-enc.so" VENC_TRACE_DIR="$name.rec" LD_LIBRARY_PATH="$mpp_lib" timeout 120 \
  "$MPI_ENC_TEST" -i "$name.yuv" -o "$name.fake.stream" -w "$width" -h "$height" -n "$frames" "${args[@]}" > "$name.log" 2>&1) || { tail -30 "$work/$name.log"; exit 1; }
jobs=$(ls "$work/$name.rec"/job*.json | wc -l)
if [ "$jobs" -ne "$frames" ]; then echo "$name: $jobs jobs for $frames frames (see $work/$name.log)"; exit 1; fi
python3 "$here/make-enc-trace.py" "$work/$name.rec" "$width" "$height" "$here/../traces/$name.vet"
