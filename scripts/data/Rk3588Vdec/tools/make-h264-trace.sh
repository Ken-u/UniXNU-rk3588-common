#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Records a decode-job trace (../traces/NAME.vdt) of Rockchip's MPP for a synthetic H.264 stream, on an x86 host:
#
#   1. gen-yuv.py            a deterministic test video (I420)
#   2. libavc avcenc         encodes it (CABAC, Main profile, I and P frames) and writes its own reconstruction
#   3. libavc avcdec         decodes the stream again: the reference pictures (bit-exact, and equal to the encoder's reconstruction: checked here)
#   4. MPP mpi_dec_test      run with trace-shim.so: MPP believes it runs on an RK3588; its parser and its vdpu34x register generator produce the job
#                            register images, and the shim records them (nothing is decoded)
#   5. make-trace.py         packs the jobs and the reference pictures' CRC-32 into the trace the board replays
#
# Needed: a C compiler, python3 with numpy (PYTHON_NUMPY, default /usr/bin/python3), libavc (AOSP external/libavc at 4275faf, built with clang: avcenc, avcdec) and MPP (rockchip-linux/mpp at
# 9b56348, built unmodified for the host with BUILD_TEST=ON: test/mpi_dec_test; the traces in ../traces were recorded with these two). Environment: AVCENC, AVCDEC, MPI_DEC_TEST (paths). Arguments:
#   [ENC_EXTRA='--entropy 0 --bframes 0'] make-h264-trace.sh NAME WIDTH HEIGHT FRAMES WORKDIR
# ENC_EXTRA replaces the encoder options that select the tool set (default: CABAC, no B pictures).
set -e
name=$1 width=$2 height=$3 frames=$4 work=$5
[ -n "$work" ] || { sed -n 3,16p "$0"; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
out="$here/../traces"
mkdir -p "$work" "$out"
cd "$work"
cc -O1 -g -Wall -shared -fPIC -o trace-shim.so "$here/trace-shim.c" -ldl
"${PYTHON_NUMPY:-/usr/bin/python3}" "$here/gen-yuv.py" "$width" "$height" "$frames" "$name.src.yuv"
"$AVCENC" --input "$name.src.yuv" --output "$name.h264" --width "$width" --height "$height" --num_frames "$frames" --rc 0 --qp_i 28 --qp_p 30 \
  --i_interval "$frames" --idr_interval "$frames" --profile MAIN ${ENC_EXTRA:---entropy 1 --bframes 0} --recon_enable 1 --recon "$name.recon.yuv" \
  --input_chroma_format YUV_420P --recon_chroma_format YUV_420P --num_cores 1 --arch X86_GENERIC > /dev/null
cat > "$name.dec.cfg" <<C
--input $name.h264
--save_output 1
--num_frames -1
--output $name.ref.yuv
--chroma_format YUV_420P
--share_display_buf 0
--num_cores 1
--loopback 0
--display 0
--fps 30
--arch X86_GENERIC
--soc GENERIC
C
"$AVCDEC" --config "$name.dec.cfg" > /dev/null
cmp "$name.ref.yuv" "$name.recon.yuv" || { echo "the decoder and the encoder disagree: not a usable reference"; exit 1; }
rm -rf "$name.jobs"
VDEC_TRACE_DIR="$work/$name.jobs" LD_PRELOAD="$work/trace-shim.so" timeout 120 "$MPI_DEC_TEST" -t 7 -i "$name.h264" -n "$frames" -o "$work/$name.mpp.yuv" > "$name.mpp.log" 2>&1 || true
python3 "$here/make-trace.py" "$name.jobs" --ref "$name.ref.yuv" --width "$width" --height "$height" --jobs "$frames" --mpp-out "$name.mpp.yuv" --out "$out/$name"
rm -f "$out/$name.inc"   # build-rk3588-vdec.py generates the C array at build time
