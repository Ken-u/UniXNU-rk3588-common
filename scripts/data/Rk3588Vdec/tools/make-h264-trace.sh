#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Records a decode-job trace (../traces/NAME.vdt) of Rockchip's MPP for a synthetic H.264 stream, on an x86 host:
#
#   1. gen-yuv.py            a deterministic test video (I420)
#   2. libavc avcenc         encodes it (CABAC, Main profile, I and P frames) and writes its own reconstruction; or x264 when X264 is set (X264=/path/x264 [X264_ARGS='--profile high
#                            --bframes 2 ...']: its --dump-yuv reconstruction is the encoder's own, independent of both decoders)
#   3. libavc avcdec         decodes the stream again: the reference pictures (bit-exact, and equal to the encoder's reconstruction: checked here)
#   4. MPP mpi_dec_test      run with trace-shim.so: MPP believes it runs on an RK3588; its parser and its vdpu34x register generator produce the job
#                            register images, and the shim records them (nothing is decoded)
#   5. make-trace.py         packs the jobs and the reference pictures' CRC-32 into the trace the board replays
#
# Needed: a C compiler, x264 (only for the High profile streams; a GPL tool used on the host, none of its code is in this repository; the streams in ../streams
# were made with x264 0480cb0 from code.videolan.org/videolan/x264, built with --disable-asm), python3 with numpy (PYTHON_NUMPY, default /usr/bin/python3), libavc (AOSP external/libavc at 4275faf, built with clang: avcenc, avcdec) and MPP (rockchip-linux/mpp at
# 9b56348, built unmodified for the host with BUILD_TEST=ON: test/mpi_dec_test; the traces in ../traces were recorded with these two). Environment: AVCENC, AVCDEC, MPI_DEC_TEST (paths). Arguments:
#   [ENC_EXTRA='--entropy 0 --bframes 0'] make-h264-trace.sh NAME WIDTH HEIGHT FRAMES WORKDIR
# CROP=left,top,right,bottom (x264 only): the stream codes the whole picture and signals that only the inside is shown (the reference is cut down to it; the trace is of the whole
# coded picture; STREAMS_OUT then also gets NAME.coded.crc).
# ENC_EXTRA replaces the encoder options that select the tool set (default: CABAC, no B pictures).
# TRACE_OUT (default ../traces) is where the .vdt goes. traces/mpp-nt/ holds the same streams recorded with MPP's no-thread test (MPI_DEC_TEST=.../mpi_dec_nt_test:
# one job after the other, one register set, buffers reused the way core/VdecCore.c does it); ../traces has them from the threaded test (two register sets in turn).
set -e
name=$1 width=$2 height=$3 frames=$4 work=$5
[ -n "$work" ] || { sed -n 3,16p "$0"; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
out="${TRACE_OUT:-$here/../traces}"
mkdir -p "$work" "$out"
cd "$work"
cc -O1 -g -Wall -shared -fPIC -o trace-shim.so "$here/trace-shim.c" -ldl -lpthread
"${PYTHON_NUMPY:-/usr/bin/python3}" "$here/gen-yuv.py" "$width" "$height" "$frames" "$name.src.yuv"
if [ -n "$X264" ]; then
  # Interlaced coding (MBAFF, frame_mbs_only_flag = 0) codes pairs of macroblock rows: x264 pads a height that is not a multiple of 32 and signals a crop, and its
  # reconstruction (and libavc's output) is the cropped picture, while the block writes the padded one: the CRCs of the trace would not be the block's.
  case "${X264_ARGS:-}" in
    *interlaced*) [ $((height % 32)) -eq 0 ] || { echo "interlaced streams need a height that is a multiple of 32 (got $height)"; exit 1; } ;;
  esac
  # x264 (High profile tools, several slices, MBAFF, ...): X264_ARGS are its options; one IDR at the start, no scene cuts, one thread (deterministic)
  # shellcheck disable=SC2086
  "$X264" --input-res "${width}x${height}" --fps 30 --qp 28 --keyint "$frames" --min-keyint "$frames" --no-scenecut --threads 1 --frames "$frames" \
    ${X264_ARGS:---profile high} ${CROP:+--crop-rect $CROP} --dump-yuv "$name.recon.yuv" -o "$name.h264" "$name.src.yuv" > "$name.x264.log" 2>&1
else
  "$AVCENC" --input "$name.src.yuv" --output "$name.h264" --width "$width" --height "$height" --num_frames "$frames" --rc 0 --qp_i 28 --qp_p 30 \
    --i_interval "$frames" --idr_interval "$frames" --profile MAIN ${ENC_EXTRA:---entropy 1 --bframes 0} --recon_enable 1 --recon "$name.recon.yuv" \
    --input_chroma_format YUV_420P --recon_chroma_format YUV_420P --num_cores 1 --arch X86_GENERIC > /dev/null
fi
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
if [ -n "$CROP" ]; then
  # CROP=left,top,right,bottom (x264 --crop-rect): the stream codes the whole WIDTHxHEIGHT picture and says that only the inside is to be shown. libavc writes the
  # visible part; x264's reconstruction is the whole coded picture (what the block writes, and what the traces' CRCs are of), so cut it down to compare.
  "${PYTHON_NUMPY:-/usr/bin/python3}" - "$name.recon.yuv" "$name.recon.vis.yuv" "$width" "$height" "$CROP" <<'PY'
import sys
src, dst, w, h, crop = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), [int(v) for v in sys.argv[5].split(',')]
l, t, r, b = crop; vw, vh = w - l - r, h - t - b
d = open(src, 'rb').read(); n = w * h * 3 // 2; out = bytearray()
for k in range(len(d) // n):
    f = d[k * n:(k + 1) * n]
    for y in range(t, t + vh): out += f[y * w + l:y * w + l + vw]
    cw = w // 2
    for base in (w * h, w * h + w * h // 4):
        for y in range(t // 2, (t + vh) // 2): out += f[base + y * cw + l // 2:base + y * cw + (l + vw) // 2]
open(dst, 'wb').write(out)
PY
  cmp "$name.ref.yuv" "$name.recon.vis.yuv" || { echo "the decoder and the encoder disagree: not a usable reference"; exit 1; }
  trace_ref="$name.recon.yuv"
else
  cmp "$name.ref.yuv" "$name.recon.yuv" || { echo "the decoder and the encoder disagree: not a usable reference"; exit 1; }
  trace_ref="$name.ref.yuv"
fi
rm -rf "$name.jobs"
# MPP deinterlaces interlaced streams through the IEP block when it can (the shim answers for it, and the picture would carry no job marker); the kernel core decodes without it
enable_deinterlace=0 VDEC_TRACE_DIR="$work/$name.jobs" LD_PRELOAD="$work/trace-shim.so" timeout 120 "$MPI_DEC_TEST" -t 7 -i "$name.h264" -n "$frames" -o "$work/$name.mpp.yuv" > "$name.mpp.log" 2>&1 || true
python3 "$here/make-trace.py" "$name.jobs" --ref "$trace_ref" --width "$width" --height "$height" --jobs "$frames" --mpp-out "$name.mpp.yuv" ${CROP:+--mpp-size $(echo "$CROP $width $height" | awk -F"[, ]" '{print ($5-$1-$3) "x" ($6-$2-$4)}')} --out "$out/$name"
rm -f "$out/$name.inc"   # build-rk3588-vdec.py generates the C array at build time
if [ -n "$STREAMS_OUT" ]; then   # STREAMS_OUT=../streams: also keep the stream and the CRC-32 of each reference picture (NV12, display order) for the core test and the VideoToolbox test
  mkdir -p "$STREAMS_OUT"
  cp "$name.h264" "$STREAMS_OUT/$name.h264"
  # NAME.crc: the visible pictures (what VideoToolbox and the kernel self-test compare); NAME.coded.crc (cropped streams only): the whole coded pictures
  # (what a job of the trace writes: tests/test-core.cpp prefers it when it exists)
  "${PYTHON_NUMPY:-/usr/bin/python3}" - "$name" "$width" "$height" "$CROP" "$STREAMS_OUT" <<'PY'
import sys, zlib
name, w, h, crop, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], sys.argv[5]
l = t = r = b = 0
if crop: l, t, r, b = [int(v) for v in crop.split(',')]
def write(src, vw, vh, path, what):
    data = open(src, 'rb').read(); n = vw * vh * 3 // 2
    with open(path, 'w') as f:
        f.write('# CRC-32 of each %s picture (NV12, %dx%d), display order: libavc 4275faf (equal to the encoder\'s own reconstruction), the reference of traces/%s.vdt\n' % (what, vw, vh, name))
        for k in range(len(data) // n):
            p = data[k * n:(k + 1) * n]; y = p[:vw * vh]; u = p[vw * vh:vw * vh * 5 // 4]; v = p[vw * vh * 5 // 4:]
            nv = bytearray(vw * vh // 2); nv[0::2] = u; nv[1::2] = v
            f.write('%08x\n' % (zlib.crc32(y + bytes(nv)) & 0xffffffff))
write(name + '.ref.yuv', w - l - r, h - t - b, '%s/%s.crc' % (out, name), 'decoded' if not crop else 'visible')
if crop: write(name + '.recon.yuv', w, h, '%s/%s.coded.crc' % (out, name), 'coded')
PY
fi
