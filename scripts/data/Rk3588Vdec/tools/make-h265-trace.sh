#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Records a decode-job trace (../traces/NAME.vdt) of Rockchip's MPP for a synthetic H.265 stream, on an x86 host. Like make-h264-trace.sh:
#
#   1. gen-yuv.py            a deterministic test video (I420)
#   2. libhevc hevcenc       encodes it (Main profile, constant QP; its picture hash SEI carries a checksum of each of its reconstructed pictures)
#   3. libhevc hevcdec       decodes the stream again: the reference pictures
#   4. check-hevc-sei.py     the reference pictures must have exactly the checksums the encoder wrote into the stream (libhevc's decoder does not check them itself)
#   5. MPP mpi_dec_test      run with trace-shim.so: MPP believes it runs on an RK3588; its H.265 parser and its vdpu34x register generator produce the job
#                            register images, and the shim records them (nothing is decoded)
#   6. make-trace.py         packs the jobs and the reference pictures' CRC-32 into the trace the board replays
#
# Needed: as make-h264-trace.sh, with libhevc (AOSP external/libhevc at 48cb531, built with clang: hevcenc, hevcdec) instead of libavc; the encoder only starts when every
# config key is given, so the sample test/encoder/vid_enc_cfg.txt of that checkout is the base. Environment: HEVCENC, HEVCDEC, MPI_DEC_TEST (paths), HEVC_CFG (the sample
# vid_enc_cfg.txt), PYTHON_NUMPY. Arguments:
#   [ENC_SET='--max_temporal_layers 2 --frame_qp 24'] make-h265-trace.sh NAME WIDTH HEIGHT FRAMES WORKDIR
# ENC_SET overrides keys of the sample config (key and value pairs, as in the file). The default is constant QP 28 (--rate_control_mode 3 --frame_qp 28), one IDR
# (no CRA, no later I pictures), no B pyramid (--max_temporal_layers 0).
set -e
name=$1 width=$2 height=$3 frames=$4 work=$5
[ -n "$work" ] || { sed -n 3,21p "$0"; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
out="$here/../traces"
mkdir -p "$work" "$out"
cd "$work"
cc -O1 -g -Wall -shared -fPIC -o trace-shim.so "$here/trace-shim.c" -ldl -lpthread
py="${PYTHON_NUMPY:-/usr/bin/python3}"
"$py" "$here/gen-yuv.py" "$width" "$height" "$frames" "$name.src.yuv"
"$py" - "$HEVC_CFG" "$name" "$width" "$height" "$frames" "$name.enc.cfg" ${ENC_SET} <<'P'
import re, sys
cfg, name, w, h, frames, dst = sys.argv[1:7]
key = {'--input': name + '.src.yuv', '--output': name + '.265', '--num_frames_to_encode': frames, '--src_width': w, '--src_height': h, '--codec_level': '90',
       '--rate_control_mode': '3', '--frame_qp': '28', '--max_closed_gop_period': frames, '--min_closed_gop_period': frames, '--max_cra_open_gop_period': '0',
       '--max_i_open_gop_period': '0', '--max_temporal_layers': '0', '--num_cores': '1', '--archType': '4', '--sei_enable_flags': '1', '--sei_hash_flags': '3',
       '--log_dump_level': '1'}
extra = sys.argv[7:]
for k, v in zip(extra[0::2], extra[1::2]):
    key[k] = v
seen = set()
lines = []
for line in open(cfg).read().split('\n'):
    m = re.match(r'^(--\w+)(\s+)(\S+)(.*)$', line)
    if m and m.group(1) in key:
        seen.add(m.group(1))
        line = '%s %s %s' % (m.group(1), key[m.group(1)], m.group(4))
    lines.append(line)
missing = set(key) - seen
if missing:
    sys.exit('not in the sample config: %s' % ', '.join(sorted(missing)))
open(dst, 'w').write('\n'.join(lines))
P
"$HEVCENC" -c "$name.enc.cfg" > "$name.enc.log" 2>&1 || { cat "$name.enc.log"; exit 1; }
[ -s "$name.265" ] || { echo "the encoder wrote nothing"; cat "$name.enc.log"; exit 1; }
cat > "$name.dec.cfg" <<C
--input $name.265
--save_output 1
--num_frames -1
--output $name.ref.yuv
--chroma_format YUV_420P
--share_display_buf 0
--max_wd 4096
--max_ht 2304
--max_level 62
--num_cores 1
--loopback 0
--display 0
--arch X86_GENERIC
--soc GENERIC
C
"$HEVCDEC" -c "$name.dec.cfg" > /dev/null
"$py" "$here/check-hevc-sei.py" "$name.265" "$name.ref.yuv" "$width" "$height" || { echo "libhevc's decode does not match the stream's picture hashes: not a usable reference"; exit 1; }
rm -rf "$name.jobs"
# 16777220 is MPP_VIDEO_CodingHEVC. mpi_dec_test reads the file again from the start once it is at the end, which makes one decode job more than FRAMES: --jobs ignores it.
VDEC_TRACE_DIR="$work/$name.jobs" LD_PRELOAD="$work/trace-shim.so" timeout 120 "$MPI_DEC_TEST" -t 16777220 -i "$name.265" -n "$frames" -o "$work/$name.mpp.yuv" > "$name.mpp.log" 2>&1 || true
python3 "$here/make-trace.py" "$name.jobs" --ref "$name.ref.yuv" --width "$width" --height "$height" --jobs "$frames" --mpp-out "$name.mpp.yuv" --out "$out/$name"
rm -f "$out/$name.inc"   # build-rk3588-vdec.py generates the C array at build time
