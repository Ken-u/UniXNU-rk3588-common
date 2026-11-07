#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Records the libhevc streams of ../streams (the six H.265 streams the kernel replays) with make-h265-trace.sh, twice each: with MPP's threaded test (-> ../traces, the
# traces the kernel replays) and with its no-thread test (-> ../traces/mpp-nt, what tests/run-core.sh compares the decoder core with), and keeps each stream and the CRC-32
# of its pictures in ../streams. libhevc's encoder is deterministic with one core, so both recordings code the same bytes; the script stops if they do not.
#
#   HEVCENC=... HEVCDEC=... HEVC_CFG=.../vid_enc_cfg.txt MPI_DEC_TEST=.../mpi_dec_test MPI_DEC_NT_TEST=.../mpi_dec_nt_test record-h265-streams.sh WORKDIR [NAME ...]
#
# Without NAMEs all of them. The tool set is the one make-h265-trace.sh lists (libhevc 48cb531, MPP 9b56348).
set -e
work=$1
[ -n "$work" ] || { sed -n 3,11p "$0"; exit 1; }
shift
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$work"
: "${HEVCENC:?}" "${HEVCDEC:?}" "${HEVC_CFG:?}" "${MPI_DEC_TEST:?}" "${MPI_DEC_NT_TEST:?}"
export HEVCENC HEVCDEC HEVC_CFG

# name width height frames "encoder keys"
record() {
  name=$1 w=$2 h=$3 n=$4 set=$5
  if [ -n "$WANT" ]; then case " $WANT " in *" $name "*) ;; *) return 0 ;; esac; fi
  mkdir -p "$work/$name-t" "$work/$name-nt"
  ENC_SET=$set MPI_DEC_TEST=$MPI_DEC_TEST STREAMS_OUT="$here/../streams" TRACE_OUT="$here/../traces" sh "$here/make-h265-trace.sh" "$name" "$w" "$h" "$n" "$work/$name-t" \
    > "$work/$name-t.log" 2>&1 || { tail -20 "$work/$name-t.log"; exit 1; }
  ENC_SET=$set MPI_DEC_TEST=$MPI_DEC_NT_TEST TRACE_OUT="$here/../traces/mpp-nt" sh "$here/make-h265-trace.sh" "$name" "$w" "$h" "$n" "$work/$name-nt" \
    > "$work/$name-nt.log" 2>&1 || { tail -20 "$work/$name-nt.log"; exit 1; }
  cmp "$work/$name-t/$name.265" "$work/$name-nt/$name.265"
  echo "$name: $(tail -1 "$work/$name-t.log")"
}

WANT=$*
record t320_hevc        320  240 10 ""
record t320_hevc_b      320  240 10 "--max_temporal_layers 2 --frame_qp 24"
record t320_hevc_sc     320  240 10 "--use_default_sc_mtx 1 --frame_qp 26"
record t320_hevc_nodbk  320  240 10 "--deblocking_type 1 --frame_qp 30"
record t320_hevc_wpp    320  240 10 "--enable_entropy_sync 1"
record t720_hevc        1280 720 10 "--codec_level 120"
