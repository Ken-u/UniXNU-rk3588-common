#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause-Patent
#
# Records the x264 streams of ../streams (High profile tools, several slices, MBAFF, scaling matrices, crop, 720p and 1080p) with make-h264-trace.sh, twice each:
# with MPP's threaded test (-> ../traces, the traces the kernel replays) and with its no-thread test (-> ../traces/mpp-nt, what tests/run-core.sh compares the
# decoder core with), and keeps each stream and the CRC-32 of its pictures in ../streams. The two recordings must have coded the same bytes (they are the same
# x264 run: x264 with one thread is deterministic) or the script stops.
#
#   X264=/path/x264 AVCDEC=... AVCENC=... MPI_DEC_TEST=.../mpi_dec_test MPI_DEC_NT_TEST=.../mpi_dec_nt_test record-h264-streams.sh WORKDIR [NAME ...]
#
# Without NAMEs all of them. The tool set is the one make-h264-trace.sh lists (x264 0480cb0, libavc 4275faf, MPP 9b56348). The libavc-made Main profile streams (t320_bframes,
# t320_cabac, t320_cavlc, t720_cabac) are not here: make-h264-trace.sh without X264, see its header.
#
# Interlaced streams (MBAFF, frame_mbs_only_flag = 0) code pairs of macroblock rows: a height that is not a multiple of 32 is padded and cropped by x264, and then the
# picture the block writes (padded) is not the one the references are of (cropped), so those streams are 320x256 and make-h264-trace.sh refuses other heights.
set -e
work=$1
[ -n "$work" ] || { sed -n 3,19p "$0"; exit 1; }
shift
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$work"
export X264_ARGS enable_deinterlace=0
: "${X264:?}" "${AVCDEC:?}" "${AVCENC:?}" "${MPI_DEC_TEST:?}" "${MPI_DEC_NT_TEST:?}"
export X264 AVCDEC AVCENC

# name width height frames crop|- "x264 options"
record() {
  name=$1 w=$2 h=$3 n=$4 crop=$5 args=$6
  if [ -n "$WANT" ]; then case " $WANT " in *" $name "*) ;; *) return 0 ;; esac; fi
  X264_ARGS=$args
  export X264_ARGS
  unset CROP
  [ "$crop" = - ] || export CROP=$crop
  mkdir -p "$work/$name-t" "$work/$name-nt"
  MPI_DEC_TEST=$MPI_DEC_TEST STREAMS_OUT="$here/../streams" TRACE_OUT="$here/../traces" sh "$here/make-h264-trace.sh" "$name" "$w" "$h" "$n" "$work/$name-t" > "$work/$name-t.log" 2>&1 \
    || { tail -20 "$work/$name-t.log"; exit 1; }
  MPI_DEC_TEST=$MPI_DEC_NT_TEST TRACE_OUT="$here/../traces/mpp-nt" sh "$here/make-h264-trace.sh" "$name" "$w" "$h" "$n" "$work/$name-nt" > "$work/$name-nt.log" 2>&1 \
    || { tail -20 "$work/$name-nt.log"; exit 1; }
  cmp "$work/$name-t/$name.h264" "$work/$name-nt/$name.h264"
  echo "$name: $(tail -1 "$work/$name-t.log")"
}

WANT=$*
H="--profile high --bframes 2 --ref 3"
record t320_high             320  240 10 -       "$H"
record t320_high_cavlc       320  240 10 -       "$H --no-cabac"
record t320_high_cqm         320  240 10 -       "$H --cqm jvt"
record t320_high_slices      320  240 10 -       "$H --slices 4"
record t320_high_mbaff       320  256 10 -       "$H --interlaced"
record t320_high_mbaff_slices 320 256 10 -       "$H --interlaced --slices 3"
record t320_high_fakeil      320  256 10 -       "$H --fake-interlaced"
record t320_high_refs        320  240 10 -       "--profile high --bframes 3 --ref 6 --b-pyramid normal --b-adapt 2"
record t320_high_p           320  240 10 -       "--profile high --bframes 0 --ref 4"
record t320_high_idr         320  240 10 -       "--profile high --bframes 0 --keyint 1 --min-keyint 1"
record t320c_high            320  240 10 0,0,8,6 "$H"
record t720_high             1280 720 10 -       "$H"
record t1080_high            1920 1088 6 0,0,0,8 "$H"
