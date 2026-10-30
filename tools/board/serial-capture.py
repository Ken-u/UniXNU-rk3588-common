#!/usr/bin/env python3
"""Serial capture: the bytes as they came, plus a copy with the elapsed seconds in front of each line.

    serial-capture.py [--stop REGEX SECS WHY]... DEV SECS RAW TS

Equivalent to `timeout SECS cat DEV > RAW` (one reader; the line settings are the caller's:
`stty -F DEV <baud> raw -echo` first), and it writes TS besides:

    [  12.345] DARWINBOOT: hand-off to the kernel
    [ 100.020] DDR c82c943893 cym 26/01/23-15:55:28,fwver: v1.23

Why: on a board without a screen the time is the signal (how long after ExitBootServices the board resets,
where a boot stalls). A log made by cat has no times.

Time zero is the start of this script (monotonic clock). A line that stays unfinished for 0.5 s is written
with a trailing " ⏎"; the bytes that follow start a new line with their own time.

--stop REGEX SECS WHY   May be repeated. Stop SECS seconds after the first complete line that matches REGEX
                        (the first rule that matches wins) and write "# stopped early: WHY" to TS. For runs
                        whose result is void once a known failure has been printed, so that nobody waits out
                        the rest of SECS.
"""
import argparse
import os
import re
import select
import signal
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stop", nargs=3, action="append", default=[], metavar=("REGEX", "SECS", "WHY"))
    ap.add_argument("dev")
    ap.add_argument("secs", type=float)
    ap.add_argument("raw")
    ap.add_argument("ts")
    args = ap.parse_args()
    rules = [(re.compile(r.encode()), float(s), why) for r, s, why in args.stop]

    fd = os.open(args.dev, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    raw = open(args.raw, "wb", buffering=0)
    ts = open(args.ts, "w", buffering=1, encoding="utf-8", errors="replace")
    t0 = time.monotonic()
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    ts.write("# capture start %s, dev %s, %.0f s\n" % (time.strftime("%F %T"), args.dev, args.secs))

    buf = b""
    line_t = 0.0
    deadline = None
    why = ""

    def clean(b):
        out = []
        for c in b.decode("latin-1"):
            if c == "\r" or c == "\x00":
                continue
            out.append(c if (c == "\t" or " " <= c <= "~") else "?")
        return "".join(out)

    def emit(b, t, partial=False):
        ts.write("[%8.3f] %s%s\n" % (t, clean(b), " ⏎" if partial else ""))

    while not stop:
        now = time.monotonic() - t0
        if now >= args.secs:
            break
        if deadline is not None and now >= deadline:
            ts.write("# stopped early: %s\n" % why)
            break
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                data = b""
            except OSError as e:
                ts.write("# read error: %s\n" % e)
                break
            if data:
                raw.write(data)
                if not buf:
                    line_t = time.monotonic() - t0
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    emit(line, line_t)
                    if deadline is None:
                        for rx, secs, reason in rules:
                            if rx.search(line):
                                deadline = (time.monotonic() - t0) + secs
                                why = reason
                                break
                    line_t = time.monotonic() - t0
        elif buf and (time.monotonic() - t0 - line_t) > 0.5:
            emit(buf, line_t, partial=True)
            buf = b""
    if buf:
        emit(buf, line_t, partial=True)
    ts.write("# capture end %.3f s, %d raw bytes\n" % (time.monotonic() - t0, os.fstat(raw.fileno()).st_size))
    ts.close()
    raw.close()
    os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
