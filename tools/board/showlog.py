#!/usr/bin/env python3
"""Read a serial log: drop VT escapes and control characters, filter lines by a regular expression, print the
last N.

    showlog.py LOG [REGEX [N]]

Example: showlog.py launch-ser.log 'UEFI|DARWINBOOT|panic' 60
"""
import re
import sys

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(2)
raw = open(sys.argv[1], "rb").read().replace(b"\x00", b"")
t = re.sub(rb"\x1b\[[0-9;?]*[A-Za-z]", b"", raw).decode("latin1")
t = "".join(c for c in t if c in "\n\r\t" or ord(c) >= 32)
lines = [l for l in t.replace("\r", "\n").split("\n") if l.strip()]
pat = re.compile(sys.argv[2]) if len(sys.argv) > 2 else None
n = int(sys.argv[3]) if len(sys.argv) > 3 else 40
print(len(raw), "bytes", len(lines), "lines")
for l in [l for l in lines if (pat.search(l) if pat else True)][-n:]:
    print(l[:230])
