#!/usr/bin/env python3
"""U-Boot console helper, run on the machine the board's serial line is attached to.

    uboot-cmd.py "cmd1" "cmd2" ...

Each command is sent and the console is read until the next `=> ` prompt (or until the timeout); the echo is
printed.

Environment: SERIAL_DEV   serial device (default /dev/ttyUSB0)
             SERIAL_BAUD  line speed (default 1500000; the RK3588 debug UART runs at 1.5 Mbaud, not 115200)
             TO           timeout of each command in seconds (default 8)

Use it only when the board sits at the U-Boot prompt (get there with uboot_grab_cli of usb-lib.sh). Commands that
take the console over, such as `fastboot usb 0`, return no prompt: give them TO=3 and let them time out.
"""
import os
import re
import select
import subprocess
import sys
import time

DEV = os.environ.get("SERIAL_DEV", "/dev/ttyUSB0")
BAUD = os.environ.get("SERIAL_BAUD", "1500000")
subprocess.run(["stty", "-F", DEV, BAUD, "raw", "-echo"], check=False)
fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)


def drain(t=0.3):
    end = time.time() + t
    out = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                out += os.read(fd, 65536)
            except BlockingIOError:
                pass
    return out


def run(cmd, timeout=8.0, prompt=b"=> "):
    drain(0.2)
    os.write(fd, (cmd + "\r").encode())
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                buf += os.read(fd, 65536)
            except BlockingIOError:
                pass
            if buf.rstrip().endswith(prompt.rstrip()) and len(buf) > len(cmd):
                time.sleep(0.15)
                buf += drain(0.1)
                break
    t = re.sub(rb"\x1b\[[0-9;?]*[A-Za-z]", b"", buf).decode("latin1")
    return "".join(c for c in t if c in "\n\r\t" or ord(c) >= 32)


if __name__ == "__main__":
    to = float(os.environ.get("TO", "8"))
    for c in sys.argv[1:]:
        print(run(c, to).replace("\r", ""), end="")
