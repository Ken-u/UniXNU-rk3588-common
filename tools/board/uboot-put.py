#!/usr/bin/env python3
"""Load a file into the RAM of a board that sits at the U-Boot prompt, over the serial line (YMODEM), and check it.

    uboot-put.py [--dev DEV] [--baud N] FILE [ADDR]

Runs on the machine the serial line is attached to, like uboot-cmd.py. U-Boot's `loady ADDR BAUD` receives the file; the
CRC-32 that U-Boot computes over it (`crc32 ADDR SIZE`) must equal the one computed here, otherwise the exit status is 1.
ADDR defaults to 0x1000000 (16 MiB into the RAM of the RK3588, free while U-Boot sits at its prompt). The file is not
written anywhere else: follow it with
    fatwrite nvme 0:1 ADDR PATH SIZE         (uboot-cmd.py "fatwrite nvme 0:1 0x1000000 EFI/UniXNU/unixnu.toml 0x75a")
to put it into a file of the ESP. U-Boot reads every number as hex: SIZE without the 0x prefix is taken as hex too, and
the file then grows by the bytes that follow in RAM (the printed last line shows the size in hex). `fatload` and `crc32`
read it back; the `N bytes read` line of fatload must equal the size of the file.

Environment: SERIAL_DEV  serial device (default /dev/ttyUSB0)   SERIAL_BAUD  line speed (default 1500000)
The device must not be open elsewhere (one reader at a time). Prints one line per step; nothing else is written to the
line besides the `loady` command, the YMODEM packets and a final newline.
"""
import argparse
import os
import re
import select
import subprocess
import sys
import time
import zlib

SOH, STX, EOT, ACK, NAK, CAN, CPMEOF = 0x01, 0x02, 0x04, 0x06, 0x15, 0x18, 0x1A


def crc16(data):
    """CRC-16/XMODEM (poly 0x1021, init 0), the check of YMODEM-CRC packets."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def packet(seq, payload, size):
    """One packet: SOH (128 bytes) or STX (1024 bytes), sequence number and its complement, payload padded with
    CPMEOF (or zeros in the header packet), CRC-16 big endian."""
    pad = 0 if seq == 0 else CPMEOF
    body = payload + bytes([pad]) * (size - len(payload))
    return bytes([SOH if size == 128 else STX, seq & 0xFF, 0xFF - (seq & 0xFF)]) + body + crc16(body).to_bytes(2, "big")


def ymodem_packets(name, data, block=1024):
    """The packets of a one-file YMODEM transfer, in order: header (name and length), data, and the empty header
    that ends the batch. EOT goes between the last two (the sender handles it)."""
    header = packet(0, name.encode() + b"\0" + str(len(data)).encode() + b"\0", 128)
    blocks = [packet(index, data[offset:offset + block], block)
              for index, offset in enumerate(range(0, len(data), block), start=1)]
    end = packet(0, b"", 128)
    return header, blocks, end


class Line:
    def __init__(self, dev, baud):
        subprocess.run(["stty", "-F", dev, str(baud), "raw", "-echo", "-ixon", "-ixoff"], check=False)
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.seen = b""

    def read(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            data = os.read(self.fd, 65536)
        except BlockingIOError:
            return b""
        self.seen += data
        return data

    def write(self, data):
        view = memoryview(data)
        while view:
            try:
                sent = os.write(self.fd, view)
                view = view[sent:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.5)

    def wait_for(self, pattern, timeout, since=0):
        """Read until the regex (bytes) matches what arrived from offset `since` of `seen` on; the match or None."""
        rx, end = re.compile(pattern), time.monotonic() + timeout
        while True:
            m = rx.search(self.seen[since:])
            if m:
                return m
            if time.monotonic() >= end:
                return None
            self.read(0.1)

    def expect_byte(self, wanted, timeout):
        """Next byte among `wanted` (a set); other bytes (console text) are skipped. None on timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for byte in self.read(0.1):
                if byte in wanted:
                    return byte
        return None


def send_ymodem(line, name, data):
    header, blocks, end = ymodem_packets(name, data)
    if line.expect_byte({ord("C")}, 15) is None:
        raise SystemExit("uboot-put: U-Boot did not start the YMODEM receiver (no 'C' within 15 s)")
    line.write(header)
    if line.expect_byte({ACK, NAK, CAN}, 5) != ACK:
        raise SystemExit("uboot-put: the header packet was refused")
    line.expect_byte({ord("C")}, 5)
    for number, pkt in enumerate(blocks, start=1):
        for attempt in range(10):
            line.write(pkt)
            answer = line.expect_byte({ACK, NAK, CAN}, 5)
            if answer == ACK:
                break
            if answer == CAN:
                raise SystemExit("uboot-put: the receiver cancelled at packet %d" % number)
        else:
            raise SystemExit("uboot-put: packet %d was not acknowledged after 10 tries" % number)
    for attempt in range(5):
        line.write(bytes([EOT]))
        if line.expect_byte({ACK, NAK}, 3) == ACK:
            break
    else:
        raise SystemExit("uboot-put: EOT was not acknowledged")
    line.expect_byte({ord("C")}, 3)
    line.write(end)
    line.expect_byte({ACK, NAK}, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dev", default=os.environ.get("SERIAL_DEV", "/dev/ttyUSB0"))
    ap.add_argument("--baud", type=int, default=int(os.environ.get("SERIAL_BAUD", "1500000")))
    ap.add_argument("file")
    ap.add_argument("addr", nargs="?", default="0x1000000")
    args = ap.parse_args()
    data = open(args.file, "rb").read()
    if not data:
        raise SystemExit("uboot-put: %s is empty" % args.file)
    addr = int(args.addr, 0)
    line = Line(args.dev, args.baud)
    mark = len(line.seen)
    line.write(b"\r")
    if not line.wait_for(rb"=> $", 5, mark):
        raise SystemExit("uboot-put: no U-Boot prompt ('=> ') on %s" % args.dev)
    mark = len(line.seen)
    line.write(("loady 0x%x %d\r" % (addr, args.baud)).encode())
    send_ymodem(line, os.path.basename(args.file), data)
    if not line.wait_for(rb"=> $", 10, mark):
        raise SystemExit("uboot-put: no prompt after the transfer")
    # The Orange Pi 5's U-Boot ends its receiver with "Timed out / ## Binary (ymodem) download aborted" after the last packet
    # was acknowledged; other builds print "## Total Size". The data is in RAM either way: the CRC below decides.
    tail = line.seen[mark:].replace(b"\x18", b"").replace(b"\x08", b"").decode("latin1").strip().splitlines()
    print("uboot-put: %s, %d bytes sent; U-Boot after the transfer: %s" % (os.path.basename(args.file), len(data),
                                                                          " | ".join(t for t in tail[-3:-1] if t.strip()) or "-"))
    mark = len(line.seen)
    line.write(("crc32 0x%x 0x%x\r" % (addr, len(data))).encode())
    m = line.wait_for(rb"==> ([0-9a-fA-F]{8})\s", 10, mark)
    want = "%08x" % (zlib.crc32(data) & 0xFFFFFFFF)
    have = m.group(1).decode().lower() if m else None
    print("uboot-put: crc32 of the first %d bytes at 0x%x: U-Boot %s, here %s" % (len(data), addr, have, want))
    if have != want:
        return 1
    print("uboot-put: ok (at the U-Boot prompt: fatwrite nvme 0:1 0x%x PATH 0x%x)" % (addr, len(data)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
