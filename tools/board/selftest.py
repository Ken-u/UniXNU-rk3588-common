#!/usr/bin/env python3
"""Host self test of the board tools. No board is needed: the serial lines are pseudo terminals and the USB
bus is a directory tree.

    python3 selftest.py

Covers serial-capture.py (raw copy, timestamps, partial lines, --stop rules), uboot-cmd.py, usb-lib.sh
(has_usb, usb_ids, usb_stable with a device that comes late or flaps, uboot_grab_cli) and hw-reset.sh.
Needs python3 3.8 or newer, bash and coreutils; takes about 20 s.
"""
import os
import pty
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
failures = []


def check(name, ok, detail=""):
    print(("ok    " if ok else "FAIL  ") + name + ("" if ok or not detail else ": " + detail))
    if not ok:
        failures.append(name)


class FakeLine:
    """A pseudo terminal in raw mode; `path` is the slave to hand to the tool, `master` is the board's side."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.received = b""
        self._stop = False
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        self.on_data = None

    def _read(self):
        import select
        while not self._stop:
            r, _, _ = select.select([self.master], [], [], 0.05)
            if not r:
                continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            self.received += data
            if self.on_data:
                self.on_data(self, data)

    def say(self, data):
        os.write(self.master, data)

    def close(self):
        self._stop = True
        self._reader.join(timeout=1)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def run(args, env=None, timeout=60, **kw):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(args, env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                          timeout=timeout, **kw)


def test_serial_capture(tmp):
    line = FakeLine()
    raw, ts = os.path.join(tmp, "a.raw"), os.path.join(tmp, "a.ts")
    p = subprocess.Popen([sys.executable, os.path.join(HERE, "serial-capture.py"), line.path, "2", raw, ts])
    time.sleep(0.4)
    line.say(b"first line\r\nsecond line\r\n")
    time.sleep(0.3)
    line.say(b"unfinished")
    p.wait(timeout=10)
    text = open(ts, encoding="utf-8").read()
    check("capture: raw copy is byte for byte", open(raw, "rb").read() == b"first line\r\nsecond line\r\nunfinished")
    check("capture: lines carry seconds", re.search(r"^\[ *\d+\.\d{3}\] first line$", text, re.M) is not None, text)
    check("capture: an unfinished line is marked", re.search(r"\] unfinished ⏎$", text, re.M) is not None, text)
    check("capture: header and trailer", text.startswith("# capture start") and "# capture end" in text, text)
    line.close()

    line = FakeLine()
    raw, ts = os.path.join(tmp, "b.raw"), os.path.join(tmp, "b.ts")
    t0 = time.time()
    p = subprocess.Popen([sys.executable, os.path.join(HERE, "serial-capture.py"),
                          "--stop", "AAA", "1", "rule a", "--stop", "BBB", "2", "rule b",
                          line.path, "30", raw, ts])
    time.sleep(0.4)
    line.say(b"fine\r\nBBB and AAA together\r\n")
    for _ in range(10):
        time.sleep(0.2)
        line.say(b"more\r\n")
    p.wait(timeout=15)
    took = time.time() - t0
    text = open(ts, encoding="utf-8").read()
    check("capture: --stop ends the run early", took < 6, "%.1f s" % took)
    check("capture: the first matching rule gives the reason", "# stopped early: rule a" in text, text)
    line.close()

    line = FakeLine()
    raw, ts = os.path.join(tmp, "c.raw"), os.path.join(tmp, "c.ts")
    p = subprocess.Popen([sys.executable, os.path.join(HERE, "serial-capture.py"),
                          "--stop", "NEVER", "1", "unused", line.path, "1.5", raw, ts])
    time.sleep(0.3)
    line.say(b"nothing special\r\n")
    p.wait(timeout=10)
    text = open(ts, encoding="utf-8").read()
    check("capture: a rule that never matches changes nothing", "stopped early" not in text and "# capture end" in text, text)
    line.close()


def test_uboot_cmd():
    line = FakeLine()

    def uboot(fake, data):
        for part in data.split(b"\r"):
            if part:
                fake.say(part + b"\r\nanswer to " + part + b"\r\n=> ")

    line.on_data = uboot
    out = run([sys.executable, os.path.join(HERE, "uboot-cmd.py"), "printenv foo", "version"],
              env={"SERIAL_DEV": line.path, "TO": "3"})
    check("uboot-cmd: every command is answered up to the prompt",
          "answer to printenv foo" in out.stdout and "answer to version" in out.stdout, out.stdout + out.stderr)
    line.close()


def bash(script, env=None, timeout=60):
    return run(["bash", "-c", script], env=env, timeout=timeout)


def put_usb(root, name, vid, pid):
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, "idVendor"), "w").write(vid + "\n")
    open(os.path.join(d, "idProduct"), "w").write(pid + "\n")


def test_usb_lib(tmp):
    bus = os.path.join(tmp, "bus")
    put_usb(bus, "1-1", "18d1", "4d00")
    put_usb(bus, "1-2", "2207", "350b")
    put_usb(bus, "usb1", "1d6b", "0002")
    os.makedirs(os.path.join(bus, "1-1:1.0"))  # interfaces have no ids
    lib = os.path.join(HERE, "usb-lib.sh")
    env = {"USB_SYSFS_DIR": bus}
    pre = '. "%s"; ' % lib
    check("usb-lib: has_usb finds a device", bash(pre + "has_usb 18d1:4d00", env).returncode == 0)
    check("usb-lib: has_usb does not take a prefix", bash(pre + "has_usb 18d1:4d0", env).returncode != 0)
    check("usb-lib: has_usb misses an absent device", bash(pre + "has_usb 18d1:d001", env).returncode != 0)
    out = bash(pre + "USB_VENDORS='18d1|2207' usb_ids", env).stdout
    check("usb-lib: usb_ids filters by vendor", out == "18d1:4d00 2207:350b ", repr(out))
    out = bash(pre + "usb_ids", env).stdout
    check("usb-lib: usb_ids shows all without a filter", "1d6b:0002" in out, repr(out))

    t0 = time.time()
    rc = bash(pre + "usb_stable 18d1:4d00 3 20", env).returncode
    check("usb-lib: usb_stable succeeds for a present device", rc == 0 and time.time() - t0 < 3)
    t0 = time.time()
    rc = bash(pre + "usb_stable 18d1:d001 2 5", env).returncode
    check("usb-lib: usb_stable gives up on an absent device", rc == 1 and 0.8 < time.time() - t0 < 4)

    def late():
        time.sleep(1.0)
        put_usb(bus, "1-3", "18d1", "d001")

    threading.Thread(target=late, daemon=True).start()
    t0 = time.time()
    rc = bash(pre + "usb_stable 18d1:d001 3 40", env).returncode
    check("usb-lib: usb_stable waits for a device that comes late", rc == 0 and time.time() - t0 > 1.2)

    def flap():
        time.sleep(0.5)
        put_usb(bus, "1-4", "18d1", "beef")
        time.sleep(0.4)
        shutil.rmtree(os.path.join(bus, "1-4"))
        time.sleep(0.6)
        put_usb(bus, "1-4", "18d1", "beef")

    threading.Thread(target=flap, daemon=True).start()
    t0 = time.time()
    rc = bash(pre + "usb_stable 18d1:beef 4 60", env).returncode
    took = time.time() - t0
    check("usb-lib: a device that drops out resets the count", rc == 0 and took > 2.0, "%.1f s" % took)


def test_grab_cli(tmp):
    lib = os.path.join(HERE, "usb-lib.sh")
    for answers in (True, False):
        line = FakeLine()

        def uboot(fake, data):
            seen = fake.received
            if answers and seen.count(b"\x03") >= 25 and seen.endswith(b"\r\n") and not getattr(fake, "sent", False):
                fake.sent = True
                fake.say(b"\r\n=> ")

        line.on_data = uboot
        log = os.path.join(tmp, "grab.log")
        out = bash('. "%s"; uboot_grab_cli "%s" "%s"' % (lib, line.path, log), timeout=30)
        if answers:
            check("usb-lib: uboot_grab_cli reports the prompt", out.returncode == 0, out.stderr)
            check("usb-lib: uboot_grab_cli sends 25 Ctrl+C then a newline",
                  line.received.count(b"\x03") == 25 and line.received.endswith(b"\r\n"), repr(line.received))
        else:
            check("usb-lib: uboot_grab_cli fails without a prompt", out.returncode != 0)
        line.close()


def test_hw_reset(tmp):
    tool = os.path.join(tmp, "reset", "tools")
    os.makedirs(tool)
    open(os.path.join(tool, "device.py"), "w").write("import sys\nprint('device.py ' + ' '.join(sys.argv[1:]))\n")
    script = os.path.join(HERE, "hw-reset.sh")
    env = {"RESET_TOOL_DIR": os.path.join(tmp, "reset"), "BOARD_HOST": ""}
    out = run(["bash", script, "loader", "--hold-ms", "3000"], env=env)
    check("hw-reset: arguments reach the controller tool", out.returncode == 0 and out.stdout == "device.py loader --hold-ms 3000\n", out.stdout + out.stderr)
    out = run(["bash", script, "frobnicate"], env=env)
    check("hw-reset: an unknown subcommand is refused", out.returncode == 2)
    e = dict(env)
    e["RESET_TOOL_DIR"] = os.path.join(tmp, "nowhere")
    out = run(["bash", script, "reset"], env=e)
    check("hw-reset: no tool and no host is an error, not a retry", out.returncode == 2 and "BOARD_HOST" in out.stderr, out.stderr)


def main():
    tmp = tempfile.mkdtemp(prefix="board-selftest-")
    try:
        test_serial_capture(tmp)
        test_uboot_cmd()
        test_usb_lib(tmp)
        test_grab_cli(tmp)
        test_hw_reset(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("%d check(s) failed" % len(failures) if failures else "all checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
