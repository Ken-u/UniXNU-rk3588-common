# Board tools

Host-side tools for bringing up and observing an RK3588 board from the machine it is connected to. They are shared
by the RK3588 device and firmware repositories and are not part of the kernel build. They know nothing about one
board: the USB ids of a board's fastboot gadget, load addresses, the boot flow and the payload stay in the
repository of that board ([`uefi-evb10`](https://github.com/Ken-u/uefi-evb10) is the first user).

| File | Purpose |
| --- | --- |
| `serial-capture.py` | One reader on the serial line. Writes the bytes as they came and a copy with the elapsed seconds in front of each line. `--stop REGEX SECS WHY` ends a run early once a known failure has been printed |
| `uboot-cmd.py` | Sends commands to the U-Boot console and prints the answer up to the next `=> ` |
| `usb-lib.sh` | Sourced by shell tools: `has_usb`, `usb_ids`, `usb_stable` (the ids come from sysfs, not lsusb), and `uboot_grab_cli` (interrupt autoboot and wait for the prompt) |
| `hw-reset.sh` | Presses the board's RESET / RECOVERY / MASKROM keys through an ESP32-S3 controller, locally or through ssh. One attempt, no retry |
| `showlog.py` | Filters a serial log: drops VT escapes and control characters, selects lines by regular expression |
| `selftest.py` | Host test of all of the above: pseudo terminals for the serial lines, a directory tree for the USB bus |

## Environment

| Variable | Used by | Meaning |
| --- | --- | --- |
| `SERIAL_DEV` | `uboot-cmd.py` | Serial device, default `/dev/ttyUSB0` |
| `SERIAL_BAUD` | `uboot-cmd.py` | Line speed, default 1500000 (the RK3588 debug UART runs at 1.5 Mbaud, not 115200) |
| `TO` | `uboot-cmd.py` | Timeout of each command in seconds, default 8 |
| `USB_SYSFS_DIR` | `usb-lib.sh` | Where the USB devices are listed, default `/sys/bus/usb/devices` |
| `USB_VENDORS` | `usb-lib.sh` | Extended regular expression of the vendor ids `usb_ids` shows, default all |
| `RESET_TOOL_DIR` | `hw-reset.sh` | Directory of the controller tool (`tools/device.py` inside); required |
| `BOARD_HOST` | `hw-reset.sh` | ssh target of the machine the controller is plugged into, needed only when the script runs elsewhere |

`serial-capture.py` and `uboot-cmd.py` expect the line to be set already: `stty -F DEV 1500000 raw -echo`.

## Use from a repository

The tools that talk to the board run on the machine the board is connected to. A repository copies the files it
needs into the directory of its own scripts on that machine and sources `usb-lib.sh` from there:

```sh
scp tools/board/serial-capture.py tools/board/uboot-cmd.py tools/board/usb-lib.sh "$BOARD_HOST:dir/"
```

A repository finds this directory through its own configuration; `uefi-evb10` reads `RK3588_COMMON` from its
`.env` and defaults to the checkout next to it.

## Rules the tools keep

- One reader on a serial line at a time. A second capture, a terminal program or a script that opens the port
  garbles both.
- Nothing is sent to a board that is not on the bus: a blind burst of bytes has hurt a board before.
- `uboot_grab_cli` sends 25 Ctrl+C from a single descriptor, 100 ms apart. A faster burst while the board's USB
  gadget was still settling has killed the board, so the caller waits until the gadget has been stable
  (`usb_stable`) before it calls the function.
- A reset is requested once. Pressing RESET again in the middle of the boot chain breaks the boot, and the
  controller refuses a press within 3 s of the previous one.
- No board-specific value (USB id, address, host name, account) goes into this directory.

## Test

```sh
python3 tools/board/selftest.py
```

It needs Python 3.8 or newer, bash and coreutils, no board, and takes about 20 s.
