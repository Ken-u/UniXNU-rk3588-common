#!/bin/bash
# Hardware reset: an ESP32-S3 controller next to the board presses its RESET / RECOVERY / MASKROM keys.
#
#   hw-reset.sh status               is the controller there (read only, presses nothing)
#   hw-reset.sh reset                press RESET: a hard reset of the whole board
#   hw-reset.sh loader               RECOVERY + RESET: the board comes up in Rockchip Loader (rockusb) mode
#   hw-reset.sh maskrom              MASKROM combination: the board stops in the BootROM (maskrom) mode
#   hw-reset.sh loader --hold-ms 3000    how long the mode key is held after RESET is released (100..10000 ms,
#                                        default 1500)
#   hw-reset.sh usb-status | result  the controller's own state / receipt of its last operation
#
# Environment: RESET_TOOL_DIR  directory of the controller tool (tools/device.py inside); required
#              BOARD_HOST      ssh target of the machine the controller is plugged into; needed only when this
#                              script runs somewhere else (key authentication, BatchMode: it never asks for a
#                              password)
#
# Run on the controller's machine it calls the tool directly; run elsewhere it goes through ssh.
# tools/device.py needs python3 and pyserial only.
#
# Semantics:
#   * One call is one key-press request. There is no automatic retry and a timeout is not resent, so this script
#     does not retry either: pressing RESET again in the middle of the boot chain (BootROM, SPL, U-Boot, UEFI)
#     breaks the boot. After a failure look at the serial log before deciding anything.
#   * The controller has a 3 s cooldown of its own; a press right after another is refused.
#   * It only presses keys. It does not prove that a board which powered itself off can be powered on again, so
#     a payload must not offer a shutdown entry.
#   * The controller's serial port (an ACM device) is used exclusively: no terminal program at the same time.
#   * Do not press while a serial capture is meant to run on: the reset interrupts it.
#
# Exit status: 0 = the controller accepted; anything else = failure (read the output).
set -euo pipefail

BOARD_HOST="${BOARD_HOST:-}"
TOOL_DIR="${RESET_TOOL_DIR:?set RESET_TOOL_DIR (the reset controller tool directory on the board host)}"

usage() {
	sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
	exit 2
}

[ $# -ge 1 ] || usage
cmd="$1"
shift
case "$cmd" in
status | reset | loader | maskrom | usb-status | result) ;;
-h | --help | help) usage ;;
*)
	echo "error: unknown subcommand $cmd" >&2
	usage
	;;
esac

if [ -f "$TOOL_DIR/tools/device.py" ]; then
	cd "$TOOL_DIR"
	exec python3 tools/device.py "$cmd" "$@"
fi

quoted=""
for a in "$cmd" "$@"; do quoted+=" $(printf '%q' "$a")"; done
[ -n "$BOARD_HOST" ] || { echo "error: set BOARD_HOST (the tool is not on this machine)" >&2; exit 2; }
exec ssh -o BatchMode=yes -o ConnectTimeout=10 "$BOARD_HOST" \
	"bash -c 'cd $(printf '%q' "$TOOL_DIR") && exec python3 tools/device.py$quoted'"
