# Sourced by the board tools, on the machine the board is connected to.
#
#   usb_list                 "VID:PID" of every USB device, one per line
#   has_usb VID:PID          succeeds when that device is on the bus
#   usb_ids                  one line with the ids whose vendor matches $USB_VENDORS
#   usb_stable VID:PID [N] [TRIES]
#                            succeeds after N (default 5) looks in a row, 0.2 s apart, that all see the
#                            device; gives up after TRIES (default 150) looks
#   uboot_grab_cli DEV LOG   interrupt U-Boot's autoboot over the serial line DEV and report whether the
#                            prompt "=> " came back (the console text goes to LOG)
#
# The ids are read from sysfs, not from lsusb: lsusb asks every device for its descriptor, and when a hub on
# the way drops and reconnects each call takes 6 to 8 s, so a "stable for N looks" test never completes.
#
# Environment: USB_SYSFS_DIR   where the devices are listed (default /sys/bus/usb/devices; the self test points
#                              it at a fake tree)
#              USB_VENDORS     extended regular expression of the vendor ids usb_ids shows (default: all)
#
# What the ids mean (a board's fastboot gadget, recovery, rockusb) is the caller's business; this file knows none.

usb_list() {
	local d
	for d in "${USB_SYSFS_DIR:-/sys/bus/usb/devices}"/*; do
		[ -r "$d/idVendor" ] && echo "$(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null)"
	done
}

has_usb() { usb_list | grep -q "^$1\$"; }

usb_ids() {
	local vendors="${USB_VENDORS:-}"
	[ -n "$vendors" ] || vendors='[0-9a-f]{4}'
	usb_list | grep -E "^($vendors):[0-9a-f]{4}\$" | tr '\n' ' '
}

usb_stable() {
	local id="$1" need="${2:-5}" tries="${3:-150}" n=0 i
	for i in $(seq 1 "$tries"); do
		if has_usb "$id"; then n=$((n + 1)); else n=0; fi
		[ "$n" -ge "$need" ] && return 0
		sleep 0.2
	done
	return 1
}

# Send Ctrl+C from one file descriptor, 25 times 100 ms apart, then a newline, while a reader collects what the
# console prints. Call it only when the board is known to be sitting in U-Boot's autoboot countdown or fastboot
# (a board whose gadget has not settled has died of a faster burst), and never while another reader has the line.
# The caller has set the line (`stty -F DEV <baud> raw -echo`).
uboot_grab_cli() {
	local ser="$1" log="$2" cap
	pkill -f "^(timeout [0-9]+ )?cat $ser\$" 2>/dev/null # only our own reader: the whole command line is anchored
	sleep 0.3
	: > "$log"
	timeout 30 cat "$ser" >> "$log" 2>/dev/null &
	cap=$!
	sleep 0.3
	python3 - "$ser" <<'PY'
import os, sys, time
fd = os.open(sys.argv[1], os.O_WRONLY | os.O_NOCTTY)
for _ in range(25):
    os.write(fd, b"\x03")
    time.sleep(0.1)
os.close(fd)
PY
	sleep 1.5
	printf '\r\n' > "$ser"
	sleep 1.5
	kill "$cap" 2>/dev/null
	wait "$cap" 2>/dev/null
	tr -d '\000' < "$log" | grep -qa '=> '
}
