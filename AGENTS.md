# Agent instructions

This repository owns shared RK3588 resources and platform operations. Keep
board peripherals, firmware selection, partition layout, and power policy in
the device repository. Preserve runtime source filenames, classes, symbols,
and protocol IDs.

Read these UniXNU pages before editing:

1. [Porting](https://unixnu.com/porting/)
2. [Device repositories](https://unixnu.com/porting/device-repository)
3. [Components](https://unixnu.com/reference/components)
4. [Device trees](https://unixnu.com/porting/device-tree)
5. [Driver matching](https://unixnu.com/drivers/device-tree-matching)

Rules specific to this repository:

- The target is Mach-O. Inline assembly goes through the Darwin assembler,
  where `;` starts a comment. Separate instructions with `"\n\t"` and check
  the object with `llvm-objdump --disassemble-symbols`.
- A node reaches the Apple device tree only with `unixnu,adt-name` and
  `status` other than `disabled`. Do not add `unixnu,cluster-type` or
  `virtual-timer-intid`; the working kernel runs without them.
- XNU patches apply with plain `git apply` in series order, after the ARM64
  series and before the OS and device series. Check new patches against a
  fresh XNU clone.
- Do not use observations from other Rockchip SoCs to decide RK3588 behavior.
- Keep addresses, hostnames, accounts, and local paths out of this repository.
- `tools/board/` is host-side tooling shared by boards. Keep any board's USB ids, load addresses, boot flow, and
  payload out of it, and run `python3 tools/board/selftest.py` after editing it.

Report commands, results, required local inputs, and hardware checkpoints.
Write concise English describing current behavior.
