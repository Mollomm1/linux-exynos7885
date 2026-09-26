# T510 WiFi module iteration

This stage binds an **inert** module. It does not power the wireless
subsystem, write PMU registers, access mailboxes, request interrupts, or
create a network interface. Firmware can be explicitly read and validated in
host memory, but is never staged or executed. It is fresh work on `gta3xl`; the abandoned
`gta3xl-scsc` implementation is not used.

## Build and flash the baseline once

From the mainlining workspace, with the three aport files already synced:

```sh
pmbootstrap build linux-samsung-gta3xlwifi
pmbootstrap install --android-recovery-zip
pmbootstrap export
```

Check the build log for `DTC ... exynos7904-t510.dtb` and `Frozen baseline`.
Flash the exported recovery ZIP using the existing recovery procedure. The user
handles flashing. The booted release should be `6.15.0-rc1-exynos7904-wifi-dev1`.
Confirm normal boot, touchscreen and USB SSH before testing the module.

The build automatically saves the complete kernel source and build output to:

```
~/.local/var/pmbootstrap/cache_distfiles/gta3xl-wifi-baseline-<commit>-r14.tar.gz
```

Keep that archive and the pmbootstrap native chroot/toolchain. Extraction needs
several GB of space. The archive includes generated headers, symbol table,
kernel/DT images, configuration, compiler identity and installed package versions.
It stays on the host; only its small identity manifest is installed on the tablet.
The tool refuses to overwrite an existing archive. For an intentional baseline
rebuild, move the old archive aside and retain its matching image and modules.
Do not mix modules from those two builds even if `uname -r` happens to match.

The module has no autoload alias, is blacklisted, is absent from initramfs, and
requires `enable=1`. Merely installing this kernel does not execute WiFi code.
The new DT reserves 4 MiB at `0xe9000000`, matching the downstream board reference.

## Build just the module

From the mainlining workspace, select the archive created above explicitly:

```sh
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py pmb-build \
    --archive "$HOME/.local/var/pmbootstrap/cache_distfiles/gta3xl-wifi-baseline-<commit>-r14.tar.gz" \
    --source linux-exynos7885 --out wifi-iterations/001
```

Use a new output directory each time. `pmb-build` reuses the native chroot's
cross compiler and invokes external-module Kbuild against the frozen tree.
It does not rebuild the kernel, recovery ZIP or uniLoader. pmbootstrap may
require local sudo authentication. Do not zap/update the native build chroot;
compiler changes are rejected. The direct `build` subcommand is also available
inside that compiler environment. Only the single driver source file is copied;
extend its Kbuild staging explicitly if the driver later gains more files.

The output contains the candidate `.ko`, a compatibility/checksum manifest and,
after testing, logs. Keep earlier output directories to retain previous modules.
Host checks: `python3 -m unittest discover -s linux-exynos7885/tools/scsc-dev`.

## Upload and test over USB SSH

Verify the tablet's host key locally before updating SSH known_hosts. Configure
key authentication for `user@172.16.42.1` with noninteractive sudo/doas, or pass
`--target root@172.16.42.1` to each command. The tool never disables host-key
checking and checks board identity, `uname -r`, `uname -v` and the installed
baseline manifest before any module operation.

```sh
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py upload --out wifi-iterations/001
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py load --out wifi-iterations/001
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py status --out wifi-iterations/001
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py unload --out wifi-iterations/001
```

Upload creates a new root-owned directory in `/tmp`; it does not load anything
or replace the installed module. Re-uploading the same candidate refuses to
overwrite that directory. For the next iteration, upload its new candidate,
unload the current inert driver, then load the candidate explicitly.

After reviewing the first successful bind/unbind logs, test ten inert cycles:

```sh
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py cycle --out wifi-iterations/001
```

Start with the module unloaded. Review logs for the driver-reported resource
binding duration (well below one second), new warnings, and resource failures.
Cycle timings include SSH overhead. The loop stops on command failure or new
warning signatures; no automatic rollback/retry or forced unloading is used.
The tool stores kernel log tails and module lists without clearing dmesg.
Collect a full log separately if a warning has already scrolled out of the tail.

If insmod succeeds but binding fails, the module may remain loaded without a
`state` attribute. Preserve logs and inspect the cause before manually removing
this inert module; the helper deliberately refuses unload without `state=inert`.
An SSH timeout is not proof that a remote kernel operation has stopped.

## Firmware restart gate and remaining work

Firmware start/stop is not implemented. An explicit root write of
`postmarketos/mx140/mx140.bin` to the device's `verify_firmware` attribute checks
header/API version, lengths, entry bounds and all three CRCs. `firmware_status`
reports the result; `state` remains `inert`. `pmu_state` is a root-only read of
six fixed PMU registers, with no register writes or mailbox access. See
`Documentation/ABI/testing/sysfs-driver-exynos-scsc-wifibt` for the interfaces.

Installed/stock images and malformed/missing image cases have been tested on
the tablet. No command in this revision starts either wireless processor.
The source audit and remaining ACPM/rail prerequisites are recorded in
[POWER-PREREQUISITES.md](POWER-PREREQUISITES.md).

Downstream `platform_mif.c:platform_mif_pmu_reset()` configures reset-ahead,
bus cleanup, logic reset, TCXO gating, isolation and the central sequencer, then
asserts reset or removes power and waits for central-sequencer state `0x80`.
That is more than clearing START/PWRON. Before enabling firmware, establish
which sequence is valid here, preserve watchdog behavior, mask/synchronize
interrupts and cancel work, and verify quiescence before releasing shared memory.
Do not introduce firmware patches, halt stubs, broad PMU dumps or boot-time tests.
Use bounded operations and stop after any failed hardware handshake.

Active firmware must hold a module reference until an explicit stop has proven
quiescence: platform remove callbacks cannot veto unload. A failed stop must
leave the module pinned and memory reserved, requiring coordinated recovery.
Future active-state code must replace this inert-only test policy explicitly.

Device-tree, core-kernel, configuration or exported-interface changes require
a new baseline and reboot. Routine module edits can use this workflow. Reliable
firmware restart without reboot remains unproven; eventual scanning, association
and network traffic need additional firmware transport and cfg80211 work.
