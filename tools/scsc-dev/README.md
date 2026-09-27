# T510 WiFi module iteration

The current `gta3xl` baseline is aport r17, pinned to kernel commit
`78e1f99c4291b3861151c4f900d967d56e60543f`. It boots as
`6.15.0-rc1-exynos7904-wifi-dev1`; its DT reserves 4 MiB at `0xe9000000`.
The image has been flashed. Its DT now exposes the R4, M4 and CP mailbox
windows. Keep this baseline and its saved build archive while iterating on code
that fits in the out-of-tree module.

The explicit `enable=1` module does not autoload. It can validate and stage the
installed `postmarketos/mx140/mx140.bin`, configure its reserved DRAM window
through the WLBT TZASC secure call and PMU BAAW registers, and build the
firmware's shared-memory ring/configuration layout in reserved DRAM. It does not
power WLBT, alter reset, access the mailbox registers, enable the mailbox IRQ,
release either processor, or register a network interface. Probe reserves the
MBOX IRQ with `IRQF_NO_AUTOEN`; the interrupt dispatcher remains inactive.
`state=inert` means firmware is not running; TZASC and the BAAW aperture may
remain configured after the module is unloaded. No `wlan` interface exists yet.

## Build the baseline once

From the mainlining workspace, with the aport files synced:

```sh
pmbootstrap build linux-samsung-gta3xlwifi
pmbootstrap install --android-recovery-zip
pmbootstrap export
```

Check the log for `DTC ... exynos7904-t510.dtb` and `Frozen baseline`. Flash
the exported recovery ZIP using the established recovery procedure. Confirm a
normal boot, touchscreen and USB SSH. The user handles flashing.

The build saves the complete kernel source and output in pmbootstrap's
`cache_distfiles`, under an archive named
`gta3xl-wifi-baseline-<commit>-r17.tar.gz`. Keep it with the matching image and
native compiler environment. The archive includes generated headers, symbol
tables, configuration, DT images and compiler identity. It remains on the host;
only a small manifest is installed on the tablet. Never mix modules from two
baseline archives with the same `uname -r`.

## Build module-only iterations

Select the matching r17 archive explicitly:

```sh
SCSC_BASELINE_DIR="$HOME/.cache/t510-wifi-dev/baseline-r17" \
    bash wifi-dev/build-module.sh 045
```

Or use `scsc-dev.py pmb-build` with the matching archive in pmbootstrap's
`cache_distfiles`. Both routes build an external module against the frozen
kernel configuration and generated files. They do not rebuild the kernel,
recovery ZIP or uniLoader. Use a new iteration directory each time. The private
unprivileged build environment is under `$HOME/.cache/t510-wifi-dev`; do not
replace its toolchain. Host checks are:

```sh
python3 -m unittest discover -s linux-exynos7885/tools/scsc-dev
```

## Upload and test over USB SSH

The module helper checks the board, kernel release/version and installed
baseline manifest. It uses strict host-key checking and root SSH or
noninteractive sudo/doas. Upload creates a new root-owned directory under
`/tmp`; it does not load the module. A module hash can only be uploaded once to
that device, so build a new artifact for each iteration.

```sh
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py upload \
    --target root@172.16.42.1 --out wifi-iterations/033-prepare-config-errors
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py load \
    --target root@172.16.42.1 --out wifi-iterations/033-prepare-config-errors
ssh root@172.16.42.1 'd=/sys/bus/platform/devices/120c0000.wifibt; \
    printf 1 > "$d/stage_firmware"; printf 1 > "$d/prepare_memory"; \
    printf 1 > "$d/prepare_config"; \
    cat "$d/firmware_status"; cat "$d/pmu_state"'
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py unload \
    --target root@172.16.42.1 --out wifi-iterations/033-prepare-config-errors
```

Run only the operation being tested. Firmware staging writes and verifies the
firmware in reserved DRAM. `prepare_memory` additionally configures the EL3
TZASC grant and PMU BAAW aperture after confirming PWRON, START and WIFI_STAT
show the block off. Neither operation starts firmware. Both return promptly.
`prepare_config` allocates the seven downstream-defined management, GDB and
mxlog rings after the firmware runtime image, writes the mxconf v0.1 header and
stream records, and verifies the config readback. It only touches the reserved
DRAM aperture. On boot `3e212fde-60e7-4649-9e77-75ac2ada202c`, the config
readback passed at offset `0x1d4248` after ACPM preparation, firmware staging
and DRAM setup. `state` remained `inert`; both modules unloaded cleanly, and
only `lo` and `usb0` were present. Firmware was not started. The helper's
`load`, `unload` and `cycle` commands are intended for the non-running state
only. If bind/unbind fails, preserve the logs and inspect them; do not force
unload. A failed SSH command is not proof that a kernel operation stopped.
Never read the whole regmap debugfs `registers` file; use the driver's bounded
`pmu_state` attribute.

## Current status and remaining implementation

On boot `3e212fde-60e7-4649-9e77-75ac2ada202c`, ACPM FVP attachment, the WLBT
preparation request, firmware staging, and memory preparation all succeeded.
The TZASC call returned zero; PMU BAAW registers read back as size `0x400` and
base `0xe9000`. The mxconf v0.1 shared-memory layout was written and verified
at `0x1d4248`, using 424 allocator blocks. The module was unloaded cleanly
afterward. `ip link` still lists only `lo` and `usb0`.

The next implementation must provide the Exynos7885 MIF transport, safe
firmware boot/stop lifecycle and the SCSC management/HIP WLAN stack that
registers a real `cfg80211` interface. The Samsung reference cannot be dropped
into Linux 6.15 unchanged: `platform_mif.c` uses removed `exynos_smc()` and
`devm_ioremap_nocache()` APIs and needs a current shared-memory mapping. Port
the required interfaces in mainline style. Do not restore the abandoned
`gta3xl-scsc` probe machinery or copy Samsung-only Android dependencies.

The saved r15 baseline already has `CONFIG_CFG80211=m` and `CONFIG_MAC80211=m`;
its `Module.symvers` exports `wiphy_register()`, `wiphy_unregister()` and
`cfg80211_register_netdevice()`. This permits developing the host WLAN stack as
external modules without changing kernel configuration or rebuilding the
recovery image. The current mainline checkout has no SCSC/HIP WLAN subtree, so
the driver itself still has to be ported before a real interface can appear.

Iteration 038 adds the SCSC MIF interrupt-bit allocator/dispatcher as a
software component in the composite module. It reserves from-host bit 0 for
R4/M4 panic signaling and supports callback allocation, pending-bit dispatch
and acknowledgment. Iteration 039 adds Exynos7885 register callbacks using the
downstream-verified INTMSR0/INTMR0/INTCR0/INTGR1 layout. The production manager
remains inactive: no callback touches MMIO until a future explicit lifecycle
enables it. The self-test now verifies inactive calls fail without invoking
callbacks, then exercises allocations and dispatch through fake registers.
Iteration 039 built against the r15 baseline, loaded, reported `state=inert`,
passed `mif_intr_selftest`, and unloaded on boot
`3e212fde-60e7-4649-9e77-75ac2ada202c`. The module is unloaded. No IRQ is
requested or dispatched, and no radio event, firmware handoff, or hardware
mailbox operation has been tested.

Iteration 040 adds the generic Linux IRQ entry point and exercises its inactive
and handled return paths in the fake-register self-test. It built, loaded, and
passed the self-test on the same boot, then unloaded. The real `mbox` IRQ stays
unrequested: it must only be armed as part of a reviewed startup sequence after
WLBT is powered, otherwise an asserted level interrupt could not safely be
acknowledged. No hardware IRQ was generated or tested.

Iteration 041 looks up the DT resource named `mbox` without requesting it or
accessing mailbox registers. On boot `3e212fde-60e7-4649-9e77-75ac2ada202c`,
the kernel resolved it to Linux IRQ 55 (the DT specifier is GIC SPI 57). The
module remained `inert`, the fake MIF self-test passed, and it unloaded; no
hardware IRQ was requested or triggered.

Iteration 034 now retains bounds-checked pointers to the eight R4/M4 mailbox
slots from the DT resources. It validates and maps the ranges without MMIO
access; IRQ delivery, interrupt-bit operations and mailbox handoff are still
not implemented.

Iteration 035 reads three additional PMU registers, without writes: CP state
(`0x0038`), shared-regulator status (`0x3644`) and option (`0x3648`). The
observed values were `0x1`, `0x20001` and `0x6`; the CP state differs from the
downstream release helper's expected `0x10`. The required CP mailbox ISSR2
request/ack is not yet supported by this driver or the mainline T510 DTS, so
firmware release remains gated.

The opt-in module can map the downstream-confirmed CP mailbox window at
`0x12080000` when a future T510 DT supplies it, without touching the mapped
registers. Its `shared_rail_state` attribute reads only fixed PMU status values
and reports whether the mapping and downstream reference values match. The CP
status comparison is diagnostic in Samsung's helper, not a hard startup gate.
This read-only attribute is not a shared-rail handshake and does not enable
firmware start; the CP ISSR protocol and an explicit recoverable lifecycle are
still required. The installed DT lacks this resource, so the current module
reports it as missing.

The downstream boot handoff writes the firmware entry to MBOX0, the verified
R4-relative config offset to MBOX1, `0xbcdeedcb` to MBOX2, and the startup flags
to MBOX3 before reset release. The values are understood, but this module does
not access those registers. The board's shared-regulator and CP-mailbox release
dependencies plus a recoverable stop path are still unresolved; keep firmware
release disabled until they are mapped and reviewed.

Before any processor release, establish the board-specific power/reset
sequence, ACPM voltage handshake, IRQ handling and firmware configuration
handoff. The tablet has been soft-bricked by earlier startup experiments. Keep
all startup opt-in, avoid blocking probe paths, require bounded handshakes and
do not run firmware-patching or halt-stub probes. A new recovery image is only
needed for kernel, config, exported-interface or DT changes; module-only
iterations on this baseline do not need a reboot.

See [POWER-PREREQUISITES.md](POWER-PREREQUISITES.md) for the measurements and
source audit, and `wifi-iterations/027-stage-firmware/README.md` plus
`wifi-iterations/030-prepare-memory/README.md` for device results.
Iteration 044 was built against r17 and checked on boot
`8c3f4049-8e46-4692-83b3-45a086006dd0`. The driver mapped `cp` and reported
`state=inert`; the fake MIF test passed. Its bounded PMU read showed
`CP_STAT=0x1`, shared status `0x20001`, and shared option `0x6`. The module was
unloaded. The shared-option state already has bit 2 set, so do not infer that a
standalone acquire/release test would restore the previous state. Firmware
start remains disabled and the device has no WLAN interface.
Iteration 045 added the disabled MBOX IRQ reservation. It bound on the same
r17 boot in 211 us, logged `reserved inactive mailbox IRQ 55`, stayed inert,
and unloaded cleanly. It performed no mailbox MMIO; interfaces remained only
`lo` and `usb0`. See `wifi-iterations/045-r17-inactive-irq/` for the build and
device logs.
