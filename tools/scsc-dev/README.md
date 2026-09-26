# T510 WiFi module iteration

The current `gta3xl` baseline is aport r15, pinned to kernel commit
`40f6ee3140196cccad7fd652e69e5159414abbea`. It boots as
`6.15.0-rc1-exynos7904-wifi-dev1`; its DT reserves 4 MiB at `0xe9000000`.
The recovery image has already been flashed. Keep this baseline and its saved
build archive while iterating on code that fits in the out-of-tree module.

The explicit `enable=1` module does not autoload. It can validate and stage the
installed `postmarketos/mx140/mx140.bin`, configure its reserved DRAM window
through the WLBT TZASC secure call and PMU BAAW registers, and read back each
operation. It does not power WLBT, alter reset, access the mailbox registers,
request IRQs, release either processor, or register a network interface.
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
`gta3xl-wifi-baseline-<commit>-r15.tar.gz`. Keep it with the matching image and
native compiler environment. The archive includes generated headers, symbol
tables, configuration, DT images and compiler identity. It remains on the host;
only a small manifest is installed on the tablet. Never mix modules from two
baseline archives with the same `uname -r`.

## Build module-only iterations

Select the matching r15 archive explicitly:

```sh
SCSC_BASELINE_DIR="$HOME/.cache/t510-wifi-dev/baseline-r15" \
    bash wifi-dev/build-module.sh 031
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
    --target root@172.16.42.1 --out wifi-iterations/031
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py load \
    --target root@172.16.42.1 --out wifi-iterations/031
ssh root@172.16.42.1 'd=/sys/bus/platform/devices/120c0000.wifibt; \
    printf 1 > "$d/stage_firmware"; printf 1 > "$d/prepare_memory"; \
    cat "$d/firmware_status"; cat "$d/pmu_state"'
python3 linux-exynos7885/tools/scsc-dev/scsc-dev.py unload \
    --target root@172.16.42.1 --out wifi-iterations/031
```

Run only the operation being tested. Firmware staging writes and verifies the
firmware in reserved DRAM. `prepare_memory` additionally configures the EL3
TZASC grant and PMU BAAW aperture after confirming PWRON, START and WIFI_STAT
show the block off. Neither operation starts firmware. Both return promptly.
The helper's `load`, `unload` and `cycle` commands are intended for this
non-running state only. If bind/unbind fails, preserve the logs and inspect
them; do not force unload. A failed SSH command is not proof that a kernel
operation stopped.

## Current status and remaining implementation

On boot `e5fc02de-0a93-4cec-8384-3d160b26580c`, firmware staging and memory
preparation both succeeded over SSH. The TZASC call returned zero; PMU BAAW
registers read back as size `0x400` and base `0xe9000`. The module was unloaded
cleanly afterward. `ip link` still lists only `lo` and `usb0`.

The next implementation must provide the Exynos7885 MIF transport, safe
firmware boot/stop lifecycle and the SCSC management/HIP WLAN stack that
registers a real `cfg80211` interface. The Samsung reference cannot be dropped
into Linux 6.15 unchanged: `platform_mif.c` uses removed `exynos_smc()` and
`devm_ioremap_nocache()` APIs and needs a current shared-memory mapping. Port
the required interfaces in mainline style. Do not restore the abandoned
`gta3xl-scsc` probe machinery or copy Samsung-only Android dependencies.

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
