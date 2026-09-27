# Exynos7885 WiFi startup prerequisites

## Working downstream reference

Use postmarketOS branch `kipz/pmaports:kipz/t510-fixes` as the authoritative
packaging reference for the downstream kernel, rather than assuming the local
Samsung source dump includes postmarketOS changes. At pmaports commit
`041ac832896c`, its `linux-samsung-gta3xlwifi` aport pins
`starfoxdot64/samsung_kernel_gta3xlwifi` commit
`29d2fb67a8e80703c51764b9c0ddaad81ef44cd2` (Linux 4.4.177) and applies the
listed aport patches. The config builds SCSC core/platform, `mcu_ipc`, SCSC
WLAN, SCSC Bluetooth/BlueZ, and ACPM DVFS; firmware lookup is configured for
`/lib/firmware/postmarketos/mx140`. The aport's wireless-related patches
include the postmarketOS firmware-path change and two BlueZ fixes.

The working source's `platform_mif.c`, `mcu_ipc.c`, T510 downstream DTS and
Exynos7885 PMUCAL system table match the corresponding local Samsung source
files. `mxman.c`, WLAN `mgt.c`/Kconfig and other packaging-adjusted files do
differ. Thus the current hardware sequence findings are still grounded in the
working device's platform code, while firmware lookup and Bluetooth build
details must use the patched pmaports tree. The complete 4.4 SCSC/HIP WLAN and
Bluetooth stacks are not present in the current mainline tree; the r17 module
is only a platform/resource prototype and does not register a WLAN interface.

## Current r17 state (2026-09-26)

After flashing r17, boot `8c3f4049-8e46-4692-83b3-45a086006dd0` runs kernel
commit `78e1f99c4291b3861151c4f900d967d56e60543f` (`#18`). The live DT has
`r4`, `m4` and `cp` mailbox resources. Module iteration 044 mapped the CP
window, passed the fake MIF IRQ self-test, and unloaded cleanly; firmware start
remains disabled and only `lo`/`usb0` exist.

Bounded PMU reads on this boot returned `CP_STAT=0x1`,
`EXT_REGULATOR_SHARED_STATUS=0x20001`, and
`EXT_REGULATOR_SHARED_OPTION=0x6`. WiFi START/PWRON remain clear and the
central sequencer reports `0`. The option's bit 2 was already set before any
driver operation. Therefore a test that sets and clears only that bit would
not restore the observed initial state, and is not a safe substitute for the
downstream CP-coordinated, reference-counted shared-rail operation. No CP
mailbox register has been accessed and no rail, reset, power, or firmware
operation has been attempted on this boot.

The only exposed writes in the current module validate firmware and prepare
reserved-memory/TZASC/BAAW/configuration. There is no start or reset attribute;
probe only maps resources. Keep it that way until the ACPM request's device
behavior, CP mailbox handshake/IRQ semantics, shared-rail ownership, and a
bounded recoverable stop path are all established. The prior r15 ACPM request
did not receive an acknowledgment and was followed by ambiguous APM fault-log
entries; module load/unload without requests was inert. Do not issue another
request or write CP/PMU state as part of this diagnostic iteration.

On later r17 boot `264ec56b-18ec-424b-abed-bb2aafea0cb6`, the packaged
mailbox and ACPM protocol modules were loaded explicitly. The protocol
registered eight polling queues. The stock Exynos7885 FVP image
`exynos7885_acpm_fvp.fw` (SHA-256
`3793478b6df52fe760a89cbcfb258a9c489365ee2d13d9dae8df2f161abadb61`) was
staged at `/lib/firmware`, and the existing `attach_fvp` control completed
on channel 4. The previously validated WLBT flag operation `[7, 1, 6, 0]` then
returned success exactly once. This confirms the APM/plugin and
voltage-preparation prerequisites on r17; it does not test WiFi reset,
shared-rail sequencing, or firmware execution. The temporary ACPM diagnostic
module was unloaded, while the mailbox/protocol modules remain loaded for this
boot. No CP mailbox or WiFi PMU registers were read or written during these
checks. SSH remained usable, and the tablet still exposes only `lo` and `usb0`.

Iteration 045 built against the frozen r17 archive, reserved MBOX IRQ 55 using
`IRQF_NO_AUTOEN`, and bound/unbound cleanly on the same boot. Its MIF dispatcher
remained inactive, so the probe made no mailbox MMIO accesses. Only `lo` and
`usb0` remained. This reserves the Linux IRQ action for a future opt-in start
path; it does not validate an interrupt from powered WLBT or implement the
SCSC host transport. Logs are under
`wifi-iterations/045-r17-inactive-irq/`.

Iteration 047 routes the previously verified ACPM WLBT flag operation through
the SCSC module's root-only `prepare_voltage` control. ACPM is resolved only
when explicitly requested; inert probe remains independent of ACPM. The
current boot returned `attempted=1 prepared=1`, and a second write was rejected
before transmission. This remains a standalone prerequisite check: WiFi
firmware was not executed and no WiFi power/reset or CP mailbox registers were
touched. The temporary SCSC module unloaded cleanly; only `lo` and `usb0`
remain. Artifacts are in `wifi-iterations/047-r17-acpm-voltage/`.

Iteration 049 adds the reference-counted shared-rail transaction helper. Its
production callbacks encode CP ISSR2 wakeup, PMU option bit 2, CP ISSR3 ready
sampling, and the reverse release order, with rollback on failures. The only
exposed operation is a fake-callback self-test; no live callback is reachable
from sysfs. Both shared-rail and MIF self-tests passed on r17, and the module
unloaded cleanly. This validates software ordering/refcount/rollback only; the
current boot's `EXT_REGULATOR_SHARED_OPTION=0x6` and CP mailbox state remain
unresolved. No CP or WiFi PMU write was made. Artifacts are in
`wifi-iterations/049-r17-shared-rail/`.

Iteration 050 extended the helper to read and restore the preexisting shared
option bit. This matters because Samsung's `pmucal_lpm_init` writes bit 0 and
clears bit 2 in `EXT_REGULATOR_SHARED_OPTION`, while the mainline image does
not run that Samsung PMUCAL init sequence and currently reads `0x6`. The
helper now leaves an initially-set bit set when its last reference is released;
it only clears a bit that it observed clear before acquiring the rail. The
module compiled against the frozen r17 baseline, passed `shared_rail_selftest`
and `mif_intr_selftest` on the current boot, and unloaded cleanly. This is
software-only validation: the production CP mailbox and PMU callbacks were
not invoked, and the source of the early-boot bit state remains unverified.
Artifacts and logs are in `wifi-iterations/050-r17-rail-preserve/`.

## Safe diagnostic and latest reboot (2026-09-26)

An attempted wildcard read of the complete regmap debugfs `registers` file
caused a synchronous external abort in the diagnostic `cat` process at
`regmap_mmio_read32le`; the shell exited and the kernel logged that the task
exited with IRQs disabled. SSH remained available. The inert SCSC module was
unloaded cleanly. Do not repeat that read; use the driver's bounded
`pmu_state` attribute. After the user rebooted, boot ID changed to
`3e212fde-60e7-4649-9e77-75ac2ada202c`; only `lo` and `usb0` were present and
the old fault was gone. The read-only ACPM stale-state check showed channel-0
TX `0/0`, clear AP-to-APM doorbell, and `stale_channel0: false`. The packaged
`acpm_protocol` module rejected descriptor 3, so it was unloaded and replaced
with the already-tested r15 module from iteration 026. That module registered
all eight queues; FVP plugin 3 attached on channel 4, and one WLBT preparation
request completed with channel-0 TX/RX `1/1`.

Iteration 033 then staged and verified the 1,360,017-byte image, prepared the
TZASC/BAAW aperture, and wrote/read back the mxconf v0.1 configuration at
`0x1d4248` using 424 allocator blocks. SCSC remained `inert`; no WiFi power,
reset, mailbox or IRQ access occurred. SCSC and ACPM test/protocol modules were
unloaded cleanly. The device still has only `lo` and `usb0`. Do not start WLBT
firmware as part of the config test; transport, reset and power prerequisites
remain unimplemented.

Iteration 034 retained pointers to all eight R4/M4 ISSR mailbox slots after
validating each mapped `0x180`-byte resource covers offsets `0x80..0x9c`. The
single opt-in bind logged all slots mapped in 79 us and unloaded cleanly. The
driver did not read or write those registers or request IRQs; WLBT stayed off.

Iteration 035 added only three targeted always-on PMU reads for the downstream
shared-regulator path. On this boot they returned `CP_STAT=0x1`,
`EXT_REGULATOR_SHARED_STATUS=0x20001`, and
`EXT_REGULATOR_SHARED_OPTION=0x6`; the existing WiFi power/reset readings were
unchanged. The downstream helper reports expected `CP_STAT=0x10` and shared
status `0x20001`. It also updates ISSR2 in the CP mailbox before setting the
shared regulator option, but the CP mailbox is not mapped in the mainline T510
DTS and has not been accessed here. Treat this mismatch as unresolved; do not
release WLBT based only on the apparently matching regulator status.

Iteration 036 rebuilt the opt-in module against r15 and added a read-only
shared-rail status attribute. On the same boot it reported `cp_mailbox=missing`,
`cp_status=0x1`, `shared_status=0x20001`, and `pmu_ready=0`; the driver returned
to `inert` and unloaded cleanly. This confirms both the absent DT resource and
the CP-state mismatch without touching the CP mailbox or writing PMU state.
The downstream helper only logs the CP-state mismatch; it does not abort. The
shared-rail operation and firmware start remain unimplemented. Device logs and
module hash are in `wifi-iterations/036-cp-mailbox-gate/`.

Iteration 038 added the software MIF interrupt-bit manager. Its device-side
self-test passed on the same boot with fake register callbacks; the driver
remained inert and unloaded. This verifies software allocation/dispatch only,
not the hardware MIF IRQ path. No mailbox or PMU writes occurred; see
`wifi-iterations/038-mif-intr/`.

## r15 follow-up, same boot as firmware staging (2026-09-26)

Boot ID `e5fc02de-0a93-4cec-8384-3d160b26580c` stayed healthy throughout
module-only work. The opt-in ACPM FVP plugin attach on channel 4 succeeded;
the WLBT ACPM flag write returned success, and one bounded MIF-rate request
completed with response value `0`. Queue indices remained synchronized. These
observations supersede the earlier blanket statement that no ACPM request
received a response. Source inspection confirms `set_wlbt_flag` sends the exact
four-word BUCK2 preparation request `[7, 1, 6, 0]` on channel 0 and waits for
the response; its successful return validates that prerequisite on this boot.

The staging module's root-only `pmu_state` read on this boot returned:

```
0140: 00100ea0
0144: 00000000
0148: 00000000
0384: 00000000
7300: 00000400
7304: 00060000
```

The module was unloaded afterward. No PMU writes, IRQ requests, reset,
processor release, or radio power-up occurred. `ip link` still showed only
`lo` and `usb0`. The firmware copy was separately verified in
`wifi-iterations/027-stage-firmware/README.md`.

The opt-in memory preparation operation then succeeded on the same boot:
the EL3 TZASC call returned zero, and PMU aperture registers read back as
size `0x400` (4 MiB) and base `0xe9000` (physical `0xe9000000`). The driver
rechecked WiFi PWRON, START and status before changing them and confirmed the
block stayed off afterward. It was unloaded cleanly. This validates the DRAM
access setup only; processor release, radio startup and `cfg80211` registration
remain untested. The operation and build are in `wifi-iterations/030-prepare-memory/`.

## r15 module-only ACPM findings (2026-09-26)

The r15 image has the Exynos7885 SRAM and mailbox DT resources. Both ACPM
modules remain explicitly loaded. The firmware reports eight channel
descriptors: some are notification buffers (`type=2`), and some queues are
interrupt-driven. A queue-only driver must skip those until it has an IRQ
handler. The downstream DVFS request channel is **0**, inherited from the
clock driver's `acpm-ipc-channel = <0>`; `acpm_dvfs` channel 5 receives MIF
notifications and is a buffer, not the request channel.

An early protocol revision rejected channel 3 because its buffer length is
one. A revised module bound after skipping unsupported descriptors. The first
read-only channel-0 request then hit a kernel Oops in `hrtimer_start_range_ns`
immediately after the doorbell write. The mailbox framework changes a freed
channel to `TXDONE_BY_POLL`; after a rebind, a client without
`knows_txdone = true` retained that method even though the controller has no
poll timer. The protocol now declares software completion and frees channels
on removal. The Oops boot was rebooted. On the new boot, modules loaded and
unloaded cleanly, but a read-only channel-0 rate query timed out. BusyBox
retried the sysfs read, leaving AP TX front/rear `2/0` and RX `0/0`. The
APM-to-AP mailbox status was `0`; AP-to-APM status was `0x00010000`, meaning
the channel-0 doorbell remained pending and unmasked. The APM did not consume
either request. An initial PMU diagnostic mistakenly read cortex registers
at `0x11c80100/104`; the downstream node maps them at `0x11c88100/104`.
Correct values were cortex configuration `1`, cortex status `0x10000001`,
central configuration `0x10001`, central status `0`. The cause of the missing
APM response remains unknown. There are two stale requests
on this boot: do not issue another one. A module-only follow-up fails closed
after a timeout on Exynos7885, avoiding another BusyBox retry. It is a safety
fix, not an APM startup fix. No WiFi power-up has been attempted. The old Oops
log is in `wifi-iterations/012-acpm-channel0/device-dmesg-after-oops.txt`.

The r14 development baseline supports module iteration, but its existing ACPM
description is a GS101 placeholder. Do not turn on WiFi by merely fixing the
clock lookup or ignoring the missing voltage-management handshake.

## Reference audit

The Samsung reference is the local `samsung_android_kernel_T510` tree, not the
abandoned `gta3xl-scsc` driver. No code from that branch has been restored.

| Item | Installed baseline | Exynos7885 downstream reference |
| --- | --- | --- |
| Mailbox compatible | `google,gs101-mbox` | Exynos7885 register layout |
| Clock lookup | CMU_TOP ID 63; provider has IDs 0–62 and no such gate | ACPM IPC driver does not request this CMU_TOP clock |
| AP-to-APM doorbell | GS101 writes bit ID to offset `0x40` | `apm_interrupt_gen()` writes bit ID+16 to offset `0x08` |
| Incoming polling mask | GS101 writes offset `0x28` | `channel_init()` writes offset `0x24`; `0x28` is status |
| Incoming acknowledgment | No Exynos7885 acknowledgment sequence | `INTCR1` at `0x20`; downstream also handles reassertion races |
| APM shared SRAM | `0x14b00000`, size `0x28000` | `0x02052000`, size `0x1b000` |
| Firmware table offset | GS101 `0xa000` | Exynos7885 `0x4b80` |
| IRQ specifier | Four cells passed to a three-cell GIC | SPI 22; needs a valid three-cell mainline specifier |

Relevant mainline files: `arch/arm64/boot/dts/exynos/exynos7885.dtsi`,
`drivers/clk/samsung/clk-exynos7885.c`, `drivers/mailbox/exynos-mailbox.c`,
and `drivers/firmware/samsung/exynos-acpm.c`.

Relevant reference files: `arch/arm64/boot/dts/exynos/dtbo/exynos7885.dts`,
`drivers/soc/samsung/acpm/acpm_ipc.{c,h}` and
`drivers/soc/samsung/acpm/fw_header/framework.h`.

The shared channel descriptor layout appears reusable: the channel array offset
is at +8, AP channel count at +24, and each 72-byte channel descriptor has the
same queue offsets. This is a source comparison, not a hardware validation.
A port must validate all table offsets, lengths and queue indexes against the
mapped SRAM before accessing them. Do not point the unmodified GS101 driver at
the Exynos7885 SRAM and let it probe automatically.

## Rail, security and reset requirements

The reference board enables `CONFIG_ACPM_DVFS`. Before WiFi reset release,
`platform_mif_reset()` calls `exynos_acpm_set_flag()`, which sends four words
`[7, 1, 6, 0]` through DVFS IPC and waits for a response. The r15 opt-in
`set_wlbt_flag` call completed successfully on the current boot, so the
voltage-preparation request itself is validated. The request still needs to be
ordered immediately before any future reset release.

The reference release path also temporarily enables the shared-regulator option
and coordinates with the CP mailbox. The matching WiFi-only tablet behavior
needs to be established before substituting an arbitrary PMU bit write.

The downstream implementation is specifically stateful, not a standalone
`EXT_REGULATOR_SHARED_OPTION` bit toggle. Under a lock, its first user writes
ISSR2 bit 0 to `1` at CP mailbox offset `0x88`, sets option bit 2 at PMU offset
`0x3648`, then reads ISSR3 bits 4:1 at offset `0x8c`. It returns a delay
indication when those bits are zero; the CP mailbox interrupt helper uses that
indication to wait 3 ms after generating a CP interrupt before releasing its
temporary shared-rail reference.
The last user clears option bit 2 and then clears ISSR2 bit 0. The downstream
helper logs when `EXT_REGULATOR_SHARED_STATUS != 0x20001` or `CP_STAT != 0x10`,
but these comparisons do not abort the operation. Our targeted reading saw the
rail status match but `CP_STAT == 1`. This is a reference-state discrepancy to
record, not by itself proof that shared-rail enable cannot work. The CP mailbox
node is at `0x12080000` in the downstream T510 DT. The installed mainline r17 DT
now maps it as the named `cp` resource; the SCSC module only maps it and
reserves its IRQ inactive. No CP mailbox register has been read or written.
Before using the mailbox, verify the separately gated block is accessible and
establish the CP state. Do not touch the mailbox through a hard-coded physical
address, and do not add an implicit rail operation to probe or module removal.

DRAM access requires secure-monitor setup (`0x82000710`, subsystem 0, carveout
base and size) and PMU memory-window setup. Both have now been performed and
read back successfully for the reserved 4 MiB region on r15. Keep checking the
secure-call result and aperture readback in future boots; do not copy the
downstream behavior of proceeding after failure.

Reset is a sequence through reset-ahead, bus cleanup, logic reset, TCXO gating,
isolation and the central sequencer, followed by assertion and bounded polling
for state `0x80`. Only after acknowledged quiescence may memory windows, IRQs and
work be released. Active firmware must hold a module reference so that failed
stop cannot lead to module unloading. No forced unloading, halt stubs or firmware
patches are part of this work.

## Measurements on the running r14 baseline

The explicit six-register PMU read returned:

```
0140: 00100ea0
0144: 00000000
0148: 00000000
0384: 00000000
7300: 00000400
7304: 00060000
```

PWRON and START are clear. The window is still the default 4 MiB at `0x60000000`,
not the board's `0xe9000000` reservation. Sequencer state is 0, so the reset
completion state `0x80` has not been demonstrated. These readings do not prove
a successful power or reset cycle. They were unchanged across firmware checks.

The installed 2019 firmware and the stock-dump 2021 firmware both pass all three
CRCs with header v1.0/API v0.2 and entry `0x1a9`. Their runtime lengths differ
(1,890,552 vs 1,895,976 bytes). Preserve matching firmware/config provenance;
neither image has been executed by this driver. The installed image is unchanged.

## Firmware handoff ABI (source audit)

The downstream `mxman_start()` initializes the management, R4 GDB, M4 GDB and
mxlog transports, then writes the boot handoff before calling
`mif->reset(mif, false)`: MBOX0 receives the firmware entry (`0x1a9` for the installed image),
MBOX1 receives the R4-relative mxconf offset (`0x1d4248` for iteration 033),
MBOX2 receives magic `0xbcdeedcb`, and MBOX3 receives
`firmware_startup_flags` (default zero). It issues a write barrier before
reset release. The config and entry values are now verified independently;
this driver has not written any mailbox register or released either core.

The downstream release path also invokes `exynos_pmu_shared_reg_enable()`,
then enables WIFI_PWRON, clears WIFI_RESET_SET, and asserts WIFI_START. Its
reset path changes reset-ahead, CLEANY bus, logic-reset, TCXO, isolation and
central-sequencer controls, waits up to 500 ms for state `0x80`, and may
separately power down or hold reset. The shared-regulator implementation and
CP mailbox coordination are Samsung-only and are not yet mapped to a safe
mainline equivalent. Do not implement or invoke this sequence until those
board-specific dependencies, IRQ behavior and a recoverable stop path are
resolved. Successful staging, TZASC, BAAW and mxconf readback alone do not
make firmware release safe.

The r15 image supplies the Exynos7885-specific ACPM transport resources. Its
protocol can bind as a module, but the first request received no response and
the APM did not service its queue or acknowledge the doorbell. A targeted read
of the APM SRAM log found `hard_fau` followed by a register dump ending at
`PC(R15)=0x7800`, then `nmi` and a second dump ending at `PC(R15)=0x1274`.
The log front stayed at 116 across two 0.2-second-spaced samples. The entries
cannot yet be tied to the current Linux boot or DVFS request. A new opt-in
protocol module refuses further transfers if its TX queue or AP-to-APM
doorbell is already pending; it is built as an opt-in module. The next step is to
determine why the APM faulted or stopped acknowledging its doorbell before
any WiFi power operation.

After a full tablet power cycle, boot `e98accc8-9d37-4d93-b2e1-3b9df2c46dde`
started with channel-0 TX `0/0`, outgoing doorbell clear, and log front 104;
the old fault records at indices 104–115 were absent. Explicitly loading the
mailbox and guarded protocol modules one at a time did not change any of those
values, and both unloaded cleanly. No ACPM request was made on this boot.
Two read-only attempts to inspect the plugin table ended with SSH exit 255,
although normal SSH and bounded state reads continued on the same boot; do not
repeat the inspection without understanding that failure. The downstream
kernel calls `plugins_init()` before clients; the mainline transport does not.
Whether this tablet needs a dynamic plugin attachment remains unknown.
The corrected diagnostic client was also loaded without reading its rate
attribute; the queue, doorbell and log front remained unchanged, and the
modules unloaded cleanly. The prior APM fault is temporally associated with
the first rate transaction or a later unrelated event, rather than with
these module probes alone. No second rate transaction has been attempted.
