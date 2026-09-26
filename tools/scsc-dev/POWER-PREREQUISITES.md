# Exynos7885 WiFi startup prerequisites

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
before the doorbell could be sent. The mailbox framework changes a freed
channel to `TXDONE_BY_POLL`; after a rebind, a client without
`knows_txdone = true` retained that method even though the controller has no
poll timer. The protocol now declares software completion, frees channels on
remove, and retains a bounded explicit test client. The Oops boot must be
rebooted before another query. No ACPM response or WiFi power-up has yet been
verified. The full log is in the workspace's
`wifi-iterations/012-acpm-channel0/device-dmesg-after-oops.txt`.

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
`platform_mif_reset()` calls `exynos_acpm_set_flag()`. That function sends four
words `[7, 1, 6, 0]` through the DVFS IPC channel, requesting a response. Its
comment identifies BUCK2 voltage preparation. The current mainline public ACPM
interface exposes PMIC operations, not this DVFS operation. Correct transport
and a checked response are needed before relying on that prerequisite.

The reference release path also temporarily enables the shared-regulator option
and coordinates with the CP mailbox. The matching WiFi-only tablet behavior
needs to be established before substituting an arbitrary PMU bit write.

DRAM access requires secure-monitor setup (`0x82000710`, subsystem 0, carveout
base and size) and PMU memory-window setup. The secure-call result must be checked;
do not copy the downstream behavior of proceeding after failure.

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

The next platform change must provide an Exynos7885-specific ACPM transport and
correct DT resources, with activation opt-in. Correcting the DT resources will
require a new boot image; the installed baseline cannot gain them by replacing
the WiFi module. Keep r14 pinned until that change is concrete and ready to test.
