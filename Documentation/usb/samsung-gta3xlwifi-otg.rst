.. SPDX-License-Identifier: GPL-2.0-only

======================================
Samsung SM-T510 USB-C bring-up and tests
======================================

Hardware and interfaces
=======================

The board uses DWC3/xHCI, S2MM005 at I2C6 address 0x33 (gpa0-2 interrupt),
and S2MU005 at I2C1 address 0x3d (gpa2-2 interrupt). The S2MM005 driver uses
resident firmware; it does not flash firmware. The S2MU005 children provide
an open/USB mux and a charger with a 5.1 V OTG regulator. Data and power
roles are independent: a powered dock can have host data and sink power.

The battery thermistor is ADC channel 1 at 0x120a0000. The board's Samsung
NTC table gives raw cutoffs 1052/3166 (50/0 C), with recovery at 1220/2964
(45/5 C). The charger fails closed on unavailable/invalid temperature,
input faults, or unavailable sink power. Charge voltage/current/termination
come from monitored-battery. The initial board limit is 4.35 V and 1 A;
input current is bounded by the advertised/negotiated grant and 1.5 A.
The charger pauses charging during system suspend, then rechecks the
thermistor and contract on resume. OTG source power can remain on while
suspended; the Type-C interrupt wakes the system on cable changes.

The S2MU005 fuel gauge is a separate I2C0 device at 0x3b. Its current_now is
positive while charging and negative while discharging. Its temperature
channel measures the gauge, not the battery NTC; charging never uses it.

The userspace interfaces are the standard /sys/class/typec,
/sys/class/usb_role, /sys/class/regulator and /sys/class/power_supply trees.
Relevant supplies are s2mm005-usb, s2mu005-charger and s2mu005-fg.
No manual DWC3 debugfs role writes or raw I2C register writes are needed.

Before flashing
===============

Save the current working image and collect a baseline on the tablet::

    uname -a
    dmesg > usb-baseline-dmesg.txt
    lsusb -t
    ip address

Confirm USB networking and SSH to a PC work. Keep the recovery/Download
Mode flashing route available: OTG replaces that same physical USB link,
so logs during host operation must be captured locally on the tablet.

Acceptance tests
================

1. Boot with no cable, then connect a PC. Confirm device/sink role and
   USB networking/SSH. Repeat with the USB-C connector reversed.
2. Connect an OTG adapter with a keyboard/mouse. Confirm host/source role,
   a new xHCI bus, enumeration in lsusb -t, and input events with evtest.
3. Read a file from a USB flash drive and compare its checksum. Repeat
   disconnect/reconnect at least ten times, then reconnect the PC and SSH.
4. Cold boot with the adapter attached. Repeat suspend/resume with no
   cable, a PC, and the adapter, including disconnect during suspend.
5. With a powered USB-C dock, attach a keyboard and storage device. Confirm
   host/sink role, s2mm005-usb/voltage_max = 5000000, boost disabled, and
   s2mu005-fg/current_now positive under light load. Exercise charger-first
   and dock-first attachment, charger removal/reinsertion, both plug
   orientations, and return to PC networking. Suspend pauses charging;
   resume must restore it only after a valid temperature/contract read.
6. Confirm no interrupt storm, repeated I2C errors, thermal read timeouts,
   xHCI failures, or regulator failures in dmesg. Inspect::

       dmesg | grep -Ei 's2mm005|s2mu005|adc|dwc3|xhci|usb|regulator'
       cat /proc/interrupts
       cat /sys/class/typec/port*/data_role
       cat /sys/class/typec/port*/power_role
       cat /sys/class/power_supply/s2mm005-usb/uevent
       cat /sys/class/power_supply/s2mu005-charger/uevent
       cat /sys/class/power_supply/s2mu005-fg/uevent

Do not infer charging from a supply being online: check actual battery
current. A powered-dock pass requires a real dock; software tests alone
cannot verify it. Higher-voltage charging, alternate modes, and firmware
updates are outside this implementation.

Software checks
===============

TYPEC_S2MM005_KUNIT_TEST covers host/source, device/sink, host/sink,
device/source, power-swap ordering, detached/wet/shorted ports, malformed
PD contracts, and rejection of non-5 V contracts. Run with KUNIT, USB_SUPPORT,
TYPEC and TYPEC_S2MM005_KUNIT_TEST enabled using tools/testing/kunit/kunit.py.

Run checkpatch, validate the bindings and board DTB, and compile the full
board configuration. Hardware bring-up is still required after these pass.
