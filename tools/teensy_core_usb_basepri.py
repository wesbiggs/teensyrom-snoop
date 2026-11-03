#!/usr/bin/env python3
"""Let the TeensyROM's bus interrupt run during USB transfers: an experiment on the Teensy core.

  tools/teensy_core_usb_basepri.py /path/to/cores/teensy4            # patch usb.c (keeps usb.c.orig)
  tools/teensy_core_usb_basepri.py /path/to/cores/teensy4 --revert   # put usb.c.orig back

On a Mac with Teensyduino 1.61 through arduino-cli, the core is
~/Library/Arduino15/packages/teensy/hardware/avr/1.61.0/cores/teensy4 . The TeensyROM's
build-firmware.mjs copies the installed core for each build, so patch it there, rebuild, test, and
revert when done (every other Teensy sketch on the machine uses the same core).

Why: every USB transmit goes through schedule_transfer() in usb.c, which runs with ALL interrupts
off (__disable_irq) while it polls USB registers (the ATDTW tripwire loop, up to 2400 CPU cycles).
The TeensyROM serves cartridge ROM reads from its phi2 interrupt (priority 16), which then answers
late: Ms. Pac-Man crashes as soon as the TeensyROM writes to USB serial, with stock firmware too
(the owner, 2026-11-03: a ping over USB 20 times a second, late interrupts >250 ns at once, crash).

The patch masks with BASEPRI instead: every interrupt of priority 32 or lower (numerically >= 32:
USB at 128, timers, Ethernet) waits, priorities 0 and 16 still run. The phi2 interrupt never calls
USB code, so the queue stays safe. The BASEPRI write itself is wrapped in cpsid/cpsie for a few
cycles (Cortex-M7 erratum 837070 on early revisions). The tripwire loop can now be interrupted by the
bus interrupt, so it gets a longer time-out (24000 cycles instead of 2400).
"""
import argparse
import os
import shutil
import sys

MARK = "teensyrom-snoop: BASEPRI"

OLD_ENTER = """	if (endpoint->callback_function) {
		transfer->status |= (1<<15);
	}
	__disable_irq();
"""
NEW_ENTER = """	if (endpoint->callback_function) {
		transfer->status |= (1<<15);
	}
	// teensyrom-snoop: BASEPRI instead of __disable_irq(), so priority 0-16 interrupts (the
	//   TeensyROM's phi2 bus interrupt) keep running; USB (128) and anything >= 32 waits
	uint32_t basepri_saved;
	__asm__ volatile("mrs %0, basepri" : "=r" (basepri_saved));
	__asm__ volatile("cpsid i\\n msr basepri, %0\\n cpsie i" :: "r" (32) : "memory");
"""
OLD_LOOP = "} while (!(USB1_USBCMD & USB_USBCMD_ATDTW) && (ARM_DWT_CYCCNT - cyccnt < 2400));"
NEW_LOOP = "} while (!(USB1_USBCMD & USB_USBCMD_ATDTW) && (ARM_DWT_CYCCNT - cyccnt < 24000));"
OLD_LEAVE = """	endpoint->last_transfer = transfer;
	__enable_irq();
"""
NEW_LEAVE = """	endpoint->last_transfer = transfer;
	__asm__ volatile("msr basepri, %0" :: "r" (basepri_saved) : "memory");
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("core", help="the cores/teensy4 directory")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    usb = os.path.join(a.core, "usb.c")
    orig = usb + ".orig"
    if a.revert:
        if not os.path.exists(orig):
            sys.exit(f"no {orig}: nothing to revert")
        shutil.copyfile(orig, usb)
        os.remove(orig)
        print(f"restored {usb}")
        return
    text = open(usb).read()
    if MARK in text:
        print(f"{usb} is already patched")
        return
    start = text.index("static void schedule_transfer(")
    end = text.index("\n}\n", start) + 3
    fn = text[start:end]
    for old in (OLD_ENTER, OLD_LOOP, OLD_LEAVE):
        if fn.count(old) != 1:
            sys.exit(f"schedule_transfer() in {usb} is not the code this script knows "
                     f"(looked for: {old.strip()[:60]}...); nothing changed")
    fn = fn.replace(OLD_ENTER, NEW_ENTER).replace(OLD_LOOP, NEW_LOOP).replace(OLD_LEAVE, NEW_LEAVE)
    shutil.copyfile(usb, orig)
    with open(usb, "w") as fh:
        fh.write(text[:start] + fn + text[end:])
    print(f"patched schedule_transfer() in {usb} (original in {orig})")


if __name__ == "__main__":
    main()
