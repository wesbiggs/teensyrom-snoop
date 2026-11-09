#!/usr/bin/env python3
"""Regenerate firmware/teensyrom-snoop.patch from a pristine TeensyROM checkout.

  tools/make_tr_patch.py /path/to/TeensyROM [--out firmware/teensyrom-snoop.patch]

Works on `git archive HEAD` of that checkout (its own uncommitted edits are ignored), copies
core/ to Source/Teensy/snoop/ and firmware/WriteSnoop.{h,ino} to Source/Teensy/, and makes six
small edits that each assert on upstream's text:

  TeensyROM.h      #define FeatWriteSnoop (main image only; MinimalBoot never sees it)
  Teensy.ino       #include "WriteSnoop.h" before ISRs.c, SnoopService() at the top of loop()
  ISRs.c           the tap, after the Kernal Replace snoop and before the PLA decode wait; and the
                   late-interrupt count at the end of the "late" branch (teensyrom-vdrive edits its top)
  SerUSBIO.ino     token 0x6453 in ProcessCommand's always-available switch
  Flash/FXUtil.cpp the menu's firmware update masks IRQ_USBHS too before flash_move (from sn10): with an
                   audio dongle in, that interrupt runs flash code every 1 ms and faulted the copy

and, for the USB audio dongle on the host port (phase 2 step 3), copies firmware/usbhost/ to
Source/Teensy/src/USBHost_t36/ (USBAudioOut, isochronous.h, isoaudio.{h,c}) and edits the vendored library:

  ehci.cpp         include isochronous.h; the siTD refill in isr(); add_qh_to_periodic_schedule steps past
                   siTDs (its TODO); the isochronous OUT engine (firmware/usbhost/ehci_iso.inc) at the end
  README.md        the local modifications listed

The hunks are kept away from the lines teensyrom-vdrive's patch changes, so both apply in either order.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T = "Source/Teensy"


def edit(tree, rel, old, new, count=1):
    p = os.path.join(tree, rel)
    with open(p, newline="") as fh:
        s = fh.read()
    crlf = "\r\n" in s
    if crlf:
        old, new = old.replace("\n", "\r\n"), new.replace("\n", "\r\n")
    n = s.count(old)
    if n != count:
        sys.exit(f"{rel}: expected {count} match(es) of:\n{old}\nfound {n} (upstream changed? update tools/make_tr_patch.py)")
    with open(p, "w", newline="") as fh:
        fh.write(s.replace(old, new))


UH = f"{T}/src/USBHost_t36"
USBHOST_FILES = ("USBAudioOut.h", "USBAudioOut.cpp", "isochronous.h", "isoaudio.h", "isoaudio.c")


def usbhost_edits(tree):
    """Isochronous OUT in the vendored USBHost_t36 (pinned at f943c4b; teensyrom-vdrive does not touch it)."""
    ehci = f"{UH}/ehci.cpp"
    edit(tree, ehci,
         '#include "USBHost_t36.h" // Read this header first for key info\n',
         '#include "USBHost_t36.h" // Read this header first for key info\n'
         '#include "isochronous.h" // teensyrom-snoop: isochronous OUT through siTDs, at the end of this file\n')
    edit(tree, ehci,
         "\tif (stat & USBHS_USBSTS_UAI) { // completed qTD(s) from the async schedule\n",
         "\tif (stat & (USBHS_USBSTS_UPI | USB_USBSTS_UI)) {\n"
         "\t\tusbhost_iso_out_service(); // teensyrom-snoop: refill the siTDs first, the deadline is a frame away\n"
         "\t}\n"
         "\tif (stat & USBHS_USBSTS_UAI) { // completed qTD(s) from the async schedule\n")
    edit(tree, ehci,
         "\t\tuint32_t num = periodictable[i];\n"
         "\t\tPipe_t *node = (Pipe_t *)(num & 0xFFFFFFE0);\n"
         "\t\tif ((num & 1) || ((num & 6) == 2 && node->periodic_interval < interval)) {\n"
         "\t\t\t//println(\"  add to slot \", i);\n"
         "\t\t\tpipe->qh.horizontal_link = num;\n"
         "\t\t\tperiodictable[i] = (uint32_t)&(pipe->qh) | 2; // 2=QH\n"
         "\t\t} else {\n"
         "\t\t\t//println(\"  traverse list \", i);\n"
         "\t\t\t// TODO: skip past iTD, siTD when/if we support isochronous\n",
         "\t\t// teensyrom-snoop: iTDs and siTDs come first in a slot; the QH tree starts after them.\n"
         "\t\t// Every descriptor type keeps its next link in word 0.\n"
         "\t\tvolatile uint32_t *head = &periodictable[i];\n"
         "\t\twhile (!(*head & 1) && (*head & 6) != 2) head = (volatile uint32_t *)(*head & 0xFFFFFFE0);\n"
         "\t\tuint32_t num = *head;\n"
         "\t\tPipe_t *node = (Pipe_t *)(num & 0xFFFFFFE0);\n"
         "\t\tif ((num & 1) || ((num & 6) == 2 && node->periodic_interval < interval)) {\n"
         "\t\t\t//println(\"  add to slot \", i);\n"
         "\t\t\tpipe->qh.horizontal_link = num;\n"
         "\t\t\t*head = (uint32_t)&(pipe->qh) | 2; // 2=QH\n"
         "\t\t} else {\n"
         "\t\t\t//println(\"  traverse list \", i);\n")
    with open(os.path.join(ROOT, "firmware", "usbhost", "ehci_iso.inc")) as fh:
        engine = fh.read()
    edit(tree, ehci,
         '\tfree_Pipe(pipe);\n\tprintln("* Delete Pipe completed");\n}\n',
         '\tfree_Pipe(pipe);\n\tprintln("* Delete Pipe completed");\n}\n' + engine)
    edit(tree, f"{UH}/README.md",
         "\n## License\n",
         "- teensyrom-snoop's patch (its phase 2, a USB audio dongle on the host port) adds `USBAudioOut.{h,cpp}`, "
         "`isochronous.h` and `isoaudio.{h,c}`, and in `ehci.cpp` the isochronous OUT engine (siTDs, at the end), its "
         "refill in `isr()`, and `add_qh_to_periodic_schedule` stepping past siTDs to the QH tree.\n"
         "\n## License\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("teensyrom")
    ap.add_argument("--out", default=os.path.join(ROOT, "firmware", "teensyrom-snoop.patch"))
    a = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="trsnoop-")
    try:
        tree = os.path.join(tmp, "tr")
        os.mkdir(tree)
        arch = subprocess.run(["git", "-C", a.teensyrom, "archive", "HEAD"], check=True, capture_output=True).stdout
        subprocess.run(["tar", "-x", "-C", tree], input=arch, check=True)
        git = ["git", "-C", tree, "-c", "user.name=x", "-c", "user.email=x@x", "-c", "core.autocrlf=false"]
        subprocess.run(git + ["init", "-q"], check=True)
        subprocess.run(git + ["add", "-A"], check=True)
        subprocess.run(git + ["commit", "-qm", "base"], check=True)

        os.makedirs(os.path.join(tree, T, "snoop"))
        for f in ("snoop.c", "snoop.h"):
            shutil.copy(os.path.join(ROOT, "core", f), os.path.join(tree, T, "snoop", f))
        for f in ("WriteSnoop.h", "WriteSnoop.ino"):
            shutil.copy(os.path.join(ROOT, "firmware", f), os.path.join(tree, T, f))
        for f in USBHOST_FILES:
            shutil.copy(os.path.join(ROOT, "firmware", "usbhost", f), os.path.join(tree, UH, f))

        edit(tree, f"{T}/TeensyROM.h",
             "   #define FeatTCPListen //Enable TCP Listen capability for remote commands\n",
             "   #define FeatTCPListen //Enable TCP Listen capability for remote commands\n"
             "   #define FeatWriteSnoop //Stream C64 writes to chosen addresses (teensyrom-snoop, token 0x6453)\n")

        edit(tree, f"{T}/Teensy.ino",
             '#include "MinimalBoot/Common/ISRs.c"\n',
             '#include "WriteSnoop.h"  //teensyrom-snoop: the ISR tap, needs Common_Defs.h and tcpClient above\n'
             '#include "MinimalBoot/Common/ISRs.c"\n')
        edit(tree, f"{T}/Teensy.ino",
             "void loop()\n{\n",
             "void loop()\n{\n"
             "#ifdef FeatWriteSnoop\n"
             "   SnoopService(); //teensyrom-snoop: drain the capture ring, send frames\n"
             "#endif\n")

        edit(tree, f"{T}/MinimalBoot/Common/ISRs.c",
             "#ifdef Fab04_GlobalKernalReplace\n"
             "   if (fKernRepl != NULL) if (fKernRepl(Address, R_Wn)) return;\n"
             "#endif\n",
             "#ifdef Fab04_GlobalKernalReplace\n"
             "   if (fKernRepl != NULL) if (fKernRepl(Address, R_Wn)) return;\n"
             "#endif\n"
             "\n"
             "#ifdef FeatWriteSnoop\n"
             "   //teensyrom-snoop: a write to a watched page; SnoopTap waits for the data (nS_DataSetup)\n"
             "   //  only when the address is watched.  After the snoops above, so their timing is unchanged.\n"
             "   if (!R_Wn && SnoopHot.page[Address >> 8]) SnoopTap(Address);\n"
             "#endif\n")

        edit(tree, f"{T}/MinimalBoot/Common/ISRs.c",
             "            if (BigBufCount < BigBufSize-1) BigBufCount++; //last entry is scratch, rewritten once full\n"
             "         }\n"
             "      #endif\n"
             "   }\n",
             "            if (BigBufCount < BigBufSize-1) BigBufCount++; //last entry is scratch, rewritten once full\n"
             "         }\n"
             "      #endif\n"
             "      #ifdef FeatWriteSnoop\n"
             "         snoop_isr_late(&SnoopHot, CycSinceLast); //teensyrom-snoop: count late interrupts (only on this late path)\n"
             "      #endif\n"
             "   }\n")

        edit(tree, f"{T}/SerUSBIO.ino",
             "   #endif\n"
             "      case ResetC64Token:   //Reset C64\n",
             "   #endif\n"
             "   #ifdef FeatWriteSnoop\n"
             "      case SnoopToken: //teensyrom-snoop: start/stop/status of the write capture (works while a program runs)\n"
             "         SnoopCommand();\n"
             "         return;\n"
             "   #endif\n"
             "      case ResetC64Token:   //Reset C64\n")

        edit(tree, f"{T}/Flash/FXUtil.cpp",
             "  NVIC_DISABLE_IRQ(IRQ_PIT);\n",
             "  NVIC_DISABLE_IRQ(IRQ_PIT);\n"
             "#ifdef FeatWriteSnoop\n"
             "  //teensyrom-snoop: with a USB audio dongle in, the USB host interrupt runs code in flash every 1 ms\n"
             "  //  (the sample fill), which faults once flash_move erases it.  The new image is in the buffer by now.\n"
             "  NVIC_DISABLE_IRQ(IRQ_USBHS);\n"
             "#endif\n")

        usbhost_edits(tree)

        subprocess.run(git + ["add", "-A"], check=True)
        diff = subprocess.run(git + ["diff", "--cached", "--no-color"], check=True, capture_output=True).stdout
        base = subprocess.run(["git", "-C", a.teensyrom, "rev-parse", "--short=8", "HEAD"],
                              check=True, capture_output=True, text=True).stdout.strip()
        with open(a.out, "wb") as fh:
            fh.write(f"teensyrom-snoop patch for TeensyROM, made on upstream {base} by tools/make_tr_patch.py\n\n".encode())
            fh.write(diff)
        print(f"wrote {a.out} ({len(diff)} bytes, upstream {base})")
    finally:
        shutil.rmtree(tmp)


if __name__ == "__main__":
    main()
