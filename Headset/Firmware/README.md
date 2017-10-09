DK2 Firmware
============

Licenses
--------

The Oculus firmware is licensed under the BSD-style license at
https://github.com/fbsamples/RiftDK2/blob/master/Headset/Firmware/LICENSE

We also provide for the Oculus firmware an additional grant of patent rights at
https://github.com/fbsamples/RiftDK2/blob/master/Headset/Firmware/PATENTS

The firmware incorporates or references materials of third parties that are
subject to other licenses.  Please see the individual headers on each file to
determine which license applies.  For ease of reference, headers that include
the link http://www.st.com/software_license_agreement_liberty_v2 refer to the
license reproduced in en.ultimate-liberty-v2.txt.

External Libraries
------------------

The ARM CMSIS library is not included, but can be downloaded from:
https://github.com/ARM-software/CMSIS/tree/master/CMSIS

The firmware was originally developed with CMSIS 3.01

This should be placed in lib/CMSIS

Building DK2 Firmware
---------------------

Building DK2 firmware currently requires IAR EWARM.  You can get a 30-day trial
here: http://supp.iar.com/Download/SW/?item=EWARM-EVAL

Programming DK2 Firmware to a Board
-----------------------------------

### Updating an already programmed board

You can update a DK2 over USB using the Oculus Configuration Utility:
https://product-guides.oculus.com/en-us/documentation/dk2/latest/tasks/ug-tray-start-firmware/

In this case, you should use the most recent DK2.bin available for your board.

* Rev 3 (GDC units): prj/REV3/Exe/DK2.bin
* Rev 3.3 (Pilot run and production units): prj/REV3_3/Exe/DK2.bin

### Programming to a blank board

For a blank board, you'll need to program it over the SWD header using an
ST-Link programmer.

In this case, you'll need the ST-Link utility and DK2Bootloader.bin

* Rev 3 (GDC units): bootloader/prj/REV3/Exe/DK2Bootloader.bin
* Rev 3.3 (Pilot run and production units):
    bootloader/prj/REV3_3/Exe/DK2Bootloader.bin
