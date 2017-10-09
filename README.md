Oculus Rift Development Kit 2
=======

Firmware, electrical CAD, mechanical CAD, artwork, and documentation for DK2.

License
-------
The CAD, artwork, and documentation are released under Creative Commons
Attribution 4.0 International.

The Oculus firmware is licensed under the BSD-style license at
https://github.com/fbsamples/RiftDK2/blob/master/Headset/Firmware/LICENSE

We also provide for the Oculus firmware an additional grant of patent rights at
https://github.com/fbsamples/RiftDK2/blob/master/Headset/Firmware/PATENTS

The firmware incorporates or references materials of third parties that are
subject to other licenses.  Please see the individual headers on each file to
determine which license applies.  For ease of reference, headers that include
the link http://www.st.com/software_license_agreement_liberty_v2 refer to the
license reproduced in en.ultimate-liberty-v2.txt.

Documentation
-------------

High-level specifications for the DK2 headset, sensor, and firmware.

Cable
-----

The cable is surprisingly one of the most complex parts of the product, and
is likely actually more advanced than the entirety of DK1.  This directory
contains schematics and high level specifications for the cable, but this
is a custom assembly that likely isn't achievable to recreate from source.

Sensor
------

The positional tracking sensor release contains electrical and mechanical
CAD.  The sensor also utilizes microcontrollers which require firmware
which was not redistributable.

Headset
-------

The headset directory contains the mainboard firmware as well as electrical
and mechanical CAD for the headset.  It also contains artwork for the
packaging.

Contributors
------------

The incredible (and incredibly small) team who made DK2:

 * Headset Electrical: Lyle Bainbridge, Ryan Brown, Matt DeVoe
 * Sensor Electrical: John Robertson
 * Headset Firmware: Nirav Patel
 * Mechanical: Julian Hammerstein, Matt Thomas, Kam Chin
 * Optical: Youngshik Yoon
 * Automation: Simon Hallam, Anusha Balan, Brant Lewis
 * Operations: Rita Chen, Licia Angino, Jack McCauley, David Dykes
 * Factory: Allen Qin, Victor Zhang, Ivan Chow, Mark Zhang
 * Misc: Palmer Luckey, Chris Dycus, Jonathan Shine
 * Artwork: Matt Ford, Jon Malkemus

RiftDK2 repository preparation: Matt Appleby, Kelly Lowenberg, John Tsai
