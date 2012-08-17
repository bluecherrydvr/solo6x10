Bluecherry solo6x10 Video4Linux2 driver
======================================
This driver is released under the GPL for the line of Bluecherry MPEG-4 and
H.264 capture cards (PCI, PCIe, Mini-PCI).
www.bluecherrydvr.com

Requirements
------------
- A Bluecherry BC-* capture card
- A newer Linux kernel > 2.6.32

Compiling
---------
Simply run: make

Before installing, please remember to uninstall any previously installed
version, as it might install to a different location and interfere.

To install, run: sudo make install && sudo depmod -a

Public bug / feature tracker
---------------------------
The public bug and feature tracker for the solo6x10 driver can be found on
Github:
https://github.com/bluecherrydvr/solo6x10/issues

General questions, comments, suggestions
----------------------------------------
Please visit our support page:
http://support.bluecherrydvr.com/forums/214485-hardware-compression-mpeg-4-h-264-driver
