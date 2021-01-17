# iichid_gpio - iichid driver with GPIO interrupt support for FreeBSD

iichid_gpio is a hacked version of iichid(4) driver from FreeBSD 13 with
built-in support of GPIO interrupts produced by some intel GPIO controllers

It currently support following controllers:

| ACPI _HID | Device name                 |
|-----------|-----------------------------|
| INT344B   | Intel Sunrise Point-LP GPIO |

## System requirements

FreeBSD 13. Earlier versions are not supported.

## Downloading

This project does not have a special home page. The source code and the
issue tracker are hosted on the Github:

https://github.com/wulf7/iichid_gpio

## Building

To build driver, cd in to extracted archive directory and type

```
$ make
```

You need the sources of the running operating system under **/usr/src**

## Installing

To install file already built just type:

```
$ sudo make install
```

To win a booting race with system iichid, it is better to load iichid_gpio
from kernel loader. Add following lines to **/boot/loader.conf** to do that:

```
ig4_load="YES"
iicbus_load="YES"
iichid_gpio_load="YES"
```
## Bug reporting

You can report bugs at 'Project Issues' page
https://github.com/wulf7/iichid_gpio/issues
It is recommended to enclose console output of 'kldload iichid.ko' command
with your report.

Some additional information that can be helpful especially if device has not
been detected at all can be obtained with following commands:

```
$ devinfo -rv	# Very verbose
$ pciconf -lv
$ sudo usbconfig
$ dmesg
```
