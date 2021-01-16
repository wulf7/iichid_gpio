KMOD =	iichid_gpio
SRCS =	iichid.c
SRCS +=	acpi_if.h bus_if.h device_if.h hid_if.h iicbus_if.h
SRCS +=	opt_acpi.h opt_hid.h
CFLAGS +=	-DINVARIANTS -DINVARIANT_SUPPORT
CFLAGS +=	-DIICHID_DEBUG
CFLAGS +=	-DIICHID_SAMPLING

.include <bsd.kmod.mk>
