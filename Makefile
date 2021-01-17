KMOD =	iichid_gpio
SRCS =	ichgpio.c ichgpio_var.h ichgpio_reg.h
SRCS +=	sptlpgpio.c
SRCS +=	iichid.c
SRCS +=	acpi_if.h bus_if.h device_if.h gpio_if.h hid_if.h iicbus_if.h
SRCS +=	opt_acpi.h opt_hid.h opt_platform.h
CFLAGS +=	-DINVARIANTS -DINVARIANT_SUPPORT
CFLAGS +=	-DIICHID_DEBUG
CFLAGS +=	-DIICHID_SAMPLING
CFLAGS +=	-DIICHID_GPIO_INTR

.include <bsd.kmod.mk>
