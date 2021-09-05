/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Vladimir Kondratyev <wulf@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/gpio/gpiobusvar.h>

#include "ichgpio_var.h"
#include "ichgpio_reg.h"

static inline uint32_t
ichgpio_read_4(struct ichgpio_softc *sc, int comm, bus_size_t off)
{
	return (bus_read_4(sc->sc_comms[comm].mem_res, off));
}

static inline void
ichgpio_write_4(struct ichgpio_softc *sc, int comm, bus_size_t off,
    uint32_t val)
{
	bus_write_4(sc->sc_comms[comm].mem_res, off, val);
}

static inline int
ichgpio_pad_community(struct ichgpio_softc *sc, int pin, int *group, int *line)
{
	int comm;
	int grp;
	int min_pin, next_pin;

	for (comm = 0; comm < sc->sc_ncomms; comm++)
		if (pin >= sc->sc_comms[comm].min_pin &&
		    pin <= sc->sc_comms[comm].max_pin)
			break;
	MPASS(comm < sc->sc_ncomms);
	if (group == NULL && line == NULL)
		return (comm);

	min_pin = sc->sc_comms[comm].min_pin;
	for (grp = 0; grp < sc->sc_descs[comm].ngroups; grp++) {
		next_pin = min_pin + sc->sc_descs[comm].groups[grp].npads;
		if (pin < next_pin) {
			if (group != NULL)
				*group = grp;
			if (line != NULL)
				*line = pin - min_pin;
			break;
		}
		min_pin = next_pin;
	}
	MPASS(grp < sc->sc_descs[comm].ngroups);
	return (comm);
}

static inline bus_size_t
ichgpio_pad_cfg0_offset(struct ichgpio_softc *sc, int comm, int group,
    int line)
{
	return (sc->sc_comms[comm].padbar +
	    (group * sc->sc_descs[comm].group_size + line) *
	    (sc->sc_comms[comm].features.debounce ? 16 : 8));
}

static inline int
ichgpio_read_pad_cfg0(struct ichgpio_softc *sc, int pin)
{
	int comm, group, line;
	bus_size_t pad_cfg0_offset;

	comm = ichgpio_pad_community(sc, pin, &group, &line);
	pad_cfg0_offset = ichgpio_pad_cfg0_offset(sc, comm, group, line);
	return (ichgpio_read_4(sc, comm, pad_cfg0_offset));
}

static inline void
ichgpio_write_pad_cfg0(struct ichgpio_softc *sc, int pin, uint32_t val)
{
	int comm, group, line;
	bus_size_t pad_cfg0_offset;

	comm = ichgpio_pad_community(sc, pin, &group, &line);
	pad_cfg0_offset = ichgpio_pad_cfg0_offset(sc, comm, group, line);
	ichgpio_write_4(sc, comm, pad_cfg0_offset, val);
}

static inline int
ichgpio_read_pad_cfg1(struct ichgpio_softc *sc, int pin)
{
	int comm, group, line;
	bus_size_t pad_cfg0_offset;

	comm = ichgpio_pad_community(sc, pin, &group, &line);
	pad_cfg0_offset = ichgpio_pad_cfg0_offset(sc, comm, group, line);
	return (ichgpio_read_4(sc, comm, pad_cfg0_offset + 4));
}

device_t
ichgpio_get_bus(device_t dev)
{
	struct ichgpio_softc *sc = device_get_softc(dev);

	return (sc->sc_busdev);
}

int
ichgpio_pin_max(device_t dev, int *maxpin)
{
	struct ichgpio_softc *sc = device_get_softc(dev);

	*maxpin = sc->sc_npins - 1;

	return (0);
}

static int
ichgpio_valid_pin(struct ichgpio_softc *sc, int pin)
{
	if (pin >= sc->sc_npins || sc->sc_pins[pin] == NULL)
		return (EINVAL);

	return (0);
}

int
ichgpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct ichgpio_softc *sc = device_get_softc(dev);

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	/* return pin name from datasheet */
	snprintf(name, GPIOMAXNAME, "%s", sc->sc_pins[pin]);
	name[GPIOMAXNAME - 1] = '\0';
	return (0);
}

/*
 * Returns true if pad configured to be used as GPIO
 */
static bool
ichgpio_pad_is_gpio(struct ichgpio_softc *sc, int pin)
{
	return (true);
}

int
ichgpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct ichgpio_softc *sc = device_get_softc(dev);

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	*caps = 0;
	if (ichgpio_pad_is_gpio(sc, pin))
		*caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;

	return (0);
}

int
ichgpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	uint32_t val;

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	*flags = 0;

	/* Get the current pin state */
	ICHGPIO_LOCK(sc);
	val = ichgpio_read_pad_cfg0(sc, pin);

	if (val & ICHGPIO_PAD_CFG0_GPIOCFG_GPIO ||
		val & ICHGPIO_PAD_CFG0_GPIOCFG_GPO)
		*flags |= GPIO_PIN_OUTPUT;

	if (val & ICHGPIO_PAD_CFG0_GPIOCFG_GPIO ||
		val & ICHGPIO_PAD_CFG0_GPIOCFG_GPI)
		*flags |= GPIO_PIN_INPUT;

	val = ichgpio_read_pad_cfg1(sc, pin);

	ICHGPIO_UNLOCK(sc);
	return (0);
}

int
ichgpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	uint32_t val;
	uint32_t allowed;

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	allowed = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;

	/*
	 * Only direction flag allowed
	 */
	if (flags & ~allowed)
		return (EINVAL);

	/*
	 * Not both directions simultaneously
	 */
	if ((flags & allowed) == allowed)
		return (EINVAL);

	/* Set the GPIO mode and state */
	ICHGPIO_LOCK(sc);
	val = ichgpio_read_pad_cfg0(sc, pin);
	if (flags & GPIO_PIN_INPUT)
		val = val & ICHGPIO_PAD_CFG0_GPIOCFG_GPI;
	if (flags & GPIO_PIN_OUTPUT)
		val = val & ICHGPIO_PAD_CFG0_GPIOCFG_GPO;
	ichgpio_write_pad_cfg0(sc, pin, val);
	ICHGPIO_UNLOCK(sc);

	return (0);
}

int
ichgpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	uint32_t val;

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	ICHGPIO_LOCK(sc);
	val = ichgpio_read_pad_cfg0(sc, pin);
	if (value == GPIO_PIN_LOW)
		val = val & ~ICHGPIO_PAD_CFG0_GPIOTXSTATE;
	else
		val = val | ICHGPIO_PAD_CFG0_GPIOTXSTATE;
	ichgpio_write_pad_cfg0(sc, pin, val);
	ICHGPIO_UNLOCK(sc);

	return (0);
}

int
ichgpio_pin_get(device_t dev, uint32_t pin, unsigned int *value)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	uint32_t val;

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	ICHGPIO_LOCK(sc);

	/* Read pin value */
	val = ichgpio_read_pad_cfg0(sc, pin);
	if (val & ICHGPIO_PAD_CFG0_GPIORXSTATE)
		*value = GPIO_PIN_HIGH;
	else
		*value = GPIO_PIN_LOW;

	ICHGPIO_UNLOCK(sc);

	return (0);
}

int
ichgpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	uint32_t val;

	if (ichgpio_valid_pin(sc, pin) != 0)
		return (EINVAL);

	ICHGPIO_LOCK(sc);

	/* Toggle the pin */
	val = ichgpio_read_pad_cfg0(sc, pin);
	val = val ^ ICHGPIO_PAD_CFG0_GPIOTXSTATE;
	ichgpio_write_pad_cfg0(sc, pin, val);

	ICHGPIO_UNLOCK(sc);

	return (0);
}

void
ichgpio_register_data(device_t dev, const ichgpio_comm_t *comm_descs,
    int ncomms, const ichgpio_pin_t *pins, int npins)
{
	struct ichgpio_softc *sc = device_get_softc(dev);

	sc->sc_pins = pins;
	sc->sc_npins = npins;
	sc->sc_descs = comm_descs;
	sc->sc_ncomms = ncomms;
}

void
ichgpio_intr_establish(device_t dev, int pin, uint32_t mode,
    void (*func)(void *), void *arg)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	int comm, group, line;
	uint32_t reg;

	MPASS(pin >= 0 && pin < sc->sc_npins);

	sc->sc_pin_ih[pin].ih_func = func;
	sc->sc_pin_ih[pin].ih_arg = arg;

	reg = ichgpio_read_pad_cfg0(sc, pin);
	reg &= ~(ICHGPIO_PAD_CFG0_RXEVCFG_MASK | ICHGPIO_PAD_CFG0_RXINV);
	switch (mode) {
	case GPIO_INTR_LEVEL_LOW:
		reg |= ICHGPIO_PAD_CFG0_RXEVCFG_LEVEL | ICHGPIO_PAD_CFG0_RXINV;
		break;
	case GPIO_INTR_LEVEL_HIGH:
		reg |= ICHGPIO_PAD_CFG0_RXEVCFG_LEVEL;
		break;
	case GPIO_INTR_EDGE_RISING:
		reg |= ICHGPIO_PAD_CFG0_RXEVCFG_EDGE;
		break;
	case GPIO_INTR_EDGE_FALLING:
		reg |= ICHGPIO_PAD_CFG0_RXEVCFG_EDGE | ICHGPIO_PAD_CFG0_RXINV;
		break;
	case GPIO_INTR_EDGE_BOTH:
		reg |= ICHGPIO_PAD_CFG0_RXEVCFG_EDGE |
		    ICHGPIO_PAD_CFG0_RXEVCFG_ZERO;
		break;
	default:
		KASSERT(0, ("Unsupported interrupt mode 0x%8x\n", mode));
		return;
	}
	ichgpio_write_pad_cfg0(sc, pin, reg);

	comm = ichgpio_pad_community(sc, pin, &group, &line);
	reg = ichgpio_read_4(sc, comm, sc->sc_descs[comm].irq_mask + group*4);
	ichgpio_write_4(sc, comm, sc->sc_descs[comm].irq_mask + group * 4,
	    reg | (1 << line));

	return;
}

void
ichgpio_intr_disestablish(device_t dev, int pin)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	int comm, group, line;
	uint32_t reg;

	MPASS(pin >= 0 && pin < sc->sc_npins);

	sc->sc_pin_ih[pin].ih_func = NULL;
	sc->sc_pin_ih[pin].ih_arg = NULL;

	comm = ichgpio_pad_community(sc, pin, &group, &line);
	reg = ichgpio_read_4(sc, comm, sc->sc_descs[comm].irq_mask + group*4);
	ichgpio_write_4(sc, comm, sc->sc_descs[comm].irq_mask + group * 4,
	    reg & ~(1 << line));
}

static void
ichgpio_intr(void *arg)
{
	struct ichgpio_softc *sc = arg;
	int comm, group, line, pin = 0;
	uint32_t reg;

	for (comm = 0; comm < sc->sc_ncomms; comm++) {
		for (group = 0; group < sc->sc_descs[comm].ngroups; group++) {
			reg = ichgpio_read_4(sc, comm,
			    sc->sc_descs[comm].irq_status + group * 4);
			ichgpio_write_4(sc, comm,
			    sc->sc_descs[comm].irq_status + group * 4, reg);
			reg &= ichgpio_read_4(sc, comm,
			    sc->sc_descs[comm].irq_mask + group * 4);
			for (line = 0;
			     line < sc->sc_descs[comm].groups[group].npads;
			     line++, pin++) {
				if ((reg & (1 << line)) != 0 &&
				    sc->sc_pin_ih[pin].ih_func != NULL)
					sc->sc_pin_ih[pin].
					    ih_func(sc->sc_pin_ih[pin].ih_arg);
			}
		}
	}
}

int
ichgpio_attach(device_t dev)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	int comm, group;
	int rid;
	int min_pin;
	uint32_t rev;
	int error;

	sc->sc_dev = dev;

	ICHGPIO_LOCK_INIT(sc);
	sc->sc_comms = malloc(sizeof(struct ichgpio_comm) * sc->sc_ncomms,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	min_pin = 0;
	for (comm = 0; comm < sc->sc_ncomms; comm++) {
		rid = sc->sc_descs[comm].rid;
		sc->sc_comms[comm].mem_res = bus_alloc_resource_any(sc->sc_dev,
		    SYS_RES_MEMORY, &rid, RF_ACTIVE);
		if (sc->sc_comms[comm].mem_res == NULL) {
			device_printf(dev, "can't allocate memory resource\n");
			ichgpio_detach(dev);
			return (ENOMEM);
		}
		sc->sc_comms[comm].padbar = ichgpio_read_4(sc, comm,
		    ICHGPIO_REG_PADBAR);
		/* Determine community features based on the revision */
		rev = (ichgpio_read_4(sc, comm, ICHGPIO_REG_REVID) &
		    ICHGPIO_REG_REVID_MASK) >> ICHGPIO_REG_REVID_SHIFT;
		if (rev >= 0x94) {
			sc->sc_comms[comm].features.debounce = true;
			sc->sc_comms[comm].features.onek_pd = true;
		}

		sc->sc_comms[comm].min_pin = min_pin;
		for (group = 0; group < sc->sc_descs[comm].ngroups; group++) {
			min_pin += sc->sc_descs[comm].groups[group].npads;
			/* Mask and ack all interrupts. */
			ichgpio_write_4(sc, comm,
			    sc->sc_descs[comm].irq_mask + group * 4, 0);
			ichgpio_write_4(sc, comm,
			    sc->sc_descs[comm].irq_status + group * 4, 0xffff);
		}
		sc->sc_comms[comm].max_pin = min_pin - 1;
	}

	KASSERT(min_pin == sc->sc_npins, ("Communities does not match pins"));

	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE);
	if (!sc->sc_irq_res) {
		device_printf(dev, "can't allocate irq resource\n");
		ichgpio_detach(dev);
		return (ENOMEM);
	}

	error = bus_setup_intr(sc->sc_dev, sc->sc_irq_res,
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, ichgpio_intr, sc, &sc->sc_ih);
	if (error) {
		device_printf(sc->sc_dev, "unable to setup irq: error %d\n",
		    error);
		ichgpio_detach(dev);
		return (ENXIO);
	}

	sc->sc_busdev = gpiobus_attach_bus(dev);
	if (sc->sc_busdev == NULL) {
		ichgpio_detach(dev);
		return (ENXIO);
	}

	sc->sc_pin_ih = malloc(sizeof(struct ichgpio_intrhand) * sc->sc_npins,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	return (0);
}

int
ichgpio_detach(device_t dev)
{
	struct ichgpio_softc *sc = device_get_softc(dev);
	int comm;

	if (sc->sc_busdev)
		gpiobus_detach_bus(dev);

	if (sc->sc_ih != NULL)
		bus_teardown_intr(sc->sc_dev, sc->sc_irq_res, sc->sc_ih);
	sc->sc_ih = NULL;
	if (sc->sc_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
	sc->sc_irq_res = NULL;
	for (comm = 0; comm < sc->sc_ncomms; comm++) {
		if (sc->sc_comms[comm].mem_res != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY,
			   sc->sc_descs[comm].rid, sc->sc_comms[comm].mem_res);
		sc->sc_comms[comm].mem_res = NULL;
	}

	ICHGPIO_LOCK_DESTROY(sc);
	free(sc->sc_comms, M_DEVBUF);
	free(sc->sc_pin_ih, M_DEVBUF);

	return (0);
}

devclass_t ichgpio_devclass;
MODULE_DEPEND(ichgpio, gpiobus, 1, 1, 1);
MODULE_VERSION(ichgpio, 1);
