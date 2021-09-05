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

#ifndef _ICHGPIO_REG_H_
#define _ICHGPIO_REG_H_

#include <machine/bus.h>
#include <machine/resource.h>

/*
 *     Macros for driver mutex locking
 */
#define ICHGPIO_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define ICHGPIO_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define ICHGPIO_LOCK_INIT(_sc)		mtx_init(&_sc->sc_mtx, \
    device_get_nameunit((_sc)->sc_dev), NULL, MTX_DEF)
#define ICHGPIO_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->sc_mtx)
#define ICHGPIO_ASSERT_LOCKED(_sc)	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)
#define ICHGPIO_ASSERT_UNLOCKED(_sc)	mtx_assert(&(_sc)->sc_mtx, MA_NOTOWNED)

#define	ICHGPIO_PIN(pin, name)	[pin] = (name)
#define	ICHGPIO_NOPIN(pin)	[pin] = NULL
#define	ICHGPIO_GROUP(gb, np)	{ .gpiobase = (gb), .npads = (np) }

#define	ICHGPIO_NOBASE		-1	/* no GPIO mapping should be created */

typedef	char *				ichgpio_pin_t;
typedef	struct ichgpio_group_desc	ichgpio_group_t;
typedef	struct ichgpio_comm_desc	ichgpio_comm_t;

struct ichgpio_group_desc {
	int		gpiobase;
	int		npads;
};

struct ichgpio_comm_desc {
	int		rid;
	int		irq_status;	/* IRQ status register offset */
	int		irq_mask;	/* IRQ mask register offset */
	const ichgpio_group_t *groups;	/* Number of pins in each pad group */
	int		ngroups;	/* Number of pad groups in commutity */
};

struct ichgpio_comm {
	struct resource	*mem_res;

	int		min_pin;
	int		max_pin;
	uint32_t	padbar;
	struct {
		bool	debounce:1;
		bool	onek_pd:1;
		int	reserved:30;
	}		features;
};

struct ichgpio_intrhand {
	void (*ih_func)(void *);
	void *ih_arg;
};

struct ichgpio_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	struct mtx		sc_mtx;

	int			sc_ncomms;
	const ichgpio_comm_t	*sc_descs;
	struct ichgpio_comm	*sc_comms;

	int			sc_npins;
	const ichgpio_pin_t 	*sc_pins;
	struct ichgpio_intrhand	*sc_pin_ih;

	int			sc_irq_rid;
	struct resource		*sc_irq_res;
	void			*sc_ih;
};

#define	ICHGPIO_REGISTER_DATA(dev, descs, pins) \
    ichgpio_register_data((dev), (descs), nitems(descs), (pins), nitems(pins))
void	ichgpio_register_data(device_t dev, const ichgpio_comm_t *comm_descs,
	    int ncomms, const ichgpio_pin_t *pins, int npins);
void	ichgpio_intr_establish(device_t, int, uint32_t, void(*)(void *),
	    void *);
void	ichgpio_intr_disestablish(device_t, int);

extern devclass_t ichgpio_devclass;

device_attach_t	ichgpio_attach;
device_detach_t	ichgpio_detach;

device_t	ichgpio_get_bus(device_t dev);
int	ichgpio_pin_max(device_t dev, int *maxpin);
int	ichgpio_pin_getname(device_t dev, uint32_t pin, char *name);
int	ichgpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps);
int	ichgpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags);
int	ichgpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags);
int	ichgpio_pin_set(device_t dev, uint32_t pin, unsigned int value);
int	ichgpio_pin_get(device_t dev, uint32_t pin, unsigned int *value);
int	ichgpio_pin_toggle(device_t dev, uint32_t pin);

#endif	/* _ICHGPIO_REG_H_ */
