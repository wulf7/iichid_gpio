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
#include <sys/kernel.h>
#include <sys/module.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/gpio/gpiobusvar.h>

#include "ichgpio_var.h"

#define	SPTLPGPIO_IRQ_STATUS	0x0100
#define	SPTLPGPIO_IRQ_MASK	0x0120
#define	SPTLPGPIO_GROUP_SIZE	24

#define	SPTLPGPIO_COMMUNITY(_rid, _grps)	{	\
	.rid = (_rid),					\
	.irq_status = SPTLPGPIO_IRQ_STATUS,		\
	.irq_mask = SPTLPGPIO_IRQ_MASK,			\
	.group_size = SPTLPGPIO_GROUP_SIZE,		\
	.groups = (_grps),				\
	.ngroups = nitems(_grps),			\
}

static char *sptlpgpio_ids[] = { "INT344B", NULL };

/* Sunrise Point-LP */
static const ichgpio_pin_t sptlpgpio_pins[] = {
	/* Community 0 */
	/* GPP_A */
	ICHGPIO_PIN(0, "RCINB"),
	ICHGPIO_PIN(1, "LAD_0"),
	ICHGPIO_PIN(2, "LAD_1"),
	ICHGPIO_PIN(3, "LAD_2"),
	ICHGPIO_PIN(4, "LAD_3"),
	ICHGPIO_PIN(5, "LFRAMEB"),
	ICHGPIO_PIN(6, "SERIQ"),
	ICHGPIO_PIN(7, "PIRQAB"),
	ICHGPIO_PIN(8, "CLKRUNB"),
	ICHGPIO_PIN(9, "CLKOUT_LPC_0"),
	ICHGPIO_PIN(10, "CLKOUT_LPC_1"),
	ICHGPIO_PIN(11, "PMEB"),
	ICHGPIO_PIN(12, "BM_BUSYB"),
	ICHGPIO_PIN(13, "SUSWARNB_SUS_PWRDNACK"),
	ICHGPIO_PIN(14, "SUS_STATB"),
	ICHGPIO_PIN(15, "SUSACKB"),
	ICHGPIO_PIN(16, "SD_1P8_SEL"),
	ICHGPIO_PIN(17, "SD_PWR_EN_B"),
	ICHGPIO_PIN(18, "ISH_GP_0"),
	ICHGPIO_PIN(19, "ISH_GP_1"),
	ICHGPIO_PIN(20, "ISH_GP_2"),
	ICHGPIO_PIN(21, "ISH_GP_3"),
	ICHGPIO_PIN(22, "ISH_GP_4"),
	ICHGPIO_PIN(23, "ISH_GP_5"),
	/* GPP_B */
	ICHGPIO_PIN(24, "CORE_VID_0"),
	ICHGPIO_PIN(25, "CORE_VID_1"),
	ICHGPIO_PIN(26, "VRALERTB"),
	ICHGPIO_PIN(27, "CPU_GP_2"),
	ICHGPIO_PIN(28, "CPU_GP_3"),
	ICHGPIO_PIN(29, "SRCCLKREQB_0"),
	ICHGPIO_PIN(30, "SRCCLKREQB_1"),
	ICHGPIO_PIN(31, "SRCCLKREQB_2"),
	ICHGPIO_PIN(32, "SRCCLKREQB_3"),
	ICHGPIO_PIN(33, "SRCCLKREQB_4"),
	ICHGPIO_PIN(34, "SRCCLKREQB_5"),
	ICHGPIO_PIN(35, "EXT_PWR_GATEB"),
	ICHGPIO_PIN(36, "SLP_S0B"),
	ICHGPIO_PIN(37, "PLTRSTB"),
	ICHGPIO_PIN(38, "SPKR"),
	ICHGPIO_PIN(39, "GSPI0_CSB"),
	ICHGPIO_PIN(40, "GSPI0_CLK"),
	ICHGPIO_PIN(41, "GSPI0_MISO"),
	ICHGPIO_PIN(42, "GSPI0_MOSI"),
	ICHGPIO_PIN(43, "GSPI1_CSB"),
	ICHGPIO_PIN(44, "GSPI1_CLK"),
	ICHGPIO_PIN(45, "GSPI1_MISO"),
	ICHGPIO_PIN(46, "GSPI1_MOSI"),
	ICHGPIO_PIN(47, "SML1ALERTB"),
	/* Community 1 */
	/* GPP_C */
	ICHGPIO_PIN(48, "SMBCLK"),
	ICHGPIO_PIN(49, "SMBDATA"),
	ICHGPIO_PIN(50, "SMBALERTB"),
	ICHGPIO_PIN(51, "SML0CLK"),
	ICHGPIO_PIN(52, "SML0DATA"),
	ICHGPIO_PIN(53, "SML0ALERTB"),
	ICHGPIO_PIN(54, "SML1CLK"),
	ICHGPIO_PIN(55, "SML1DATA"),
	ICHGPIO_PIN(56, "UART0_RXD"),
	ICHGPIO_PIN(57, "UART0_TXD"),
	ICHGPIO_PIN(58, "UART0_RTSB"),
	ICHGPIO_PIN(59, "UART0_CTSB"),
	ICHGPIO_PIN(60, "UART1_RXD"),
	ICHGPIO_PIN(61, "UART1_TXD"),
	ICHGPIO_PIN(62, "UART1_RTSB"),
	ICHGPIO_PIN(63, "UART1_CTSB"),
	ICHGPIO_PIN(64, "I2C0_SDA"),
	ICHGPIO_PIN(65, "I2C0_SCL"),
	ICHGPIO_PIN(66, "I2C1_SDA"),
	ICHGPIO_PIN(67, "I2C1_SCL"),
	ICHGPIO_PIN(68, "UART2_RXD"),
	ICHGPIO_PIN(69, "UART2_TXD"),
	ICHGPIO_PIN(70, "UART2_RTSB"),
	ICHGPIO_PIN(71, "UART2_CTSB"),
	/* GPP_D */
	ICHGPIO_PIN(72, "SPI1_CSB"),
	ICHGPIO_PIN(73, "SPI1_CLK"),
	ICHGPIO_PIN(74, "SPI1_MISO_IO_1"),
	ICHGPIO_PIN(75, "SPI1_MOSI_IO_0"),
	ICHGPIO_PIN(76, "FLASHTRIG"),
	ICHGPIO_PIN(77, "ISH_I2C0_SDA"),
	ICHGPIO_PIN(78, "ISH_I2C0_SCL"),
	ICHGPIO_PIN(79, "ISH_I2C1_SDA"),
	ICHGPIO_PIN(80, "ISH_I2C1_SCL"),
	ICHGPIO_PIN(81, "ISH_SPI_CSB"),
	ICHGPIO_PIN(82, "ISH_SPI_CLK"),
	ICHGPIO_PIN(83, "ISH_SPI_MISO"),
	ICHGPIO_PIN(84, "ISH_SPI_MOSI"),
	ICHGPIO_PIN(85, "ISH_UART0_RXD"),
	ICHGPIO_PIN(86, "ISH_UART0_TXD"),
	ICHGPIO_PIN(87, "ISH_UART0_RTSB"),
	ICHGPIO_PIN(88, "ISH_UART0_CTSB"),
	ICHGPIO_PIN(89, "DMIC_CLK_1"),
	ICHGPIO_PIN(90, "DMIC_DATA_1"),
	ICHGPIO_PIN(91, "DMIC_CLK_0"),
	ICHGPIO_PIN(92, "DMIC_DATA_0"),
	ICHGPIO_PIN(93, "SPI1_IO_2"),
	ICHGPIO_PIN(94, "SPI1_IO_3"),
	ICHGPIO_PIN(95, "SSP_MCLK"),
	/* GPP_E */
	ICHGPIO_PIN(96, "SATAXPCIE_0"),
	ICHGPIO_PIN(97, "SATAXPCIE_1"),
	ICHGPIO_PIN(98, "SATAXPCIE_2"),
	ICHGPIO_PIN(99, "CPU_GP_0"),
	ICHGPIO_PIN(100, "SATA_DEVSLP_0"),
	ICHGPIO_PIN(101, "SATA_DEVSLP_1"),
	ICHGPIO_PIN(102, "SATA_DEVSLP_2"),
	ICHGPIO_PIN(103, "CPU_GP_1"),
	ICHGPIO_PIN(104, "SATA_LEDB"),
	ICHGPIO_PIN(105, "USB2_OCB_0"),
	ICHGPIO_PIN(106, "USB2_OCB_1"),
	ICHGPIO_PIN(107, "USB2_OCB_2"),
	ICHGPIO_PIN(108, "USB2_OCB_3"),
	ICHGPIO_PIN(109, "DDSP_HPD_0"),
	ICHGPIO_PIN(110, "DDSP_HPD_1"),
	ICHGPIO_PIN(111, "DDSP_HPD_2"),
	ICHGPIO_PIN(112, "DDSP_HPD_3"),
	ICHGPIO_PIN(113, "EDP_HPD"),
	ICHGPIO_PIN(114, "DDPB_CTRLCLK"),
	ICHGPIO_PIN(115, "DDPB_CTRLDATA"),
	ICHGPIO_PIN(116, "DDPC_CTRLCLK"),
	ICHGPIO_PIN(117, "DDPC_CTRLDATA"),
	ICHGPIO_PIN(118, "DDPD_CTRLCLK"),
	ICHGPIO_PIN(119, "DDPD_CTRLDATA"),
	/* Community 3 */
	/* GPP_F */
	ICHGPIO_PIN(120, "SSP2_SCLK"),
	ICHGPIO_PIN(121, "SSP2_SFRM"),
	ICHGPIO_PIN(122, "SSP2_TXD"),
	ICHGPIO_PIN(123, "SSP2_RXD"),
	ICHGPIO_PIN(124, "I2C2_SDA"),
	ICHGPIO_PIN(125, "I2C2_SCL"),
	ICHGPIO_PIN(126, "I2C3_SDA"),
	ICHGPIO_PIN(127, "I2C3_SCL"),
	ICHGPIO_PIN(128, "I2C4_SDA"),
	ICHGPIO_PIN(129, "I2C4_SCL"),
	ICHGPIO_PIN(130, "I2C5_SDA"),
	ICHGPIO_PIN(131, "I2C5_SCL"),
	ICHGPIO_PIN(132, "EMMC_CMD"),
	ICHGPIO_PIN(133, "EMMC_DATA_0"),
	ICHGPIO_PIN(134, "EMMC_DATA_1"),
	ICHGPIO_PIN(135, "EMMC_DATA_2"),
	ICHGPIO_PIN(136, "EMMC_DATA_3"),
	ICHGPIO_PIN(137, "EMMC_DATA_4"),
	ICHGPIO_PIN(138, "EMMC_DATA_5"),
	ICHGPIO_PIN(139, "EMMC_DATA_6"),
	ICHGPIO_PIN(140, "EMMC_DATA_7"),
	ICHGPIO_PIN(141, "EMMC_RCLK"),
	ICHGPIO_PIN(142, "EMMC_CLK"),
	ICHGPIO_PIN(143, "GPP_F_23"),
	/* GPP_G */
	ICHGPIO_PIN(144, "SD_CMD"),
	ICHGPIO_PIN(145, "SD_DATA_0"),
	ICHGPIO_PIN(146, "SD_DATA_1"),
	ICHGPIO_PIN(147, "SD_DATA_2"),
	ICHGPIO_PIN(148, "SD_DATA_3"),
	ICHGPIO_PIN(149, "SD_CDB"),
	ICHGPIO_PIN(150, "SD_CLK"),
	ICHGPIO_PIN(151, "SD_WP"),
};

static const ichgpio_group_t sclgpio_comm0_groups[] = {
	ICHGPIO_GROUP(SPTLPGPIO_GROUP_SIZE),
	ICHGPIO_GROUP(SPTLPGPIO_GROUP_SIZE),
};
static const ichgpio_group_t sclgpio_comm1_groups[] = {
	ICHGPIO_GROUP(SPTLPGPIO_GROUP_SIZE),
	ICHGPIO_GROUP(SPTLPGPIO_GROUP_SIZE),
	ICHGPIO_GROUP(SPTLPGPIO_GROUP_SIZE),
};
static const ichgpio_group_t sclgpio_comm3_groups[] = {
	ICHGPIO_GROUP(SPTLPGPIO_GROUP_SIZE),
	ICHGPIO_GROUP(8),
};

static const ichgpio_comm_t sptlpgpio_comms[] = {
	SPTLPGPIO_COMMUNITY(0, sclgpio_comm0_groups),
	SPTLPGPIO_COMMUNITY(1, sclgpio_comm1_groups),
	SPTLPGPIO_COMMUNITY(2, sclgpio_comm3_groups),
};

static int
sptlpgpio_probe(device_t dev)
{
	ACPI_HANDLE handle;
	int rv;
	int uid;

	if (acpi_disabled("sptlpgpio"))
		return (ENXIO);

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, sptlpgpio_ids, NULL);
	if (rv > 0)
		return (rv);

	handle = acpi_get_handle(dev);
	if (ACPI_FAILURE(acpi_GetInteger(handle, "_UID", &uid)))
		uid = 0;
	if (uid != 0)	/* SW_UID */
		return (ENXIO);

	device_set_desc(dev, "Intel Sunrise Point-LP GPIO");

	return (rv);
}

static int
sptlpgpio_attach(device_t dev)
{
	ICHGPIO_REGISTER_DATA(dev, sptlpgpio_comms, sptlpgpio_pins);

	return (ichgpio_attach(dev));
}

static device_method_t sptlpgpio_methods[] = {
	DEVMETHOD(device_probe,     	sptlpgpio_probe),
	DEVMETHOD(device_attach,	sptlpgpio_attach),
	DEVMETHOD(device_detach,	ichgpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		ichgpio_get_bus),
	DEVMETHOD(gpio_pin_max,		ichgpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	ichgpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	ichgpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	ichgpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	ichgpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		ichgpio_pin_get),
	DEVMETHOD(gpio_pin_set,		ichgpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	ichgpio_pin_toggle),

	DEVMETHOD_END
};

DEFINE_CLASS_0(ichgpio, sptlpgpio_driver, sptlpgpio_methods,
    sizeof(struct ichgpio_softc));
DRIVER_MODULE(sptlpgpio, acpi, sptlpgpio_driver, ichgpio_devclass, NULL, 0);
MODULE_DEPEND(sptlpgpio, acpi, 1, 1, 1);
MODULE_DEPEND(sptlpgpio, ichgpio, 1, 1, 1);
MODULE_VERSION(sptlpgpio, 1);
ACPI_PNP_INFO(sptlpgpio_ids);
