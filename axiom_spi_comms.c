// SPDX-License-Identifier: GPL-2.0
/*
 * TouchNetix aXiom Touchscreen Driver
 *
 * Copyright (C) 2018-2023 TouchNetix Ltd.
 *
 * Author(s): Mark Satterthwaite <mark.satterthwaite@touchnetix.com>
 *            Bart Prescott <bartp@baasheep.co.uk>
 *            Hannah Rossiter <hannah.rossiter@touchnetix.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/version.h>
#include "axiom_core.h"

#define COMMS_HEADER_LENGTH      (4U)
#define COMMS_PADDING_LENGTH    (32U)

static bool poll_enable;
module_param(poll_enable, bool, 0444);
MODULE_PARM_DESC(poll_enable, "Enable polling mode [default 0=no]");

static int poll_interval;
module_param(poll_interval, int, 0444);
MODULE_PARM_DESC(poll_interval, "Polling period in ms [default = 100]");

struct axiom_data {
	struct axiom_data_core data_core;
	struct spi_device *spi;

	u8 pad_buf[COMMS_PADDING_LENGTH];
	bool irq_allocated; // indicates the IRQ was allocated during probe
};

// purpose: Helper function to read a specified usage and write it into the provided buffer
// returns: Length of the usage read
static u16 axiom_read_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
	struct axiom_data *data = pAxiomData;
	struct spi_device *spi = data->spi;
	struct spi_transfer xfr_header;
	struct spi_transfer xfr_padding;
	struct spi_transfer xfr_payload;
	struct spi_message msg;
	struct AxiomCmdHeader cmdHeader;
	s32 rc;

	memset(&xfr_header,  0, sizeof(xfr_header));
	memset(&xfr_padding, 0, sizeof(xfr_padding));
	memset(&xfr_payload, 0, sizeof(xfr_payload));

	// Build the header
	cmdHeader.target_address = usage_to_target_address(&data->data_core, usage, page, 0);
	cmdHeader.length = length;
	cmdHeader.read = 1; // READ Bit set

	// Setup the SPI transfer operations
	xfr_header.tx_buf = &cmdHeader;
	xfr_header.len    = sizeof(cmdHeader);

	// Nothing to do for the padding, it is already zeroed
	// The payload will get filled in by the SPI operation

	xfr_padding.tx_buf = data->pad_buf;
	xfr_padding.len    = sizeof(data->pad_buf);

	xfr_payload.rx_buf = pBuffer;
	xfr_payload.len    = length;

	// Build the SPI transaction
	spi_message_init(&msg);
	spi_message_add_tail(&xfr_header, &msg);
	spi_message_add_tail(&xfr_padding, &msg);
	spi_message_add_tail(&xfr_payload, &msg);

	rc = spi_sync(spi, &msg);
	if (rc != 0) {
		// SPI failed!
		dev_err(&data->spi->dev, "Failed to SPI transfer. RC:%d\n", rc);
		return 0;
	}

	udelay(data->data_core.bus_holdoff_delay_us);
	return length;
}

// purpose: Helper function to write data in a provided buffer to a specified usage
// returns: Length of the data to write
static u16 axiom_write_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
	struct axiom_data *data = pAxiomData;
	struct spi_device *spi = data->spi;
	struct spi_transfer xfr_header;
	struct spi_transfer xfr_padding;
	struct spi_transfer xfr_payload;
	struct spi_message msg;
	struct AxiomCmdHeader cmdHeader;
	s32 rc;

	memset(&xfr_header,  0, sizeof(xfr_header));
	memset(&xfr_padding, 0, sizeof(xfr_padding));
	memset(&xfr_payload, 0, sizeof(xfr_payload));

	// Build the header
	cmdHeader.target_address = usage_to_target_address(&data->data_core, usage, page, 0);
	cmdHeader.length = length;
	cmdHeader.read = 1; // READ Bit set

	// Setup the SPI transfer operations
	xfr_header.tx_buf = &cmdHeader;
	xfr_header.len    = sizeof(cmdHeader);

	// Nothing to do for the padding, it is already zeroed
	// Setup the SPI transfer operations
	xfr_padding.tx_buf = data->pad_buf;
	xfr_padding.len    = sizeof(data->pad_buf);

	xfr_payload.tx_buf = pBuffer;
	xfr_payload.len    = length;

	// Build the SPI transaction
	spi_message_init(&msg);
	spi_message_add_tail(&xfr_header, &msg);
	spi_message_add_tail(&xfr_padding, &msg);
	spi_message_add_tail(&xfr_payload, &msg);

	rc = spi_sync(spi, &msg);
	if (rc != 0) {
		// SPI failed!
		dev_err(&data->spi->dev, "Failed to SPI transfer. RC:%d\n", rc);
		return 0;
	}

	udelay(data->data_core.bus_holdoff_delay_us);
	return length;
}

// purpose: Process the interrupt notifying the system a new report is available
// returns: Value to indicate the interrupt has been handled
static irqreturn_t axiom_irq(int irq, void *handle)
{
	struct axiom_data *data = handle;
	u8 *pRX_data = &data->data_core.rx_buf[0];

	axiom_read_usage(data, 0x34, 0, data->data_core.max_report_len, pRX_data);
	axiom_process_report(&data->data_core, pRX_data);

	return IRQ_HANDLED;
}

// purpose: Function called in IRQ context when device is plugged in.
// returns: Error code
static int axiom_spi_probe(struct spi_device *spi)
{
	struct axiom_data *data;
	struct axiom_data_core *data_core;
	u32 error;
	u32 target;

	dev_info(&spi->dev, "Probe Start\n");

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 4000000;

	if (spi->irq == 0)
		dev_err(&spi->dev, "No IRQ specified!\n");

	dev_info(&spi->dev, "Using IRQ on pin: %u\n", spi->irq);

	error = spi_setup(spi);
	if (error) {
		dev_err(&spi->dev, "Failed to setup SPI bus!\n");
		return error;
	}

	// Kernel will manage this data, it will be automatically unloaded when the
	// module is unloaded.
	data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_ATOMIC);
	if (data == NULL)
		return -ENOMEM;

	data_core = &data->data_core;
	data->spi = spi;
	memset(data->pad_buf, 0, sizeof(data->pad_buf));
	axiom_init_data_core(data_core, &spi->dev, data, &axiom_read_usage, &axiom_write_usage);

	spi_set_drvdata(spi, data);

	axiom_discover(data_core);
	axiom_rebaseline(data_core);

	// Now Register with the Input Sub-System
	//-------------------------------------------------
	data_core->input_dev = axiom_register_input_subsystem(poll_enable, poll_interval);
	if (data_core->input_dev == NULL) {
		dev_err(&spi->dev, "Failed to register input device, error: %d\n", error);
		return error;
	}

	input_set_drvdata(data_core->input_dev, data_core);

	dev_info(&spi->dev, "AXIOM: SPI driver registered with Input Sub-System.\n");
	//-------------------------------------------------

	// Ensure that all reports are initialised to not be present.
	for (target = 0; target < U41_MAX_TARGETS; target++)
		data_core->targets[target].state = Target_State_Not_Present;

	if (poll_enable == 0) {
		// Delay just a smidge before enabling the IRQ
		udelay(data_core->bus_holdoff_delay_us);
		data->irq_allocated = (0 == (error = devm_request_threaded_irq(&spi->dev, spi->irq,
											NULL, axiom_irq,
											IRQF_TRIGGER_LOW | IRQF_ONESHOT,
											"axiom_irq", data)));
		if (error != 0) {
			dev_err(&spi->dev, "Failed to request IRQ %u (error: %d)\n", spi->irq, error);
			return error;
		}
	}
	dev_info(&spi->dev, "Probe End\n");

	return 0;
}

// purpose: Clean-up when device is disconnected
#if(LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0))
static int axiom_spi_remove(struct spi_device *spi)
#else
static void axiom_spi_remove(struct spi_device *spi)
#endif
{
	struct axiom_data *data;
	struct axiom_data_core *data_core;

	data = spi_get_drvdata(spi);
	data_core = &data->data_core;

	if (data->irq_allocated)  {
		dev_info(&spi->dev, "freeing IRQ %u...\n", spi->irq);
		devm_free_irq(&spi->dev, spi->irq, data);
		data->irq_allocated = false;
	}

	axiom_remove(data_core);

	dev_info(&spi->dev, "Removed\n");

#if(LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0))
	return 0;
#endif
}

static const struct spi_device_id axiom_spi_id_table[] = {
	{ "axiom" },
	{ },
};
MODULE_DEVICE_TABLE(spi, axiom_spi_id_table);

static const struct of_device_id axiom_spi_dt_ids[] = {
	{
		.compatible = "axiom_spi,axiom",
		.data = "axiom",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, axiom_spi_dt_ids);

static struct spi_driver axiom_spi_driver = {
	.id_table = axiom_spi_id_table,
	.driver = {
		.name = "axiom_spi",
		.of_match_table = of_match_ptr(axiom_spi_dt_ids),
	},
	.probe = axiom_spi_probe,
	.remove = axiom_spi_remove,
};

module_spi_driver(axiom_spi_driver);

MODULE_AUTHOR("TouchNetix <support@touchnetix.com>");
MODULE_DESCRIPTION("aXiom touchscreen SPI bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("axiom");
MODULE_VERSION("1.0.0");
