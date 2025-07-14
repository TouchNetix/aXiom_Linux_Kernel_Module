// SPDX-License-Identifier: GPL-2.0
/*
 * TouchNetix aXiom Touchscreen Driver
 *
 * Copyright (C) 2020-2023 TouchNetix Ltd.
 *
 * Author(s): Bart Prescott <bartp@baasheep.co.uk>
 *            Pedro Torruella <pedro.torruella@touchnetix.com>
 *            Mark Satterthwaite <mark.satterthwaite@touchnetix.com>
 *            Hannah Rossiter <hannah.rossiter@touchnetix.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include "axiom_core.h"

static bool poll_enable;
module_param(poll_enable, bool, 0444);
MODULE_PARM_DESC(poll_enable, "Enable polling mode [default 0=no]");

static int poll_interval;
module_param(poll_interval, int, 0444);
MODULE_PARM_DESC(poll_interval, "Polling period in ms [default = 100]");

struct axiom_data {
	struct axiom_data_core data_core;
	struct i2c_client *i2cClient;
	bool irq_allocated; // indicates the IRQ was allocated during probe
};

void axiom_reset(struct axiom_data_core *data_core)
{
	gpiod_set_value_cansleep(data_core->rst_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(data_core->rst_gpio, 0);
	msleep(110);
}
EXPORT_SYMBOL_GPL(axiom_reset);

// purpose: Helper function to read a specified usage and write it into the provided buffer
// returns: Length of the usage read
static u16 axiom_read_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
	u16 target_address = usage_to_target_address(&((struct axiom_data*)pAxiomData)->data_core, usage, page, 0);
	return axiom_read_page(pAxiomData, target_address,length,pBuffer);
}

u16 axiom_read_page(void *pAxiomData, u16 target_address, u16 length, u8 *pBuffer)
{
	struct axiom_data *data = pAxiomData;
	struct i2c_client *i2cClient = data->i2cClient;
	struct device *pDev = data->data_core.pDev;
	struct i2c_msg msg[2];
	struct AxiomCmdHeader cmdHeader;
	int ret, i;

	// Build the header
	cmdHeader.target_address = target_address;
	cmdHeader.length = length;
	cmdHeader.read = 1;

	msg[0].addr = i2cClient->addr;
	msg[0].flags = 0; // (odd that the I2C_M_WR flag is not defined in i2c.h)
	msg[0].len = sizeof(cmdHeader);
	msg[0].buf = (u8 *)&cmdHeader;
	msg[1].addr = i2cClient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = (char *)pBuffer;

	for (i = 0; i < 8; i++) {
		ret = i2c_transfer(i2cClient->adapter, msg, 2);
		if (ret == 2)
			return length;

		dev_err(pDev, "Failed I2C read page. RC:%d\n", ret);
		udelay(data->data_core.bus_holdoff_delay_us);
	}

	//dev_dbg(pDev, "Payload Data %*ph\n", length, *pBuffer);
	udelay(data->data_core.bus_holdoff_delay_us);
	return length;
}
EXPORT_SYMBOL_GPL(axiom_read_page);

// purpose: Helper function to write data in a provided buffer to a specified usage
// returns: Length of the data to write
static u16 axiom_write_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
	u16 target_address = usage_to_target_address(&((struct axiom_data*)pAxiomData)->data_core, usage, page, 0);
	return axiom_write_page(pAxiomData, target_address,length,pBuffer);
}

u16 axiom_write_page(void *pAxiomData, u16 target_address, u16 length, u8 *pBuffer)
{
	struct axiom_data *data = pAxiomData;
	struct i2c_client *i2cClient = data->i2cClient;
	struct device *pDev = data->data_core.pDev;
	struct i2c_msg msg[1];
	int ret, i;
	u8 write_buf[786];

	write_buf[0] = (target_address & 0x00FF);
	write_buf[1] = (target_address & 0xFF00) >> 8;
	write_buf[2] = (length & 0x00FF);
	write_buf[3] = (length & 0x7F00) >> 8; // Ensure the read bit is clear

	for (i = 0; i < length; i++)
		write_buf[4 + i] = pBuffer[i];

	msg[0].addr = i2cClient->addr;
	msg[0].flags = 0;
	msg[0].len = (4 + length);
	msg[0].buf = (u8 *)&write_buf;

	for (i = 0; i < 8; i++) {
		ret = i2c_transfer(i2cClient->adapter, msg, 1);
		if (ret == 1)
			return length;
		dev_err(pDev, "Failed I2C write page. RC:%d\n", ret);
		udelay(data->data_core.bus_holdoff_delay_us);
	}

	udelay(data->data_core.bus_holdoff_delay_us);
	return length;
}
EXPORT_SYMBOL_GPL(axiom_write_page);

// purpose: Process the interrupt notifying the system a new report is available
// returns: Value to indicate the interrupt has been handled
static irqreturn_t axiom_irq(int irq, void *handle)
{
	struct axiom_data *data = handle;
	struct axiom_data_core *data_core = &data->data_core;
	u8 *pRX_data = &data_core->rx_buf[0];

	(*data_core->pAxiomReadUsage)(data_core->pAxiomData, 0x34, 0, data_core->max_report_len, pRX_data);
	axiom_process_report(&data->data_core, pRX_data);

	return IRQ_HANDLED;
}

// purpose: Function called in IRQ context when device is plugged in.
// returns: Error code
#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int axiom_i2c_probe(struct i2c_client *i2cClient)
#else
static int axiom_i2c_probe(struct i2c_client *i2cClient, const struct i2c_device_id *id)
#endif
{
#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
	const struct i2c_device_id *id __maybe_unused = i2c_client_get_device_id(i2cClient);
#endif
	struct device *pDev = &i2cClient->dev;
	struct axiom_data *data;
	struct axiom_data_core *data_core;
	u32 error = 0;
	u32 target;
	u32 i2cFunctionality;
	bool ret;

	dev_info(pDev, "aXiom Probe\n");
	dev_info(pDev, "Device IRQ: %u\n", i2cClient->irq);
	dev_info(pDev, "Device address: 0x%04x\n", i2cClient->addr);

	i2cFunctionality = i2c_get_functionality(i2cClient->adapter);
	dev_info(pDev, "The i2c adapter reported functionality: 0x%08x\n", i2cFunctionality);

	if ((i2cClient->irq == 0) &&
			(poll_enable == 0)) {
		dev_err(pDev, "No IRQ specified!\n");
		return -EINVAL;
	}

	// Kernel will manage this data, it will be automatically unloaded when the
	// module is unloaded.
	data = devm_kzalloc(pDev, sizeof(*data), GFP_ATOMIC);
	if (data == NULL)
		return -ENOMEM;

	data_core = &data->data_core;
	data->i2cClient = i2cClient;
	axiom_init_data_core(data_core, pDev, data, &axiom_read_usage, &axiom_write_usage);

	i2c_set_clientdata(i2cClient, data);

	axiom_discover(data_core);
	axiom_rebaseline(data_core);
	ret = axiom_dev_init(data_core);
	if (ret)
		dev_info(pDev, "axiom dev node creation failed");


	// Now Register with the Input Sub-System
	//-------------------------------------------------
	data_core->input_dev = axiom_register_input_subsystem(poll_enable, poll_interval);
	if (data_core->input_dev == NULL) {
		dev_err(pDev, "Failed to register input device, error: %d\n", error);
		return error;
	}

	input_set_drvdata(data_core->input_dev, data_core);

	dev_info(pDev, "AXIOM: I2C driver registered with Input Sub-System.\n");
	//-------------------------------------------------

	// Delay just a smidge before enabling the IRQ
	udelay(data_core->bus_holdoff_delay_us);

	// Ensure that all reports are initialised to not be present.
	for (target = 0; target < U41_MAX_TARGETS; target++)
		data_core->targets[target].state = Target_State_Not_Present;

	if (poll_enable == 0) {
		data->irq_allocated = (0 == (error = devm_request_threaded_irq(pDev, i2cClient->irq,
											NULL, axiom_irq,
											IRQF_TRIGGER_LOW | IRQF_ONESHOT,
											"axiom_irq", data)));
		if (error != 0) {
			dev_err(pDev, "Failed to request IRQ %u (error: %d)\n", i2cClient->irq, error);
			return error;
		}
		data_core->irq_line = i2cClient->irq;
	}
	dev_info(pDev, "Reseting axiom in probe");
	axiom_reset(data_core);
	dev_info(pDev, "Probe End\n");

	return 0;
}

// purpose: Clean-up when device is disconnected
#if (KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE)
static int axiom_i2c_remove(struct i2c_client *i2cClient)
#else
static void axiom_i2c_remove(struct i2c_client *i2cClient)
#endif
{
	struct axiom_data *data;
	struct axiom_data_core *data_core;

	data = i2c_get_clientdata(i2cClient);
	data_core = &data->data_core;
	axiom_dev_exit(data_core);

	if (data->irq_allocated) {
		dev_info(&i2cClient->dev, "freeing IRQ %u...\n", i2cClient->irq);
		devm_free_irq(&i2cClient->dev, i2cClient->irq, data);
		data->irq_allocated = false;
	}

	axiom_remove(data_core);

	dev_info(&i2cClient->dev, "Removed\n");

#if (KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE)
	return 0;
#endif
}

static const struct i2c_device_id axiom_i2c_id_table[] = {
	{ "axiom" },
	{ },
};
MODULE_DEVICE_TABLE(i2c, axiom_i2c_id_table);

static const struct of_device_id axiom_i2c_dt_ids[] = {
	{
		.compatible = "axiom_i2c,axiom",
		.data = "axiom",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, axiom_i2c_dt_ids);

static struct i2c_driver axiom_i2c_driver = {
	.driver = {
		.name = "axiom_i2c",
		.of_match_table = of_match_ptr(axiom_i2c_dt_ids),
	},
	.id_table = axiom_i2c_id_table,
	.probe = axiom_i2c_probe,
	.remove = axiom_i2c_remove,
};

module_i2c_driver(axiom_i2c_driver);

MODULE_AUTHOR("TouchNetix <support@touchnetix.com>");
MODULE_DESCRIPTION("aXiom touchscreen I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("axiom");
MODULE_VERSION("1.0.0");
