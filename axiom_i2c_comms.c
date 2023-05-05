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

//#define DEBUG   // Enable debug messages

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
#include <linux/i2c.h>
#include <linux/string.h>
#include "axiom_core.h"

struct axiom_data
{
    struct axiom_data_core data_core;

    // I2C client data
    struct i2c_client *i2cClient;
    bool irq_allocated; // indicates the IRQ was allocated during probe
};

// purpose: Helper function to read a specified usage and write it into the provided buffer
// returns: Length of the usage read
static u16 axiom_read_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
    struct axiom_data *data = pAxiomData;
    struct i2c_client *i2cClient = data->i2cClient;
    struct device *pDev = data->data_core.pDev;
	int ret;
	struct i2c_msg msg[2];
    AxiomCmdHeader cmdHeader;
    
    //dev_dbg(pDev,"axiom_read_usage(usage %u, page %u, length %u)", usage,  page, length);

    // Build the header
    cmdHeader.target_address = usage_to_target_address(&data->data_core, usage, page, 0);
    cmdHeader.length = length;
    cmdHeader.read = 1;
    //dev_dbg(pDev, "cmdHeader %*ph\n", sizeof(cmdHeader), &cmdHeader);

	msg[0].addr = i2cClient->addr;
	msg[0].flags = 0; // (odd that the I2C_M_WR flag is not defined in i2c.h)
	msg[0].len = sizeof(cmdHeader);
	msg[0].buf = (u8 *)&cmdHeader;
	msg[1].addr = i2cClient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = (char *)pBuffer;

	ret = i2c_transfer(i2cClient->adapter, msg, 2);
    if(ret != 2)
    {
        dev_err(pDev, "Failed I2C read transfer. RC:%d\n", ret);
        return 0;
    }
    
    //dev_dbg(pDev, "Payload Data %*ph\n", length, *pBuffer);
    udelay(data->data_core.bus_holdoff_delay_us);
    return length;
}

// purpose: Helper function to write data in a provided buffer to a specified usage
// returns: Length of the data to write
static u16 axiom_write_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
    struct axiom_data *data = pAxiomData;
    struct i2c_client *i2cClient = data->i2cClient;
    struct device *pDev = data->data_core.pDev;
	int ret;
	struct i2c_msg msg[2];
    AxiomCmdHeader cmdHeader;

    //dev_dbg(pDev,"axiom_write_usage(usage %u, page %u, length %u)", usage,  page, length);

    cmdHeader.target_address = usage_to_target_address(&data->data_core, usage, page, 0);
    cmdHeader.length = length;
    cmdHeader.read = 0;

    //dev_dbg(pDev, "cmdHeader %*ph\n", sizeof(cmdHeader), &cmdHeader);

	msg[0].addr = i2cClient->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(cmdHeader);
	msg[0].buf = (u8 *)&cmdHeader;
	msg[1].addr = i2cClient->addr;
	msg[1].flags = 0;
	msg[1].len = length;
	msg[1].buf = (char *)pBuffer;

	ret = i2c_transfer(i2cClient->adapter, msg, 2);
    if(ret != 2)
    {
        dev_err(pDev, "Failed I2C write transfer. RC:%d\n", ret);
        return 0;
    }
    
    //dev_dbg(pDev, "Payload Data %*ph\n", length, pBuffer);
    udelay(data->data_core.bus_holdoff_delay_us);
    return length;
}

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
static int axiom_i2c_probe(struct i2c_client *i2cClient, const struct i2c_device_id *id)
{
    struct device *pDev = &i2cClient->dev;
    struct axiom_data *data;
    struct axiom_data_core *data_core;
    u32 error;
    u32 target;
    u32 i2cFunctionality;

    dev_info(pDev, "axiom_i2c_probe\n");
    dev_info(pDev, "Device IRQ: %u\n", i2cClient->irq);
    dev_info(pDev, "Device address: 0x%04x\n", i2cClient->addr);
    dev_info(pDev, "Device flags: 0x%04x\n", i2cClient->flags);
    dev_info(pDev, "Device name: %s\n", i2cClient->name);

    i2cFunctionality = i2c_get_functionality(i2cClient->adapter);
    dev_info(pDev, "The i2c adapter reported functionality: 0x%08x\n", i2cFunctionality);

    if (i2cClient->irq == 0)
    {
        dev_err(pDev, "No IRQ specified!\n");
        return -EINVAL;
    }
    // Kernel will manage this data, it will be automatically unloaded when the
    // module is unloaded.
    data = devm_kzalloc(pDev, sizeof(*data), GFP_ATOMIC);
    if (data == NULL)
    {
        dev_err(pDev, "Failed to allocate memory for aXiom data structure!\n");
        return -ENOMEM;
    }

    data_core = &data->data_core;
    data->i2cClient = i2cClient;
    axiom_init_data_core(data_core, pDev, data, &axiom_read_usage, &axiom_write_usage);

    i2c_set_clientdata(i2cClient, data);

    axiom_discover(data_core);
    axiom_rebaseline(data_core);

    // Now Register with the Input Sub-System
    //-------------------------------------------------
    data_core->input_dev = axiom_register_input_subsystem();
    if (data_core->input_dev == NULL)
    {
        dev_err(pDev, "Failed to register input device, error: %d\n", error);
        return error;
    }
    dev_info(pDev, "AXIOM: I2C driver registered with Input Sub-System.\n");
    //-------------------------------------------------

    // Delay just a smidge before enabling the IRQ
    udelay(data_core->bus_holdoff_delay_us);

    // Ensure that all reports are initialised to not be present.
    for (target = 0; target < U41_MAX_TARGETS; target++)
    {
        data_core->targets[target].state = Target_State_Not_Present;
    }

    data->irq_allocated = (0 == (error = devm_request_threaded_irq(pDev, i2cClient->irq,
                                      NULL, axiom_irq,
                                      IRQF_TRIGGER_LOW | IRQF_ONESHOT,
                                      "axiom_irq", data)));
    if (error != 0)
    {
        dev_err(pDev, "Failed to request IRQ %u (error: %d)\n", i2cClient->irq, error);
        return error;
    }

    dev_info(pDev, "Probe End\n");

    return 0;
}

// purpose: Clean-up when device is disconnected
static int axiom_i2c_remove(struct i2c_client *i2cClient)
{
    struct axiom_data *data;
    struct axiom_data_core *data_core;

    dev_info(&i2cClient->dev, "axiom_i2c_remove\n");

    data = i2c_get_clientdata(i2cClient);
    data_core = &data->data_core;

    if (data->irq_allocated) 
    {
        dev_info(&i2cClient->dev, "freeing IRQ %u...\n", i2cClient->irq);
        devm_free_irq(&i2cClient->dev, i2cClient->irq, data);
        data->irq_allocated = false;
    }

    axiom_remove(data_core);
    
    dev_info(&i2cClient->dev, "Removed\n");

    return 0;
}

static const struct i2c_device_id axiom_i2c_id_table[] = {
	{ "axiom" },
	{ },
};
MODULE_DEVICE_TABLE(i2c, axiom_i2c_id_table);

static const struct of_device_id axiom_i2c_dt_ids[] =
{
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
    //.shutdown = axiom_i2c_shutdown,
    //.suspend = axiom_i2c_suspend,
    //.resume = axiom_i2c_resume
};

module_i2c_driver(axiom_i2c_driver);

MODULE_AUTHOR("TouchNetix <support@touchnetix.com>");
MODULE_DESCRIPTION("aXiom touchscreen I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("axiom");
MODULE_VERSION("1.0.0");
