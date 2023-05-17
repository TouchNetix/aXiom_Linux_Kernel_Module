/*
 * TouchNetix aXiom Touchscreen Driver
 *
 * Copyright (C) 2020-2023 TouchNetix Ltd.
 *
 * Author(s): Mark Satterthwaite <mark.satterthwaite@touchnetix.com>
 *            Pedro Torruella <pedro.torruella@touchnetix.com>
 *            Bart Prescott <bartp@baasheep.co.uk>
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/crc16.h>
#include <linux/slab.h>

#include "axiom_core.h"

// purpose: Decodes and populates the local u31 structure.
//          Given a buffer of data read from page 0 of u31 in an aXiom
//          device.
void axiom_get_dev_info(struct axiom_data_core *data_core, u8 *data)
{
    struct u31_DeviceInfo *pU31_info;

    if (data_core == NULL)
        return;
    pU31_info = &data_core->u31_Info;
    if (pU31_info != NULL)
    {
        pU31_info->bootloader_mode = ((data[1] & 0x80) != 0) ? 1 : 0;
        pU31_info->device_id = ((data[1] & 0x7F) << 8) | data[0];
        pU31_info->fw_minor = data[2];
        pU31_info->fw_major = data[3];
        pU31_info->fw_info_extra = (data[4]) | (data[5] << 8);
        pU31_info->bootloader_fw_ver_minor = data[6];
        pU31_info->bootloader_fw_ver_major = data[7];
        pU31_info->jedec_id = (data[8]) | (data[9] << 8);
        pU31_info->num_usages = data[10];
        pU31_info->silicon_revision = data[11];
    }
}


// purpose: Decodes and populates the local Usage Table.
//          Given a buffer of data read from page 1 onwards of u31 from
//          an aXiom device.
//
//          See the relevant aXiom's Touch Controller Programmers guide
//          for information on what a usage table is.
//
// returns: u8 with the maximum report length, across all reports for this device.
u8 axiom_populate_usage_table(struct axiom_data_core *data_core, u8 *pRX_data)
{
    u32 usage_id = 0;
    u8 max_report_len = 0;
    struct u31_DeviceInfo *pU31_info;
    struct usage_Entry *pUsage_Table;

    if (data_core == NULL)
        return 0;
    pU31_info = &data_core->u31_Info;
    pUsage_Table = data_core->usage_table;

    // Decode the usage table
    for (usage_id = 0; usage_id < pU31_info->num_usages; usage_id++)
    {
        u16 offset = (usage_id * U31_BYTES_PER_USAGE);
        u8 id = pRX_data[offset + 0];
        u8 start_page = pRX_data[offset + 1];
        u8 num_pages = pRX_data[offset + 2];
        // Convert words to bytes
        u8 max_offset = ((pRX_data[offset + 3] & 0x7F) + 1) * 2;


        // Store the entry into the usage table
        pUsage_Table[usage_id].id = id;
        pUsage_Table[usage_id].is_report = ((num_pages == 0) ? 1 : 0);
        pUsage_Table[usage_id].start_page = start_page;
        pUsage_Table[usage_id].num_pages = num_pages;


        dev_info(data_core->pDev, "Usage %2u Info: %*ph\n", usage_id, U31_BYTES_PER_USAGE,
                &pRX_data[offset]);


        // Identify the max report length the module will receive
        if ((pUsage_Table[usage_id].is_report) && (max_offset > max_report_len))
        {
            max_report_len = max_offset;
        }
    }
    data_core->usage_table_populated = true;

    return max_report_len;
}


// purpose: Helper function to translate usage/page/offset
//          triplet into physical address.
//
// returns: u16 with physical address to required register
u16 usage_to_target_address(struct axiom_data_core *data_core,
                            u8 usage,
                            u8 page,
                            u8 offset)
{
    u16 target_address = 0;
    u32 usage_index;
    struct u31_DeviceInfo *pU31_info;
    struct usage_Entry *pUsage_Table;

    if (data_core == NULL)
        return 0;
    pU31_info = &data_core->u31_Info;
    pUsage_Table = data_core->usage_table;

    // At the moment the convention is that u31 is always at
    // physical address 0x0.
    if (!data_core->usage_table_populated && (usage == 0x31))
    {
        target_address = ((page << 8) + offset);
    }
    else if (data_core->usage_table_populated)
    {
        for (usage_index = 0; usage_index < pU31_info->num_usages; usage_index++)
        {
            if (pUsage_Table[usage_index].id == usage)
            {
                if (page < pUsage_Table[usage_index].num_pages)
                {
                    target_address = ((pUsage_Table[usage_index].start_page + page) << 8) + offset;
                }
                else
                {
                    target_address = 0xFFFF;
                    pr_err("ERROR: aXiom-core: Invalid usage table! usage: %u, page: %u, offset: %u\n", usage, page, offset);
                }
                break;
            }
        }
    }
    else
    {
        target_address = 0xFFFF;
        pr_err("ERROR: aXiom-core: Unpopulated usage table for usage: %u\n", usage);
    }

    pr_debug("aXiom-core: target_address is 0x%04x for usage: %u page %u\n", target_address, usage, page);
    return target_address;
}

// purpose: Function to retrieve, store, and print the connected device information
// returns: True/false depending on success of the process
bool axiom_discover(struct axiom_data_core *data_core)
{
    u8 *pRX_data=&data_core->rx_buf[0];
    struct device *pDev = data_core->pDev;

    // First the first page of u31 to get the device infromation and the number of usages
    if (0 == (*data_core->pAxiomReadUsage)(data_core->pAxiomData, 0x31, 0, AX_U31_PAGE0_LENGTH, pRX_data))
    {
        dev_err(pDev, "Failed axiom_discover\n");
        return false;
    }

    axiom_get_dev_info(data_core, pRX_data);

    dev_info(pDev, "Data Decode:\n");
    dev_info(pDev, "  Bootloader Mode: %u\n", data_core->u31_Info.bootloader_mode);
    dev_info(pDev, "  Device ID      : %04x\n", data_core->u31_Info.device_id);
    dev_info(pDev, "  Firmware Rev   : %02x.%02x\n",
            data_core->u31_Info.fw_major, data_core->u31_Info.fw_minor);
    dev_info(pDev, "  Bootloader Rev : %02x.%02x\n",
            data_core->u31_Info.bootloader_fw_ver_major, data_core->u31_Info.bootloader_fw_ver_minor);
    dev_info(pDev, "  FW Extra Info  : %04x\n", data_core->u31_Info.fw_info_extra);
    dev_info(pDev, "  Silicon        : %02x\n", data_core->u31_Info.jedec_id);
    dev_info(pDev, "  Num Usages     : %04x\n", data_core->u31_Info.num_usages);

    // Read the second page of u31 to get the usage table
    if (0 == (*data_core->pAxiomReadUsage)(data_core->pAxiomData, 0x31, 1, (U31_BYTES_PER_USAGE * data_core->u31_Info.num_usages), pRX_data))
    {
        dev_err(pDev, "Failed axiom_discover\n");
        return false;
    }
    data_core->max_report_len = axiom_populate_usage_table(data_core, pRX_data);
    dev_info(pDev, "Max Report Length: %u\n", data_core->max_report_len);
    return true;
}

// purpose: Helper function to rebaseline the touchscreen, effectively zero-ing it
void axiom_rebaseline(struct axiom_data_core *data_core)
{
    struct device *pDev = data_core->pDev;
    u8 buffer[8] = {0};

    memset(buffer, 0, sizeof(buffer));

    buffer[0] = 0x03;   // Rebaseline request

    if (sizeof(buffer) == (*data_core->pAxiomWriteUsage)(data_core->pAxiomData, 0x02, 0, sizeof(buffer), buffer))
        dev_info(pDev, "Capture Baseline Requested\n");
    else
        dev_err(pDev, "Rebaseline failed\n");
}


// purpose: Allocate the resources for the data core and give them all default values.
void axiom_init_data_core(struct axiom_data_core *data_core, struct device *pDev, void *pAxiomData, void *pAxiomReadUsage, void *pAxiomWriteUsage)
{
    memset(&data_core->targets, 0, sizeof(data_core->targets));
    memset(&data_core->u31_Info, 0, sizeof(data_core->u31_Info));
    memset(data_core->rx_buf, 0, sizeof(data_core->rx_buf));
    data_core->usage_table_populated = false;
    data_core->max_report_len = 0;
    data_core->input_dev = NULL;
    data_core->pDev = pDev;
    data_core->bus_holdoff_delay_us = 250;
    data_core->report_counter = 0;
    data_core->report_overflow_counter = 0;
    data_core->pAxiomData = pAxiomData;
    data_core->pAxiomReadUsage = pAxiomReadUsage;
    data_core->pAxiomWriteUsage = pAxiomWriteUsage;
}

// purpose: Free up any resources allocated during discover etc.
void axiom_remove(struct axiom_data_core *data_core)
{
    struct device *pDev = data_core->pDev;

    if (data_core->usage_table)
    {
        dev_info(pDev, "freeing usage table...\n");
        data_core->usage_table_populated = false;
    }

    if (data_core->input_dev)
    {
        dev_info(pDev, "unregistering input device...\n");
        input_unregister_device(data_core->input_dev);
        data_core->input_dev = NULL;
    }
}

// purpose: Support function to axiom_process_report.
//          It validates the crc and multiplexes the axiom reports
//          to the appropriate report handler
void axiom_process_report(struct axiom_data_core *data_core, u8 *pReport)
{
    u8 len;
    u16 crc_calc;
    u16 crc_report;
    u8 usage = pReport[1];
    struct device *pDev = data_core->pDev;

    if ((pReport[0] & COMMS_OVERFLOW_MSK) != 0)
    {
        data_core->report_overflow_counter++;
        if (data_core->report_overflow_counter > 10)
        {
            //it's normal to get a couple of overflows as the driver starts up
            dev_err(pDev, "Report overflow bit set. Counter:%u.\n", data_core->report_overflow_counter);
        }
    }

    len = (pReport[0] & COMMS_REPORT_LEN_MSK) << 1;
    if (len == 0)
    {
        dev_err(pDev, "Zero length report discarded.\n");
        return;
    }

    dev_dbg(pDev, "Payload Data %*ph\n", len, pReport);

    // Validate the report CRC
    crc_report = (pReport[len - 1] << 8) | (pReport[len - 2]);
    crc_calc = crc16(0, pReport, (len - 2) ); // Length is in 16 bit words and remove the size of the CRC16 itself
    if(crc_calc != crc_report)
    {
        dev_err(pDev, "CRC mismatch! Expected: %04X, Calculated CRC: %04X. Report discarded.\n", crc_report, crc_calc);
        return;
    }

    switch (usage)
    {
        case USAGE_2DCTS_REPORT_ID:
            dev_info(pDev, "aXiom-rep: Got touch report! (%d bytes)\n", len);
            axiom_process_u41_report(&pReport[1], data_core);
            break;
        case USAGE_2AUX_REPORT_ID: // This is an aux report (force)
            dev_info(pDev, "aXiom-rep: Got aux force report! (%d bytes)\n", len);
#ifdef U46_ENABLE_RAW_FORCE_DATA
            axiom_process_u46_report(&pReport[1], data_core);
#endif
            break;
        case USAGE_2HB_REPORT_ID:
            // This is a heartbeat report
            dev_info(pDev, "aXiom-rep: Got Heartbeat report! (%d bytes)\n", len);
            break;
        default:
            dev_info(pDev, "aXiom-rep: got other report 0x%x (%d bytes)\n", usage, len);
            dev_dbg(pDev, "BUF: %*ph", len, pReport);
            // Don't care or are not handled processed, discard
            break;
    }
    data_core->report_counter++;
}

// purpose: Support function to axiom_process_u41_report.
//          It generates input-subsystem events for every target.
//
// returns: Bools, if true, after calling this function the caller shall
//          issue a Sync to the input sub-system.
bool axiom_process_u41_report_target(struct axiom_data_core *data_core,
                                     struct u41_Target_Report *pTarget)
{
    struct input_dev *input_dev;
    struct u41_Target *pTargetPrevState;
    enum u41_Target_State_e currentState;
    bool update = false;
    int slot;

    if (data_core == NULL)
        return false;
    input_dev = data_core->input_dev;

    // Verify the target index
    if (pTarget->index >= U41_MAX_TARGETS)
    {
        pr_debug("Invalid target index! %u\n", pTarget->index);
        return false;
    }

    // Grab a pointer to an element in the target status array, this makes the
    // code slightly more readable and easier to use.
    pTargetPrevState = &data_core->targets[pTarget->index];

    currentState = ((pTarget->present == 0)                         ? Target_State_Not_Present
                    : (pTarget->z >= 0)                             ? Target_State_Touching
                    : (pTarget->z > PROX_LEVEL) && (pTarget->z < 0) ? Target_State_Hover
                    : (pTarget->z == PROX_LEVEL)                    ? Target_State_Prox
                                                                    : Target_State_Not_Present);
    if (pTargetPrevState->state == currentState
        && pTargetPrevState->x == pTarget->x
        && pTargetPrevState->y == pTarget->y
        && pTargetPrevState->z == pTarget->z)
    {
        //There was no change to the previous reported state, so do nothing...
        return false;
    }

#ifdef AXIOM_USE_KERNEL_SLOT_ASSIGNMENT
    slot = input_mt_get_slot_by_key(input_dev, pTarget->index);
    if (slot < 0)
        slot = pTarget->index;
#else
    slot = pTarget->index;
#endif
    pr_debug("U41 Target T%u, slot:%u present:%u, x:%u, y:%u, z:%d\n",
        pTarget->index, slot, pTarget->present,
        pTarget->x, pTarget->y, pTarget->z);
    switch (currentState)
    {
        default:
        case Target_State_Not_Present:
        case Target_State_Prox:
        {
            if (pTargetPrevState->insert)
            {
                update = true;
                pTargetPrevState->insert = false;
                input_mt_slot(input_dev, slot);
                if (slot == 0) input_report_key(input_dev, BTN_LEFT, 0);
                input_mt_report_slot_inactive(input_dev);
                //make sure the previous coordinates are all off screen when the finger comes back...
                pTarget->x = pTarget->y = 65535;
                pTarget->z = PROX_LEVEL;
            }
            break;
        }
        case Target_State_Hover:
        case Target_State_Touching:
        {
            pTargetPrevState->insert = true;
            update = true;
            input_mt_slot(input_dev, slot);
#ifdef AXIOM_USE_KERNEL_SLOT_ASSIGNMENT
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
#else
            input_report_abs(input_dev, ABS_MT_TRACKING_ID, slot);
#endif
            input_report_abs(input_dev, ABS_MT_POSITION_X, pTarget->x);
            input_report_abs(input_dev, ABS_X, pTarget->x);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, pTarget->y);
            input_report_abs(input_dev, ABS_Y, pTarget->y);

            if (currentState == Target_State_Touching)
            {
                input_report_abs(input_dev, ABS_MT_DISTANCE, 0);
                input_report_abs(input_dev, ABS_DISTANCE, 0);
                input_report_abs(input_dev, ABS_MT_PRESSURE, pTarget->z);
                input_report_abs(input_dev, ABS_PRESSURE, pTarget->z);
            }
            else //(currentState == Target_State_Hover)
            {
                input_report_abs(input_dev, ABS_MT_DISTANCE, -pTarget->z);
                input_report_abs(input_dev, ABS_DISTANCE, -pTarget->z);
                input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
                input_report_abs(input_dev, ABS_PRESSURE, 0);
            }

            if (slot == 0) input_report_key(input_dev, BTN_LEFT, (currentState == Target_State_Touching));

            break;
        }
    }

    pTargetPrevState->state = currentState;
    pTargetPrevState->x = pTarget->x;
    pTargetPrevState->y = pTarget->y;
    pTargetPrevState->z = pTarget->z;
    if (update) input_mt_sync_frame(input_dev);
    return update;
}

// purpose: To take a raw buffer with u41 report data and decode it.
//          also generate input events if needed.
//
//          rx_buf : ptr to a u8 array
//                   [0]: Usage number
//                   [1]: Status LSB
//                   [2]: Status MSB
void axiom_process_u41_report(u8 *rx_buf, struct axiom_data_core *data_core)
{
    u16 target_status = 0;
    u32 i = 0;
    bool update_done = false;
    struct input_dev *input_dev;
    struct u41_Target_Report target;

    if (data_core == NULL)
        return;

    input_dev = data_core->input_dev;

    if (rx_buf[0] != 0x41)
    {
        pr_err("ERROR: aXiom-core: data in buffer does not have expected u41 format.\n");
        return;
    }

    target_status = ((rx_buf[1]) | (rx_buf[2] << 8));

    // For details on this decoding, consult the appropriate
    // aXiom's Programmers Guide and Touch Controller Protocol Usage Definitions.
    for (i = 0; i < U41_MAX_TARGETS; i++)
    {
        target.index = i;
        target.present = ((target_status & (1 << i)) != 0) ? 1 : 0;
        target.x = (rx_buf[(i * 4) + 3]) | (rx_buf[(i * 4) + 4] << 8);
        target.y = (rx_buf[(i * 4) + 5]) | (rx_buf[(i * 4) + 6] << 8);
        target.z = (s8)(rx_buf[i + 43]);
        update_done |= axiom_process_u41_report_target(data_core, &target);
    }

    if (update_done)
    {
        input_sync(input_dev);
    }
}


#ifdef U46_ENABLE_RAW_FORCE_DATA
// purpose: To take a raw buffer with u46 report data and decode it.
//          Also generates input events if needed.
void axiom_process_u46_report(u8 *rx_buf, struct axiom_data_core *data_core)
{
    u16 aux_value;
    u32 i = 0;
    u32 event_value;
    struct input_dev *input_dev;

    if (data_core == NULL)
        return;
    input_dev = data_core->input_dev;

    for (i = 0; i < U46_AUX_CHANNELS; i++)
    {
        aux_value = ((rx_buf[(i * 2) + 2] << 8) | (rx_buf[(i * 2) + 1])) & U46_AUX_MASK;
        event_value = (i << 16) | (aux_value);
        // Create the event.
        input_event(input_dev, EV_MSC, MSC_RAW, event_value);
    }

    input_mt_sync(input_dev);
    input_sync(input_dev);
}
#endif


// purpose: To register an aXiom based driver with the Input Sub-System.
// returns: Pointer to the register input device.
struct input_dev *axiom_register_input_subsystem(void)
{
    struct input_dev *input_dev = input_allocate_device();
    int ret = 0;
    if (input_dev == NULL)
    {
        pr_err("ERROR: aXiom-core: Failed to allocate memory for input device!\n");
        return NULL;
    }

    input_dev->name = "TouchNetix aXiom Touchscreen";
    input_dev->phys = "input/axiom_ts";

    // Single Touch
    input_set_abs_params(input_dev, ABS_X, 0, 65535, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, 0, 65535, 0, 0);

    // Multi Touch
    // Min, Max, Fuzz (expected noise in px, try 4?) and Flat
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 65535, 0, 0);
    // Min, Max, Fuzz (expected noise in px, try 4?) and Flat
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 65535, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_DISTANCE, 0, 127, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 127, 0, 0);

#ifdef AXIOM_USE_TOUCHSCREEN_INTERFACE
    input_mt_init_slots(input_dev, U41_MAX_TARGETS, INPUT_MT_DIRECT);
#else //TABLET_INTERFACE (emulates mouse pointer as expected)
    input_abs_set_res(input_dev, ABS_MT_POSITION_X, 100);
    input_abs_set_res(input_dev, ABS_MT_POSITION_Y, 100);
    input_abs_set_res(input_dev, ABS_X, 100);
    input_abs_set_res(input_dev, ABS_Y, 100);
    input_mt_init_slots(input_dev, U41_MAX_TARGETS, INPUT_MT_POINTER);

    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 127, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE, 0, 127, 0, 0);
    input_set_capability(input_dev, EV_KEY, BTN_TOOL_PEN);
#endif

    input_set_capability(input_dev, EV_KEY, BTN_LEFT);

    // Force
#ifdef U46_ENABLE_RAW_FORCE_DATA
    set_bit(EV_REL, input_dev->evbit);
    set_bit(EV_MSC, input_dev->evbit);

    // Declare that we support "RAW" Miscellaneous events
    set_bit(MSC_RAW, input_dev->mscbit);
#endif

    ret = input_register_device(input_dev);
    if (ret != 0)
    {
        pr_err("ERROR: aXiom-core: Could not register with Input Sub-system.\n");
        input_unregister_device(input_dev);
        return NULL;
    }
    return input_dev;
}

EXPORT_SYMBOL_GPL(axiom_get_dev_info);
EXPORT_SYMBOL_GPL(axiom_populate_usage_table);
EXPORT_SYMBOL_GPL(usage_to_target_address);
EXPORT_SYMBOL_GPL(axiom_discover);
EXPORT_SYMBOL_GPL(axiom_rebaseline);
EXPORT_SYMBOL_GPL(axiom_init_data_core);
EXPORT_SYMBOL_GPL(axiom_remove);
EXPORT_SYMBOL_GPL(axiom_process_report);
EXPORT_SYMBOL_GPL(axiom_process_u41_report);
EXPORT_SYMBOL_GPL(axiom_process_u46_report);
EXPORT_SYMBOL_GPL(axiom_register_input_subsystem);

MODULE_AUTHOR("TouchNetix <support@touchnetix.com>");
MODULE_DESCRIPTION("aXiom touchscreen core logic");
MODULE_LICENSE("GPL");
MODULE_ALIAS("axiom");
MODULE_VERSION("1.0.0");
