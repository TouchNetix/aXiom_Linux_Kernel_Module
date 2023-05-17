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

#ifndef __AXIOM_CORE_H
#define __AXIOM_CORE_H

////////////////////////////////////////////////////////////////////////////////
//USER OPTIONS

//#define AXIOM_USE_TOUCHSCREEN_INTERFACE     //registers the axiom device as a touch screen instead of as a mouse pointer
//#define AXIOM_USE_KERNEL_SLOT_ASSIGNMENT    //uses the multitouch protocol for target tracking/assignment instead of axiom
#define U46_ENABLE_RAW_FORCE_DATA             //enables the raw data for up to 4 force channels to be sent to the input subsystem
////////////////////////////////////////////////////////////////////////////////

// u31 has 2 pages for usage table entries. (2 * AX_COMMS_PAGE_SIZE) / U31_BYTES_PER_USAGE = 85
#define U31_MAX_USAGES              (85U) 
#define U41_MAX_TARGETS             (10U)
#define U46_AUX_CHANNELS            (4U)
#define U46_AUX_MASK                (0xFFFU)
#define U31_BYTES_PER_USAGE         (6U)
#define USAGE_2DCTS_REPORT_ID       (0x41U)
#define USAGE_2AUX_REPORT_ID        (0x46U)
#define USAGE_2HB_REPORT_ID         (0x01U)
#define PROX_LEVEL                  (-128)
#define AX_U31_PAGE0_LENGTH         (0x0C)
// For details check TNxAN00035: "aXiom_Touch_Controller_Comms_Protocol"
#define AX_COMMS_WRITE              (0x00U)
#define AX_COMMS_READ               (0x80U)
#define AX_COMMS_BYTES_MASK         (0xFFU)

#define COMMS_MAX_USAGE_PAGES       (3)
#define AX_COMMS_PAGE_SIZE          (256)

#define COMMS_OVERFLOW_MSK          (0x80)
#define COMMS_REPORT_LEN_MSK        (0x7F)

#include <linux/input.h>

#ifdef __AXIOM_KERNEL_5_4
#define input_mt_report_slot_inactive(dev) input_mt_report_slot_state(dev, 0, false)
#endif

// purpose: Holds device specific information
struct u31_DeviceInfo
{
    u8 bootloader_mode;
    u16 device_id;
    u8 fw_major;
    u8 fw_minor;
    u16 fw_info_extra;
    u8 bootloader_fw_ver_major;
    u8 bootloader_fw_ver_minor;
    u16 jedec_id;
    u8 num_usages;
    u8 silicon_revision;
};


// purpose: Describes parameters of a specific usage, essenstially a single
//          element of the "Usage Table"
struct usage_Entry
{
    u8 id;
    u8 is_report;
    u8 start_page;
    u8 num_pages;
};

// purpose: Holds state of a "Target", A.K.A. as a "touch", but called a
//          target as it can be a detected "target" prior to touch, eg, hovering.
enum u41_Target_State_e
{
    Target_State_Not_Present = 0,
    Target_State_Prox        = 1,
    Target_State_Hover       = 2,
    Target_State_Touching    = 3,

    Target_State_Min         = Target_State_Not_Present,
    Target_State_Max         = Target_State_Touching,
};

// purpose: Holds information describing a target.
struct u41_Target
{
    enum u41_Target_State_e state;
    u16 x;
    u16 y;
    s8  z;
    bool insert;
    bool touch;
};

// purpose: Holds decoded data from an aXiom u41 report.
struct u41_Target_Report
{
    u8  index;
    u8  present;
    u16 x;
    u16 y;
    s8  z;
};

// purpose: I2C & SPI command header structure
typedef struct AxiomCmdHeader_struct
{
    u16 target_address;
    u16 length  :15;
    u16 read    :1;
    u8  writeData[];
} AxiomCmdHeader , *pAxiomCmdHeader;

// purpose: Groups several structures needed for the core module.
struct axiom_data_core
{
    // aXiom entries
    struct u31_DeviceInfo u31_Info;
    struct u41_Target targets[U41_MAX_TARGETS];
    struct usage_Entry usage_table[U31_MAX_USAGES];
    bool usage_table_populated;
    u8 max_report_len;
    u32 report_overflow_counter;
    u32 report_counter;
    u32 bus_holdoff_delay_us;
    // This could potentially be allocated during discovery.
    u8 rx_buf[COMMS_MAX_USAGE_PAGES * AX_COMMS_PAGE_SIZE];

    // Input Sub-system
    struct input_dev *input_dev;

    // The device
    struct device *pDev;
    void *pAxiomData;
    u16 (*pAxiomReadUsage)(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer);
    u16 (*pAxiomWriteUsage)(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer);
};

extern void axiom_get_dev_info(struct axiom_data_core *data_core, u8 *data);

extern u8 axiom_populate_usage_table(struct axiom_data_core *data_core, u8 *pRX_data);

extern u16 usage_to_target_address(struct axiom_data_core *data_core,
                                   u8 usage, u8 page, u8 offset);

extern bool axiom_discover(struct axiom_data_core *data_core);

extern void axiom_rebaseline(struct axiom_data_core *data_core);

extern void axiom_init_data_core(struct axiom_data_core *data_core, struct device *pDev, void *pAxiomData, void *pAxiomReadUsage, void *pAxiomWriteUsage);

extern void axiom_remove(struct axiom_data_core *data_core);

extern void axiom_process_report(struct axiom_data_core *data_core, u8 *pReport);

extern void axiom_process_u41_report(u8 *rx_buf, struct axiom_data_core *data_core);

extern void axiom_process_u46_report(u8 *rx_buf, struct axiom_data_core *data_core);

extern struct input_dev *axiom_register_input_subsystem(void);

#endif  /* __AXIOM_CORE_H */
