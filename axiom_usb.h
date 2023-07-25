/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef __AXIOM_USB_H
#define __AXIOM_USB_H

#define AX_HID_VID                (0x03EB)    // TouchNetix/Atmel VID
#define AX_HID_VID2               (0x0483)    // TouchNetix/Atmel VID
#define AX_HID_PID                (0x6F02)    // TBP Basic Mode
#define AX_HID_INTNUM             (0)         // Interface 0 on the bridge


// Declaration for Interface numbers
// On TNx USB Bridges there are three interfaces implemented.
// For more details, refer to the relevant protocol bridge datasheet
#define AX_IF_TBPCTRL             (0x0)    // TBP Control Interface
#define AX_IF_DIGITIZER           (0x1)    // Digitizer Interface
#define AX_IF_FORCEDATA           (0x2)    // Press Data Interface

// TNx Touch-Bridge-Protocol Commands
// For more details, refer to the relevant protocol bridge datasheet
#define AX_TBP_CMD_NULL           (0x86)
#define AX_TBP_I2C_DEVICE1        (0x51)  // Read as: issue i2c/SPI transaction to device 1.
#define AX_TBP_I2C_DEVICE2        (0x52)  // Read as: issue i2c/SPI transaction to device 2.
#define AX_TBP_REPEAT             (0x88)  // a.k.a. "put bridge in proxy mode".

// Definitions for USB Id's or reports coming from USB-Bridge
#define AX_TBP_USBID_UNSOLICITED  (0x9A)  // "unsolicited" report from the bridge when in Proxy Mode.

//HID_CONNECT_FF
#define CONNECT_MASK (HID_CONNECT_DEFAULT)

#define HID_PAYLOAD_HEADER_SIZE   (2)
#define HID_PAYLOAD_MAXSIZE       (512)    // a.k.a Maximum USB Packet size
#define HID_PAYLOAD_SIZE_DEFAULT  (64)
#define HID_PAYLOAD_SIZE_TNxPB005 (512)
#define HID_PAYLOAD_SIZE_TNxPB007 (64)
#define HID_PAYLOAD_SIZE_AXPB009  (64)
#define TXBUFFER_SIZE             (HID_PAYLOAD_MAXSIZE + 1)
#define RXBUFFER_SIZE             (HID_PAYLOAD_MAXSIZE + 1)
#define RX_PAYLOAD_MAX            (64 - HID_PAYLOAD_HEADER_SIZE)	// Max transfer size of the USB bridge
																	// interface minus the size of the USB
																	// header to account for additional info
																	// received

// Used when waiting for a response from the bridge.
// Defined in milli-seconds
#define AX_USB_TIMEOUT            (1500U)


const struct hid_device_id axiom_ids[] = {
	{ HID_USB_DEVICE(AX_HID_VID, AX_HID_PID)}, // , .product=2
	{ HID_USB_DEVICE(AX_HID_VID2, AX_HID_PID)},
};


// purpose: The following structure is used to hold all information
//          needed by the driver during run-time. Buffers for USB transactions
//          need to be dynamically allocated (kmalloc) in order for them to
//          support DMA transfers. If it is desired to move this to stack, then
//          tx_buf and rx_buf will have to be turned into pointers.
//
struct axiom_data {
	struct axiom_data_core data_core;

	// USB
	struct usb_device *usbdev;
	struct usb_interface *iface;
	struct hid_device *hdev;
	struct delayed_work work;
	struct workqueue_struct *wq;

	spinlock_t datalock;
	unsigned long irqflags;

	int axiom_hid_payload_size;
	u8 tx_buf[TXBUFFER_SIZE];
	u8 rx_buf[RXBUFFER_SIZE];
	bool proxy_active;
	bool usb_report_available;
	bool do_reports;
};

#endif /* __AXIOM_USB_H */
