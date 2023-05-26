// SPDX-License-Identifier: GPL-2.0
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

// NOTE: For implementation details don't forget to consult README.md

//#define DEBUG   // Enable debug messages

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include "axiom_core.h"
#include "axiom_usb.h"

struct AxiomUsbCmdHeader {
	u8 usbReportId;
	u8 AxiomBridgeCmd;
	u8 wrLen;
	u8 rdLen;
	struct AxiomCmdHeader i2c;
};

// purpose: Helper function to calculate how many USB transfers (chunks)
//          are required to transfer a set amount of bytes
// returns: Status code
int get_rx_chunks(int usage_table_bytes)
{
	int ret = usage_table_bytes / RX_PAYLOAD_MAX;

	if ((((usage_table_bytes % RX_PAYLOAD_MAX) != 0) && (usage_table_bytes > RX_PAYLOAD_MAX)) || (usage_table_bytes < RX_PAYLOAD_MAX))
		return ret + 1;
	else
		return ret;
}


// purpose: Small helper function to reduce repeated code needed
//          to send a USB payload and get response back from bridge.
// returns: Status code
int axiom_usb_do_comms(struct axiom_data *data)
{
	u8 *tx_buf = data->tx_buf;
	u8 *rx_buf = data->rx_buf;
	struct hid_device *hdev = data->hdev;
	spinlock_t *lock = &data->datalock;
	unsigned long *flags = &data->irqflags;
	struct AxiomUsbCmdHeader *pUsbCmd = (struct AxiomUsbCmdHeader *)&data->tx_buf[0];
	int timeout = AX_USB_TIMEOUT;
	int ret = 0;

	// Reset the control flag on the "data" structure
	spin_lock_irqsave(lock, *flags);
	// ACK the IRQ function to indicate we got our report
	data->usb_report_available = false;
	memset(rx_buf, 0, RXBUFFER_SIZE);
	spin_unlock_irqrestore(lock, *flags);

	if (pUsbCmd->AxiomBridgeCmd == AX_TBP_I2C_DEVICE1) {
		if (pUsbCmd->i2c.read == 0 && pUsbCmd->i2c.length > 0)
			hid_dbg(hdev, "i2c write: %*ph\n", pUsbCmd->i2c.length, pUsbCmd->i2c.writeData);
	}

	// Issue the output report
	ret = hid_hw_output_report(hdev, tx_buf, data->axiom_hid_payload_size+1);
	if (ret > 0 && data->do_reports) {
		// Wait here until we get the respond from the Bridge
		while (data->usb_report_available == false) {
			if (timeout <= 0) {
				ret = -EBUSY;
				break;
			}

			timeout--;
			mdelay(1);
		}
	}

	if (ret > 0) {
		hid_dbg(hdev, "AXIOM:...%s RESP=0x%02x rc=%d\n", __func__, rx_buf[0], ret);
		hid_dbg(hdev, "HID response header: %*ph\n", 2, &rx_buf[0]);

		if (rx_buf[1] > 0)
			hid_dbg(hdev, "Axiom data: %*ph\n", rx_buf[1], &rx_buf[2]);
	} else {
		hid_err(hdev, "ERROR:...%s rc=%d\n", __func__, ret);
	}

	return ret;
}

// purpose: Helper function to read a specified usage and write it into the provided buffer
// returns: Length of the usage read
static u16 axiom_read_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
	struct axiom_data *data = pAxiomData;
	struct hid_device *hdev = data->hdev;
	spinlock_t *lock = &data->datalock;
	unsigned long *flags = &data->irqflags;
	int numberOfBlocks;
	int bytes_read;
	u8 bytes_to_read;
	int chunk;
	int ret;

	struct AxiomUsbCmdHeader *pUsbCmd = (struct AxiomUsbCmdHeader *)&data->tx_buf[0];
	u8 *rx_buf = &data->rx_buf[0];

	numberOfBlocks = get_rx_chunks(length);
	bytes_read = 0;
	for (chunk = 0; chunk < numberOfBlocks; chunk++) {
		int chunkPage = ((chunk * RX_PAYLOAD_MAX) / AX_COMMS_PAGE_SIZE) + page;

		if (chunk != (numberOfBlocks - 1))
			bytes_to_read = RX_PAYLOAD_MAX;
		else
			bytes_to_read = (length % RX_PAYLOAD_MAX);

		//hid_info(hdev, "Usage chunk %d, page: %d, bytes_to_read: %d\n", chunk, page, bytes_to_read);
		spin_lock_irqsave(lock, *flags);
		memset(&data->tx_buf[0], 0, TXBUFFER_SIZE);
		pUsbCmd->usbReportId = 0;
		pUsbCmd->AxiomBridgeCmd = AX_TBP_I2C_DEVICE1;
		pUsbCmd->wrLen = sizeof(struct AxiomCmdHeader);
		pUsbCmd->rdLen = bytes_to_read;
		pUsbCmd->i2c.target_address = usage_to_target_address(&data->data_core, usage, chunkPage, bytes_read);
		pUsbCmd->i2c.length = bytes_to_read;
		pUsbCmd->i2c.read = 1;
		spin_unlock_irqrestore(lock, *flags);

		// Send command
		ret = axiom_usb_do_comms(data);
		if (ret <= 0) {
			hid_err(hdev, "Failed to %s\n", __func__);
			return 0;
		}

		// Check response
		spin_lock_irqsave(lock, *flags);
		if (rx_buf[0] != 0x0 || rx_buf[1] != bytes_to_read)
			hid_err(hdev, "ERROR: aXiom-usb response from bridge is not as expected\n");

		// Copy raw data into the aXiom-data struct.
		memcpy(&pBuffer[chunk * RX_PAYLOAD_MAX], &rx_buf[2], bytes_to_read);
		bytes_read += bytes_to_read;
		spin_unlock_irqrestore(lock, *flags);
		hid_info(hdev, "%s: bytes read: %d, chunk %d\n", __func__, bytes_read, chunk);
	}

	return length;
}

// purpose: Helper function to write data in a provided buffer to a specified usage
// returns: Length of the data to write
static u16 axiom_write_usage(void *pAxiomData, u8 usage, u8 page, u16 length, u8 *pBuffer)
{
	struct axiom_data *data = pAxiomData;
	struct hid_device *hdev = data->hdev;
	spinlock_t *lock = &data->datalock;
	unsigned long *flags = &data->irqflags;
	int ret;
	struct AxiomUsbCmdHeader *pUsbCmd = (struct AxiomUsbCmdHeader *)&data->tx_buf[0];

	//hid_dbg(hdev,"axiom_write_usage(usage %u, page %u, length %u)", usage,  page, length);
	spin_lock_irqsave(lock, *flags);
	memset(&data->tx_buf[0], 0, TXBUFFER_SIZE);
	pUsbCmd->usbReportId = 0;
	pUsbCmd->AxiomBridgeCmd = AX_TBP_I2C_DEVICE1;
	pUsbCmd->wrLen = sizeof(struct AxiomCmdHeader);
	pUsbCmd->rdLen = length;
	pUsbCmd->i2c.target_address = usage_to_target_address(&data->data_core, usage, page, 0);
	pUsbCmd->i2c.length = length;
	pUsbCmd->i2c.read = 0;
	memcpy(pUsbCmd->i2c.writeData, pBuffer, length);
	spin_unlock_irqrestore(lock, *flags);

	// Send command
	ret = axiom_usb_do_comms(data);
	if (ret <= 0) {
		hid_err(hdev, "Failed to axiom_read_usage\n");
		return 0;
	}
	return length;
	}

// purpose: Helper function to take the USB bridge out of proxy mode
// returns: Status code
int axiom_stop_proxy(struct hid_device *hdev, struct axiom_data *data)
{
	spinlock_t *lock = &data->datalock;
	unsigned long *flags = &data->irqflags;
	u8 *tx_buf = data->tx_buf;
	u8 *rx_buf = data->rx_buf;
	int ret = 0;

	// Issue a CMD_NULL to put the bridge on a known state
	// (ie cancel “Proxy” (repeat) mode)
	spin_lock_irqsave(lock, *flags);
	memset(tx_buf, 0, TXBUFFER_SIZE);
	tx_buf[0] = 0x0;             // USB report ID
	tx_buf[1] = AX_TBP_CMD_NULL; // Bridge command

	// Ready to start talking to Bridge.
	spin_unlock_irqrestore(lock, *flags);

	// Send command
	ret = axiom_usb_do_comms(data);
	if (ret > 0 && data->do_reports) {
		// Check response
		spin_lock_irqsave(lock, *flags);

		if (rx_buf[0] != AX_TBP_CMD_NULL)
			hid_err(hdev, "ERROR: aXiom-usb did not get correct response from bridge.\n");

		spin_unlock_irqrestore(lock, *flags);
	}

	hid_info(hdev, "AXIOM:...%s rc=%d\n", __func__, ret);
	return ret;
}


// purpose: Function executed as a workqueue item.
//          The following tasks are done:
//           * Extract information from the device's (u31) to
//             allow the driver build its local usage table.
//           * Read further pages of u31 to build local usage table.
//           * With the device information and usage table,
//             build command to configure the USB Bridge into "Proxy" mode.
static void axiom_usb_setup(struct work_struct *work)
{
	struct axiom_data *data = container_of(work, struct axiom_data, work.work);
	struct axiom_data_core *data_core = &data->data_core;
	struct hid_device *hdev = data->hdev;
	spinlock_t *lock = &data->datalock;
	unsigned long *flags = &data->irqflags;
	u8 *tx_buf = data->tx_buf;
	u8 *rx_buf = data->rx_buf;
	u16 target_address = 0;
	int ret = 0;

	hid_info(hdev, "AXIOM: aXiom-usb entering USB Setup [%s ver 0x%x]\n", hdev->name, hdev->version);

	data->axiom_hid_payload_size = HID_PAYLOAD_SIZE_DEFAULT;
	if (strncmp("TouchNetix TNxPB-005", hdev->name, 20) == 0)
		data->axiom_hid_payload_size = HID_PAYLOAD_SIZE_TNxPB005;
	else if (strncmp("TouchNetix TNxPB-007", hdev->name, 20) == 0)
		data->axiom_hid_payload_size = HID_PAYLOAD_SIZE_TNxPB007;
	else if (strncmp("TouchNetix AXPB009", hdev->name, 20) == 0)
		data->axiom_hid_payload_size = HID_PAYLOAD_SIZE_AXPB009;
	else
		//Check other hardware ID's...
		data->axiom_hid_payload_size = HID_PAYLOAD_SIZE_DEFAULT;

	hid_info(hdev, "AXIOM: axiom_hid_payload_size: %d\n", data->axiom_hid_payload_size);
	data->do_reports = true;
	ret = axiom_stop_proxy(hdev, data);
	if (ret <= 0)
		goto usb_abort;

	if (!axiom_discover(data_core))
		goto usb_abort;

	axiom_rebaseline(data_core);

	///////////////////////////////////////////////////
	// Putting Bridge in Proxy Mode:
	///////////////////////////////////////////////////
	spin_lock_irqsave(lock, *flags);
	target_address = usage_to_target_address(data_core, 0x34, 0, 0);
	memset(tx_buf, 0, TXBUFFER_SIZE);
	// Payload to put the bridge into proxy mode
	tx_buf[0] = 0x00;                 // USB report ID
	tx_buf[1] = AX_TBP_REPEAT;        // Bridge command: PROXY
	tx_buf[2] = 0x58;                 // First device, when nIRQ Low, with 0 delay
	tx_buf[3] = 0x04;                 // Write 4 bytes
	tx_buf[4] = data_core->max_report_len; // Read 62 bytes

	// The target address (TA) *should* not be hard coded! The TA for U34 can be
	// determined by reading the usage table from U31.
	tx_buf[5] = target_address & 0xff; // Write byte 0 - TA LSB. (offset = 0)
	tx_buf[6] = target_address >> 8;   // Write byte 1 - TA MSB. (U34).
	tx_buf[7] = data_core->max_report_len;  // Write byte 2 - Length LSB to read from aXiom. 62 bytes
	tx_buf[8] = AX_COMMS_READ;         // Write byte 3 - Length MSB and Read bit set
	spin_unlock_irqrestore(lock, *flags);

	hid_info(hdev, "AXIOM: aXiom-usb trying to put device into Proxy Mode\n");

	// Send command
	ret = axiom_usb_do_comms(data);
	if (ret <= 0)
		goto usb_abort;

	// Check response
	spin_lock_irqsave(lock, *flags);
	if ((rx_buf[0] == AX_TBP_REPEAT && rx_buf[1] == 0x0) ||         // PB-005
		(rx_buf[0] == AX_TBP_USBID_UNSOLICITED && rx_buf[1] == 0x4) // PB-007
	) {
		hid_info(hdev, "AXIOM: aXiom-usb has USB Bridge in Proxy Mode!\n");
	} else {
		hid_err(hdev, "ERROR: aXiom-usb failed to put bridge in Proxy Mode!\n");
		spin_unlock_irqrestore(lock, *flags);
		goto usb_abort;
	}

	// Record in the data structure that proxy mode is active!
	data->proxy_active = true;

	// NOTE: axiom_usb_setup is supposed to run only once.
	// NOTE: we have not used the synchronous version of cancel_delayed_work
	cancel_delayed_work(&data->work);

	spin_unlock_irqrestore(lock, *flags);
	hid_info(hdev, "AXIOM: aXiom-usb finished aXiom USB driver set-up!\n");
	return;

usb_abort:
	hid_err(hdev, "ERROR: aXiom-usb failed to initialize device...\n");
	cancel_delayed_work(&data->work);
	//any further cleanup will occur in axiom_usb_remove...
}


// purpose: Function called in IRQ context when device is plugged in.
// returns: Error code
static int axiom_usb_probe(struct hid_device *hdev,
							const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	int iface_no = intf->altsetting->desc.bInterfaceNumber;
	unsigned int connect_mask = CONNECT_MASK;
	struct axiom_data *data;
	struct axiom_data_core *data_core;
	int ret = 0;
	int i = 0;
	spinlock_t *lock;
	unsigned long *flags;

	// Only connect to the "Digitizer Control Interface"
	if (iface_no != AX_IF_TBPCTRL) {
		hid_info(hdev, "AXIOM: %s interface %d, done\n", __func__, iface_no);
		// Returning -ENOSYS here just throws up an error in dmesg.
		// Just to return without allocating resources is OK
		return 0;
	}

	hid_info(hdev, "AXIOM: Starting aXiom Probe!\n");

	hid_info(hdev, "AXIOM: Attaching axiom_usb driver to Interface %d...\n", iface_no);

	// Get some memory that we can use within the driver.
	data = kzalloc(sizeof(struct axiom_data), GFP_KERNEL);
	if (data == NULL) {
		hid_err(hdev, "Failed to allocate memory for aXiom data structure!\n");
		return -ENOMEM;
	}

	lock = &data->datalock;
	spin_lock_init(lock);
	flags = &data->irqflags;

	spin_lock_irqsave(lock, *flags);

	data_core = &data->data_core;
	axiom_init_data_core(data_core, &hdev->dev, data, &axiom_read_usage, &axiom_write_usage);

	// Now cross-link structures to be able to access all from either side.
	data->hdev = hdev;
	data->iface = intf;
	data->usbdev = to_usb_device(hdev->dev.parent->parent);

	data->proxy_active = false;
	data->do_reports = false;
	data->usb_report_available = false;
	data->wq = create_singlethread_workqueue("axiomusb");


	hid_set_drvdata(hdev, data);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "ERROR: %s: parse failed\n", __func__);
		goto abort;
	}

	// Initialize values on the workqueue structure.
	INIT_DELAYED_WORK(&data->work, axiom_usb_setup);

	// The following is necessary to get started.
	ret = hid_hw_start(hdev, connect_mask);
	if (ret) {
		hid_err(hdev, "ERROR: hw start failed\n");
		goto abort;
	}

	// NOTE: Some devices seem to issue power-mode requests at this stage.
	//       This is not necessary for us.

	// If an open is not issued, then the bridge will not get "the ping" to
	// start sending reports.
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "ERROR: hw open failed.\n");
		goto abort;
	}

	data_core->input_dev = axiom_register_input_subsystem();
	if (data_core->input_dev == NULL) {
		hid_err(hdev, "ERROR: Failed to register input device, error: %d\n", ret);
		goto abort;
	}
	hid_info(hdev, "AXIOM: USB driver registered with Input Sub-System.\n");

	// Ensure that all reports are initialised to not be present.
	for (i = 0; i < U41_MAX_TARGETS; i++)
		data_core->targets[i].state = Target_State_Not_Present;

	// Queue delayed work, HZ means schedule to be done after at least 1 second.
	ret = queue_delayed_work(data->wq, &data->work, HZ);
	spin_unlock_irqrestore(lock, *flags);

	hid_info(hdev, "AXIOM: Spun aXiom worker! %d\n", ret);
	dev_info(&hdev->dev, "AXIOM: Probe done on interface %d\n", iface_no);
	return 0;

abort:
	destroy_workqueue(data->wq);
	hid_hw_stop(hdev);
	hid_set_drvdata(hdev, NULL);
	kfree(data);
	return ret;
}


// purpose: Clean-up when device is disconnected
static void axiom_usb_remove(struct hid_device *hdev)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	int iface_no = intf->altsetting->desc.bInterfaceNumber;
	struct axiom_data *data;
	struct axiom_data_core *data_core;

	if (iface_no != AX_IF_TBPCTRL) {
		hid_info(hdev, "AXIOM: %s interface %d removed\n", __func__, iface_no);
		return;
	}

	data = (struct axiom_data *)hid_get_drvdata(hdev);
	data_core = &data->data_core;

	data->do_reports = false;
	axiom_stop_proxy(hdev, data);

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
	cancel_delayed_work_sync(&data->work);
	flush_workqueue(data->wq);
	destroy_workqueue(data->wq);

	axiom_remove(data_core);

	kfree(data);
	hid_set_drvdata(hdev, NULL);
	hid_info(hdev, "AXIOM: aXiom USB Bridge removed! %s %d\n", __func__, __LINE__);
}

// purpose: Called withint IRQ context, everytime there is a USB report arriving
//          from the USB Bridge.
static int axiom_usb_raw_event(struct hid_device *hdev,
								struct hid_report *report,
								u8 *raw_data,
								int size)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	int iface_no = intf->altsetting->desc.bInterfaceNumber;
	struct axiom_data *data;
	struct axiom_data_core *data_core;
	spinlock_t *lock;
	unsigned long *flags;
	int tocopy;

	if (iface_no != AX_IF_TBPCTRL) {
		hid_info(hdev, "AXIOM: %s on interface %d not handled!\n", __func__, iface_no);
		return 0;
	}
	//hid_info(hdev, "AXIOM: axiom_usb_raw_event\n");

	data = hid_get_drvdata(hdev);
	data_core = &data->data_core;
	lock = &data->datalock;
	flags = &data->irqflags;

	// Check if the rest of the driver is ready to handle reports,
	// if the other functions are not ready then discard report.
	spin_lock_irqsave(lock, *flags);
	if (data->do_reports == true) {
		// If the usage table is populated and proxy mode is active, then
		// we assume most likely this is a report transmitted by the Bridge
		// from the aXiom device.
		if (data_core->usage_table_populated == true && data->proxy_active == true) {
			// Check for correct USB Id
			if (raw_data[0] == AX_TBP_USBID_UNSOLICITED) {
				axiom_process_report(data_core, &raw_data[2]);
			} else {
				hid_dbg(hdev, "aXiom-rep: Got a different report\n");
				hid_dbg(hdev, "BUF: %*ph", size, raw_data);
			}
		}

		// If the Usage Table is not populated or the device is not
		// in proxy mode then we pass the data to the back-end of this
		// module, as some set-up process must be going on.
		else {
			// Check that we are good to write into the RX buffer.
			if (data->usb_report_available == false) {
				hid_dbg(hdev, "aXiom-rep: Putting report data in buffer. (%d bytes)\n", size);

				// Check that we don't copy more than our RX buffer.
				if (size > RXBUFFER_SIZE)
					tocopy = RXBUFFER_SIZE;
				else
					tocopy = size;

				// Copy raw data into the aXiom-data struct.
				memcpy(data->rx_buf, raw_data, tocopy);
				hid_dbg(hdev, "BUF: %*ph", tocopy, raw_data);
				hid_dbg(hdev, "    size: %d", size);
				hid_dbg(hdev, "    tocopy: %d", tocopy);

				// Signal that there is a report available.
				data->usb_report_available = true;
			} else {
				hid_err(hdev, "ERROR: Report response not retrieved by aXiom driver\n");
			}
		}
	}
	spin_unlock_irqrestore(lock, *flags);
	return 0;
}


static struct hid_driver axiom_usb_driver = {
	.name = "axiom_usb",
	.id_table = axiom_ids,
	.probe = axiom_usb_probe,
	.remove = axiom_usb_remove,
	.raw_event = axiom_usb_raw_event
	};
module_hid_driver(axiom_usb_driver);

MODULE_AUTHOR("TouchNetix <support@touchnetix.com>");
MODULE_DESCRIPTION("aXiom touchscreen USB bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("axiom");
MODULE_VERSION("1.0.0");
