// SPDX-License-Identifier: GPL-2.0
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include "axiom_core.h"

/*
 * axiom_fw_update - Main entry point for aXiom firmware update.
 * Loads firmware, enters bootloader, writes firmware, resets, and restores device.
 */
int axiom_fw_update(struct axiom_data_core *data_core, unsigned char *buffer, int size)
{
	bool index;

	dev_info(data_core->pDev, "Device info before download:");
	print_device_info(data_core);

	// Enter bootloader mode before starting firmware update
	index = enter_bootloader_mode(data_core);
	if (index) {
		// Download firmware to device in chunks
		index = alc_download(data_core, buffer, size);
		if (index) {
			mdelay(100);
			// Reset the device after firmware download
			bootloader_reset_axiom(data_core);
			mdelay(2000);
			dev_info(data_core->pDev, "freeing usage table...\n");
			// Repopulate usage table after firmware update
			data_core->usage_table_populated = false;
			dev_info(data_core->pDev, "Device info after download:");
			axiom_discover(data_core);
			enable_irq(data_core->irq_line);
			axiom_reset(data_core);
			dev_info(data_core->pDev, "Firmware update successful");
		}
	}
	return index;
}
EXPORT_SYMBOL_GPL(axiom_fw_update);

/*
 * alc_download - Download and write all firmware chunks to the device.
 * Returns 1 on success, negative error code on failure.
 */
int alc_download(struct axiom_data_core *data_core, unsigned char *buffer, int size)
{
	u8 chunk_header[8];
	u32 chunk_length = 0;
	int i, j;
	unsigned char *chunk_payload;

	for (i = 0; i < size; i++) {
		// Each chunk starts with an 8-byte header
		for (j = 0; j < 8; j++) {
			chunk_header[j] = buffer[i];
			i++;
		}
		// Chunk length is stored in header bytes 6 and 7 (little endian)
		chunk_length = (chunk_header[6] << 8) | (chunk_header[7]);
		dev_info(data_core->pDev, "chunk_length is 0x%x bytes", chunk_length);
		chunk_payload = &buffer[i];
		// Write the chunk to the device
		if (chunk_length != bootloader_write_chunk(data_core, chunk_header, chunk_length, chunk_payload)) {
			dev_info(data_core->pDev, "%s failed\n", __func__);
			return -EINVAL;
		}
		// Move index to the next chunk (subtract 1 because for-loop will increment i)
		i += (chunk_length - 1);
		dev_info(data_core->pDev, "progress %d", i);
	}
	return true;
}
EXPORT_SYMBOL_GPL(alc_download);

/*
 * system_manager_command - Send a command to the system manager usage (u02).
 * Handles command, unlock keys, and optional verification.
 * Returns 0 on success, negative error code on failure.
 */
int system_manager_command(struct axiom_data_core *data_core, int command, int seq, bool skip_verify)
{
	int resp = 0, retry;
	u8 buffer[8] = {0};
	u16 target_address = usage_to_target_address(data_core, 0x02, 0, 0);

	if (data_core->usage_table_populated) {
		if (axiom_read_page(data_core->pAxiomData, target_address, 8, buffer) != 8) {
			dev_info(data_core->pDev, "%s failed to read\n", __func__);
			return -EINVAL;
		}

		// See SYSTEM_MANAGER_CMD_XXX
		buffer[0] = (command & 0x00FF);
		buffer[1] = (command & 0xFF00) >> 8;
		// A key needs to be provided to unlock the hardware
		if (command == SYSMGR_CMD_SAVE_CONFIG) {
			// Fill in the magic values to commit the data
			buffer[2] = 0x00;
			buffer[3] = 0x00;
			buffer[4] = 0x0C;
			buffer[5] = 0xB1;
			buffer[6] = 0xDE;
			buffer[7] = 0xC0;
		}
		// Bootloader entry sequence requires three steps with different keys
		if (command == SYSMGR_CMD_ENTER_BOOTLOADER) {
			if (seq == 0) {
				buffer[2] = 0x55;
				buffer[3] = 0x55;
			} else if (seq == 1) {
				buffer[2] = 0xAA;
				buffer[3] = 0xAA;
			} else if (seq == 2) {
				buffer[2] = 0x5A;
				buffer[3] = 0xA5;
			}
		}

		if (axiom_write_page(data_core->pAxiomData, target_address, 8, buffer) != 8) {
			dev_info(data_core->pDev, "%s failed to write\n", __func__);
			return -EINVAL;
		}
		mdelay(50);

		// Optionally verify command completion by polling for status
		if (skip_verify == false) {
			for (retry = 0; retry < 10; retry++) {
				mdelay(50);
				if (axiom_read_page(data_core->pAxiomData, target_address, 8, buffer) != 8) {
					dev_info(data_core->pDev, "%s failed to read\n", __func__);
					return -EINVAL;
				}
				resp = buffer[0] + (buffer[1] << 8);
				if (resp == command) {
					dev_info(data_core->pDev, "Command still in progress: %04X", resp);
					continue;
				} else if (resp == 0x0000) {
					dev_info(data_core->pDev, "Command completed successfully: %04X", resp);
					return 0;

				dev_err(data_core->pDev, "ERROR: u02 System Manager error code: %04X", resp);
				return -EINVAL;
				}
			}
			if (retry == 10) {
				dev_info(data_core->pDev, "%s failed and timed out\n", __func__);
				return -EINVAL;
			}
		}
		return resp;
	}
	dev_err(data_core->pDev, "Usage table is not populated");
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(system_manager_command);

/*
 * bootloader_get_busy_status - Check if the bootloader is busy.
 * Returns true if busy, false otherwise.
 */
bool bootloader_get_busy_status(struct axiom_data_core *data_core)
{
	u8 pBuffer[4] = {0};

	axiom_read_page(data_core->pAxiomData, BLP_REG_STATUS, 4, pBuffer);
	// Busy bit is bit 0 of byte 2 in the status register
	return (pBuffer[2] & 0x01) != 0;
}
EXPORT_SYMBOL_GPL(bootloader_get_busy_status);

/*
 * bootloader_reset_axiom - Issue a reset command to aXiom in bootloader mode.
 * Always returns true.
 * NOTE: The reset function here doesn't work as intended, therefore manual reset must be done for changes to reflect.
 */
bool bootloader_reset_axiom(struct axiom_data_core *data_core)
{
	bool reset;
	u8 pBuffer[2] = {0x02, 0x00};

	// Send reset command to bootloader command register
	reset = axiom_write_page(data_core->pAxiomData, BLP_REG_COMMAND, 2, pBuffer);
	if (!reset)
		dev_info(data_core->pDev, "Axiom reset failed --> Ignoring");

	return true;
}
EXPORT_SYMBOL_GPL(bootloader_reset_axiom);

/*
 * write_busy_wait - Wait until aXiom is not busy (with timeout).
 * Returns 0 if ready, -1 if timeout.
 */
int write_busy_wait(struct axiom_data_core *data_core)
{
	// Ensure aXiom is available to process our request
	u32 current_timeout = 0;
	int ret = 0;

	// Poll the busy status with a timeout (max 5 seconds)
	while (bootloader_get_busy_status(data_core)) {
		if (current_timeout < 100) {
			current_timeout = current_timeout + 1;
		} else {
			dev_err(data_core->pDev, "ERROR: aXiom not responding... current_timeout: %d ", current_timeout);
			ret = -1;
			break;
		}
		mdelay(50);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(write_busy_wait);
/*
 * bootloader_write_chunk - Write a firmware chunk (header + payload) to bootloader.
 * Returns length on success, negative error code on failure.
 */
int bootloader_write_chunk(struct axiom_data_core *data_core, u8 *header, u32 length, u8 *payload)
{
	u32 offset, chunk_size;
	u8 payload_chunk[255];
	u32 length_to_write;
	int i;

	if (write_busy_wait(data_core) < 0)
		return -EFAULT;

	// Write chunk header first
	if (axiom_write_page(data_core->pAxiomData, BLP_FIFO_ADDRESS, 8, header) != 8) {
		dev_info(data_core->pDev, "%s failed to write header\n", __func__);
		return -EINVAL;
	}
	if (write_busy_wait(data_core) < 0)
		return -EFAULT;

	// Write payload in 255-byte chunks
	offset = 0;
	chunk_size = 255;
	while (offset < length) {
		length_to_write = 0;
		if ((offset + chunk_size) < length)
			length_to_write = chunk_size;
		else
			length_to_write = length - offset;

		// Copy payload data to temporary buffer
		for (i = 0; i < length_to_write; i++)
			payload_chunk[i] = payload[offset + i];
		if (length_to_write != axiom_write_page(data_core->pAxiomData, BLP_FIFO_ADDRESS, length_to_write, payload_chunk)) {
			dev_info(data_core->pDev, "%s failed to write payload\n", __func__);
			return -EINVAL;
		}
		if (write_busy_wait(data_core) < 0)
			return -EFAULT;
		offset += length_to_write;
	}
	return length;
}
EXPORT_SYMBOL_GPL(bootloader_write_chunk);

/*
 * enter_bootloader_mode - Try to enter aXiom bootloader mode.
 * Returns true if successful, false otherwise.
 */
bool enter_bootloader_mode(struct axiom_data_core *data_core)
{
	int attempts;
	u8 buffer_read[12] = {0};
	bool in_bootloader;
	u16 u31_ta, u31_page0;

	attempts = 5;
	u31_ta = usage_to_target_address(data_core, 0x31, 0, 0);
	// Read status to check if already in bootloader mode
	u31_page0 = axiom_read_page(data_core->pAxiomData, u31_ta, 12, buffer_read);
	in_bootloader = buffer_read[1] & 0x80;

	if (in_bootloader != 0) {
		dev_err(data_core->pDev, "Already in bootloader mode");
		return true;
	}

	if (data_core->usage_table_populated == 0) {
		dev_err(data_core->pDev, "Usage table is not populated");
		return false;
	}

	// Try up to 5 times to enter bootloader mode
	while ((in_bootloader == 0) && (attempts > 0)) {
		dev_err(data_core->pDev, "Disable irq for firmware flashing");
		disable_irq(data_core->irq_line);
		mdelay(100);
		// Bootloader entry requires a 3-step unlock sequence
		system_manager_command(data_core, SYSMGR_CMD_ENTER_BOOTLOADER, 0, false);
		mdelay(100);
		system_manager_command(data_core, SYSMGR_CMD_ENTER_BOOTLOADER, 1, false);
		mdelay(100);
		system_manager_command(data_core, SYSMGR_CMD_ENTER_BOOTLOADER, 2, true);
		mdelay(200);
		u31_page0 = axiom_read_page(data_core->pAxiomData, u31_ta, 12, buffer_read);
		in_bootloader = buffer_read[1] & 0x80;
		if (in_bootloader != 0) {
			dev_info(data_core->pDev, "Now in bootloader mode %d", in_bootloader);
			return true;
		}
		attempts -= 1;
	}
	dev_err(data_core->pDev, "Failed to enter bootloader mode");
	return false;
}
EXPORT_SYMBOL_GPL(enter_bootloader_mode);

MODULE_AUTHOR("Karthik Choda <karthikchoda0110@gmail.com>");
MODULE_AUTHOR("Varun Rajesh <ee22b069@smail.iitm.ac.in>");
MODULE_DESCRIPTION("aXiom Firmware update");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:axiom");