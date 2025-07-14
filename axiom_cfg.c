// SPDX-License-Identifier: GPL-2.0
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include "axiom_core.h"

#define CRC_CAL(usage_content, pos) ((u32)((usage_content)[pos + 3] << 24 | (usage_content)[pos + 2] << 16 | (usage_content)[pos + 1] << 8 | (usage_content)[pos]))

/*
 * Loads and flashes a configuration file to the device, disables IRQs,
 * parses the config, writes to device, resets, and verifies CRCs.
 */
int axiom_cfg_update(struct axiom_data_core *data_core, unsigned char *buffer, int size)
{
	bool compare_crc = 0;
	struct u33_CRCData file_crc_data = {0};
	struct u33_CRCData device_crc_data = {0};

	dev_info(data_core->pDev, "u33 CRC Data Before Config Load:");
	device_crc_data = get_target_crc_data(data_core, device_crc_data);
	printU33Data(data_core, device_crc_data);

	// Disable IRQs before flashing config to avoid race conditions
	dev_err(data_core->pDev, "disable irq for config flashing");
	disable_irq(data_core->irq_line);
	mdelay(100);
	system_manager_command(data_core, SYSMGR_CMD_STOP, 0, false);
	msleep(500);

	// Parse and write the configuration file to the device
	file_crc_data = parse_config_file(data_core, file_crc_data, buffer, size);

	// Save config and reset device to apply changes
	system_manager_command(data_core, SYSMGR_CMD_SAVE_CONFIG, 0, false);
	msleep(2000);
	system_manager_command(data_core, SYSMGR_CMD_SOFT_RESET, 0, false);
	msleep(1000);

	// Free and repopulate usage table after config update
	dev_info(data_core->pDev, "freeing usage table...\n");
	data_core->usage_table_populated = false;
	dev_info(data_core->pDev, "Device info After download:");
	axiom_discover(data_core);
	enable_irq(data_core->irq_line);
	axiom_reset(data_core);

	// Compare CRCs after config load to ensure integrity
	dev_info(data_core->pDev, "u33 CRC Data After Config Load and Compared with File Contents:");
	device_crc_data = get_target_crc_data(data_core, device_crc_data);
	printU33Data(data_core, device_crc_data);

	compare_crc = compare_CRC_config(data_core, file_crc_data, device_crc_data);
	if (!compare_crc) {
		dev_err(data_core->pDev, "Config CRC Check failed");
		return -EINVAL;
	}
	dev_info(data_core->pDev, "Configuration update successful");
	return true;
}
EXPORT_SYMBOL_GPL(axiom_cfg_update);

/*
 * Compares CRC fields between the config file and device.
 */
int compare_CRC_config(struct axiom_data_core *data_core, struct u33_CRCData file_crc_data, struct u33_CRCData device_crc_data)
{
	// Compare each CRC field individually for detailed error reporting
	if (device_crc_data.config_nvm_crc != file_crc_data.config_nvm_crc) {
		dev_err(data_core->pDev, "config nvm crc error");
		return -EINVAL;
	}
	if (device_crc_data.config_ram_crc != file_crc_data.config_ram_crc) {
		dev_err(data_core->pDev, "config ram crc error");
		return -EINVAL;
	}
	if (device_crc_data.cdu_u05_comments_crc != file_crc_data.cdu_u05_comments_crc) {
		dev_err(data_core->pDev, "cdu u05 comments crc error");
		return -EINVAL;
	}
	if (device_crc_data.cdu_u93_profile_crc != file_crc_data.cdu_u93_profile_crc) {
		dev_err(data_core->pDev, "cdu u93 profile crc error");
		return -EINVAL;
	}
	if (device_crc_data.cdu_u94_delta_scale_map_crc != file_crc_data.cdu_u94_delta_scale_map_crc) {
		dev_err(data_core->pDev, "cdu u94 delta scale map crc error");
		return -EINVAL;
	}
	if (device_crc_data.runtime_hash != file_crc_data.runtime_hash) {
		dev_err(data_core->pDev, "runtime hash crc error");
		return -EINVAL;
	}
	dev_info(data_core->pDev, "CRC Hash check successful");
	return true;
}
EXPORT_SYMBOL_GPL(compare_CRC_config);

/*
 * Logs all CRC fields and revision info from a u33_CRCData struct.
 */
void printU33Data(struct axiom_data_core *data_core, struct u33_CRCData crc_data)
{
	dev_info(data_core->pDev, "u33 revision : %x\n", crc_data.revision);
	dev_info(data_core->pDev, "runtime_crc : 0x%08X\n", crc_data.runtime_crc);
	dev_info(data_core->pDev, "runtime_nvm_crc : 0x%08X\n", crc_data.runtime_nvm_crc);
	dev_info(data_core->pDev, "bootloader_crc : 0x%08X\n", crc_data.bootloader_crc);
	dev_info(data_core->pDev, "config_nvm_crc : 0x%08X\n", crc_data.config_nvm_crc);
	dev_info(data_core->pDev, "config_ram_crc : 0x%08X\n", crc_data.config_ram_crc);
	if (crc_data.revision <= 1)
		dev_info(data_core->pDev, "cdu_u05_comments_crc : 0x%08X\n", crc_data.cdu_u05_comments_crc);
	dev_info(data_core->pDev, "cdu_u93_profile_crc : 0x%08X\n", crc_data.cdu_u93_profile_crc);
	dev_info(data_core->pDev, "cdu_u94_delta_scale_map_crc : 0x%08X\n", crc_data.cdu_u94_delta_scale_map_crc);
	dev_info(data_core->pDev, "runtime_hash : 0x%08X\n", crc_data.runtime_hash);
}
EXPORT_SYMBOL_GPL(printU33Data);

/*
 * Parses the configuration file, writes each usage section to the device,
 * and extracts CRC data if present.
 */
struct u33_CRCData parse_config_file(struct axiom_data_core *data_core, struct u33_CRCData file_crc_data, unsigned char *buffer, int size)
{
	int index;
	u32 signature;
	u16 version;
	u16 target_address;

	// Check file signature to ensure it's a valid aXiom config file
	signature = (u32)(buffer[0] << 24 | buffer[1] << 16 | buffer[2] << 8 | buffer[3]);
	dev_info(data_core->pDev, "CFG File Signature : 0x%08X\n", signature); // Extract Signature --> To identify the file as an aXiom config file.
	if (signature != 0x20071969) {
		dev_err(data_core->pDev, "CFG File not recognized\n");
		return file_crc_data;
	}
	version = (uint16_t)((buffer[4] << 8) | buffer[5]); // Extract Version
	dev_info(data_core->pDev, "CFG File Version : %x\n", version);

	// Start parsing usages from offset 13 (header size)
	for (index = 13; index < size; index++) {
		dev_dbg(data_core->pDev, "index : %d\n", index);
		// Extract usage, revision, profile, length, usage_content
		file_crc_data.usage =  (u8)(buffer[index]);
		file_crc_data.revision = (u8)(buffer[index+1]);
		file_crc_data.profile = (u8)(buffer[index+2]);
		file_crc_data.length = (u16)((buffer[index+4] << 8) | buffer[index+3]);
		file_crc_data.usage_content = &buffer[index + 5];

		dev_dbg(data_core->pDev, "CFG usage : 0x%x\n", file_crc_data.usage); // In HEXA
		dev_dbg(data_core->pDev, "CFG revision : 0x%x\n", file_crc_data.revision); // In HEXA
		dev_dbg(data_core->pDev, "CFG profile : 0x%x\n", file_crc_data.profile); // In HEXA
		dev_dbg(data_core->pDev, "CFG length : %d\n", file_crc_data.length); // In DEC

		// Usage 0x04 is reserved, skip it
		if (file_crc_data.usage != 0x04) {
			if (ignore_usage_list(data_core, file_crc_data.usage)) {
				dev_info(data_core->pDev, "Skipping usage as it should be ignored");
			} else if (cdu_usage_list(data_core, file_crc_data.usage)) {
				dev_info(data_core->pDev, "Encountered cdu type usage");
				// CDU usages require special write logic
				if (cdu_write(data_core, file_crc_data.usage, file_crc_data.length, file_crc_data.usage_content)) {
					dev_err(data_core->pDev, "cdu_write failed for %d usage", file_crc_data.usage);
					return file_crc_data;
				}
			} else {
				dev_info(data_core->pDev, "Regular usage %x", file_crc_data.usage);
				target_address = usage_to_target_address(data_core, file_crc_data.usage, 0, 0);
				if (file_crc_data.length != axiom_write_page(data_core->pAxiomData, target_address, file_crc_data.length, file_crc_data.usage_content)) {
					dev_err(data_core->pDev, "axiom_write_usage failed for %d usage", file_crc_data.usage);
					return file_crc_data;
				}
			}
		}

		// If this is the CRC usage, extract CRC data for later comparison
		if (file_crc_data.usage == 0x33) {
			file_crc_data = get_u33_crc_data(data_core, file_crc_data);
			dev_info(data_core->pDev, "CRC data from file:");
			printU33Data(data_core, file_crc_data);
		}
		// Move to next usage section
		index += (file_crc_data.length + 4);
	}
	return file_crc_data;
}
EXPORT_SYMBOL_GPL(parse_config_file);

/*
 * Reads CRC data from the device for usage 0x33 and fills a u33_CRCData struct.
 */
struct u33_CRCData get_target_crc_data(struct axiom_data_core *data_core, struct u33_CRCData device_crc_data)
{
	u8 *pRX_data;
	u16 target_address = usage_to_target_address(data_core, 0x33, 0, 0);
	struct usage_Entry *pUsage = NULL;

	pRX_data = &data_core->rx_buf[0];
	dev_info(data_core->pDev, "%s Called : %04x\n", __func__, target_address);
	if (data_core->usage_table_populated) {
		dev_dbg(data_core->pDev, "usage_table populated");
		pUsage = get_usage(data_core, 0x33);
		device_crc_data.usage = 0x33;
		device_crc_data.revision = pUsage->usage_rev;
		device_crc_data.length = pUsage->length;
		// Read CRC data from device
		if (device_crc_data.length != axiom_read_page(data_core->pAxiomData, target_address, device_crc_data.length, pRX_data)) {
			dev_info(data_core->pDev, "%s is failed in read\n", __func__);
			return device_crc_data;
		}
		device_crc_data.usage_content = &pRX_data[0];
		device_crc_data = get_u33_crc_data(data_core, device_crc_data);
		return device_crc_data;
	}
	dev_err(data_core->pDev, "usage_table is not populated");
	return device_crc_data;
}
EXPORT_SYMBOL_GPL(get_target_crc_data);

/*
 * Extracts all CRC fields from the usage content buffer based on revision.
 */
struct u33_CRCData get_u33_crc_data(struct axiom_data_core *data_core, struct u33_CRCData crc_data)
{
	// Extract crc_data
	dev_dbg(data_core->pDev, "crc_data CFG usage : %x\n", crc_data.usage); // In HEXA
	dev_dbg(data_core->pDev, "crc_data CFG revision : %x\n", crc_data.revision); // In HEXA
	dev_dbg(data_core->pDev, "crc_data CFG profile : %x\n", crc_data.profile); // In HEXA
	dev_dbg(data_core->pDev, "crc_data CFG length : %d\n", crc_data.length); // In DEC

	// Extract CRCs from usage_content buffer
	crc_data.revision = crc_data.revision;
	crc_data.runtime_crc = CRC_CAL(crc_data.usage_content, 0);
	crc_data.runtime_nvm_crc = CRC_CAL(crc_data.usage_content, 4);
	crc_data.bootloader_crc = CRC_CAL(crc_data.usage_content, 8);
	crc_data.config_nvm_crc = CRC_CAL(crc_data.usage_content, 12);
	crc_data.config_ram_crc = CRC_CAL(crc_data.usage_content, 16);

	// CRC field layout depends on revision
	if (crc_data.revision <= 1) {
		crc_data.cdu_u05_comments_crc = CRC_CAL(crc_data.usage_content, 20);
		crc_data.cdu_u22_seqeunces_crc = CRC_CAL(crc_data.usage_content, 24);
		crc_data.cdu_u43_hotspots_crc = CRC_CAL(crc_data.usage_content, 28);
		crc_data.cdu_u93_profile_crc = CRC_CAL(crc_data.usage_content, 32);
		crc_data.cdu_u94_delta_scale_map_crc = CRC_CAL(crc_data.usage_content, 36);
		crc_data.runtime_hash = CRC_CAL(crc_data.usage_content, 40);
	} else if (crc_data.revision <= 2) {
		crc_data.cdu_u22_seqeunces_crc = CRC_CAL(crc_data.usage_content, 20);
		crc_data.cdu_u43_hotspots_crc = CRC_CAL(crc_data.usage_content, 24);
		crc_data.cdu_u93_profile_crc = CRC_CAL(crc_data.usage_content, 28);
		crc_data.cdu_u94_delta_scale_map_crc = CRC_CAL(crc_data.usage_content, 32);
		crc_data.runtime_hash = CRC_CAL(crc_data.usage_content, 36);
	} else if (crc_data.revision <= 3) { // revision 3
		crc_data.cdu_u22_seqeunces_crc = CRC_CAL(crc_data.usage_content, 20);
		crc_data.cdu_u43_hotspots_crc = CRC_CAL(crc_data.usage_content, 24);
		crc_data.cdu_u77_dials_crc = CRC_CAL(crc_data.usage_content, 28);
		crc_data.cdu_u93_profile_crc = CRC_CAL(crc_data.usage_content, 32);
		crc_data.cdu_u94_delta_scale_map_crc = CRC_CAL(crc_data.usage_content, 36);
		crc_data.runtime_hash = CRC_CAL(crc_data.usage_content, 40);
	} else	{
		dev_err(data_core->pDev, "Config revision is updated, please update the code to handle it");
	}
	return crc_data;
}
EXPORT_SYMBOL_GPL(get_u33_crc_data);

/*
 * Checks if a usage should be skipped during config flashing.
 */
bool ignore_usage_list(struct axiom_data_core *data_core, u8 usage)
{
	int i;
	u8 list[] = {0x31, 0x32, 0x33, 0x36, 0x82};

	// These usages are ignored/skipped during config update
	for (i = 0; i < 5; i++) {
		if (usage == list[i]) {
			dev_info(data_core->pDev, "%s = %x\n", __func__, usage);
			return true;
		}
	}
	return false;
}
EXPORT_SYMBOL_GPL(ignore_usage_list);

/*
 * Checks if a usage is a CDU type.
 */
bool cdu_usage_list(struct axiom_data_core *data_core, u8 usage)
{
	int i;
	u8 list[] = {0x05, 0x22, 0x43, 0x93, 0x94};

	// CDU usages require special handling
	for (i = 0; i < 5; i++) {
		if (usage == list[i]) {
			dev_info(data_core->pDev, "%s = %x\n", __func__, usage);
			return true;
		}
	}
	return false;
}
EXPORT_SYMBOL_GPL(cdu_usage_list);

/*
 * Writes CDU data to the device in chunks, with retries and status checks.
 */
u16 cdu_store(struct axiom_data_core *data_core, u8 usage, u16 length, u8 *pBuffer)
{
	int i, retry = 0;
	u8 command, status;
	u16 offset;
	u16 target_address = usage_to_target_address(data_core, usage, 0, 0);
	u8 *cdu_buffer;

	cdu_buffer = kcalloc(CDU_XFER_SIZE_FULL, sizeof(u8), GFP_KERNEL);
	offset = 0;
	command = CDU_CMD_STORE;

	while (offset < length) {
		// Set the command in the buffer
		cdu_buffer[0] = (command & 0x00FF);
		cdu_buffer[1] = ((command & 0xFF00) >> 8);

		// Set the offset into the buffer
		cdu_buffer[4] = (offset & 0x00FF);
		cdu_buffer[5] = ((offset & 0xFF00) >> 8);

		// Copy data chunk to buffer, pad with zeros if needed
		if ((offset + CDU_XFER_SIZE) < length)
			for (i = 0; i < CDU_XFER_SIZE; i++)
				cdu_buffer[i + 8] = pBuffer[offset+i];
		else
			for (i = 0; i < CDU_XFER_SIZE; i++)
				if ((offset + i) >= length)
					cdu_buffer[i + 8] = 0;
				else
					cdu_buffer[i + 8] = pBuffer[offset + i];

		if (axiom_write_page(data_core->pAxiomData, target_address, CDU_XFER_SIZE_FULL, cdu_buffer) != CDU_XFER_SIZE_FULL) {
			dev_info(data_core->pDev, "%s failed to write\n", __func__);
			return -EINVAL;
		}

		// Wait for device to process the chunk, retry if busy
		while (retry < 10) {
			msleep(1000);
			if (axiom_read_page(data_core->pAxiomData, target_address, CDU_XFER_SIZE_FULL, cdu_buffer) != CDU_XFER_SIZE_FULL) {
				dev_info(data_core->pDev, "%s failed to read\n", __func__);
				return -EINVAL;
			}
			status = cdu_buffer[0] | (cdu_buffer[1] << 8);
			if (status == 0) {
				dev_info(data_core->pDev, "%s Data was successfully transferred", __func__);
				break;
			} else if ((status & CDU_ERROR_MASK) != 0) {
				dev_err(data_core->pDev, "%s ERROR: CDU Store Failure! Status: %x", __func__, status);
				kfree(cdu_buffer);
				return -EINVAL;
			}
			dev_info(data_core->pDev, "%s aXiom is still processing the request %d", __func__, retry);
			retry++;
		}
		if (retry == 10) {
			dev_info(data_core->pDev, "%s failed and timed out,\n", __func__);
			kfree(cdu_buffer);
			return -EINVAL;
		}
		offset += CDU_XFER_SIZE;
	}
	kfree(cdu_buffer);
	return length;
}
EXPORT_SYMBOL_GPL(cdu_store);

/*
 * Commits CDU data to the device, with retries and status checks.
 */
u16 cdu_commit(struct axiom_data_core *data_core, u8 usage, u16 length)
{
	int retry = 0;
	u8 command, status;
	u16 target_address = usage_to_target_address(data_core, usage, 0, 0);
	u8 *cdu_buffer;

	cdu_buffer = kcalloc(CDU_XFER_SIZE_FULL, sizeof(u8), GFP_KERNEL);

	command = CDU_CMD_COMMIT;
	cdu_buffer[0] = (command & 0x00FF);
	cdu_buffer[1] = ((command & 0xFF00) >> 8);
	cdu_buffer[2] = 0x0C;
	cdu_buffer[3] = 0xB1;
	cdu_buffer[4] = 0xDE;
	cdu_buffer[5] = 0xC0;

	if (axiom_write_page(data_core->pAxiomData, target_address, CDU_XFER_SIZE_FULL, cdu_buffer) != CDU_XFER_SIZE_FULL) {
		dev_info(data_core->pDev, "%s failed to write\n", __func__);
		return -EINVAL;
	}

	// Wait for commit to complete, retry if busy
	while (retry < 10) {
		msleep(1000);
		if (axiom_read_page(data_core->pAxiomData, target_address, CDU_XFER_SIZE_FULL, cdu_buffer) != CDU_XFER_SIZE_FULL) {
			dev_info(data_core->pDev, "%s failed to read\n", __func__);
			return -EINVAL;
		}
		status = cdu_buffer[0] | (cdu_buffer[1] << 8);
		if (status == 0) {
			dev_info(data_core->pDev, "%s Data was successfully transferred", __func__);
			break;
		} else if ((status & CDU_ERROR_MASK) != 0) {
			dev_err(data_core->pDev, "%s ERROR: CDU Store Failure! Status: %x", __func__, status);
			kfree(cdu_buffer);
			return -EINVAL;
		}
		dev_info(data_core->pDev, "%s aXiom is still processing the request %d", __func__, retry);
		retry++;
		}
	if (retry == 10) {
		dev_info(data_core->pDev, "%s failed and timed out,\n", __func__);
		kfree(cdu_buffer);
		return -EINVAL;
	}
	kfree(cdu_buffer);
	return length;
}
EXPORT_SYMBOL_GPL(cdu_commit);

/*
 * Writes CDU data to the device.
 */
u16 cdu_write(struct axiom_data_core *data_core, u8 usage, u16 length, u8 *pBuffer)
{
	u16 ret;

	ret = cdu_store(data_core, usage, length, pBuffer);
	if (!ret)
		dev_info(data_core->pDev, "In %s cdu_store failed\n", __func__);

	ret = cdu_commit(data_core, usage, length);
	if (!ret)
		dev_info(data_core->pDev, "In %s cdu_commit failed\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(cdu_write);

/*
 * Reads and prints CRC data from the device for verification.
 */
bool axiom_cfg_checksum(struct axiom_data_core *data_core)
{
	struct u33_CRCData device_crc_data = {0};

	device_crc_data = get_target_crc_data(data_core, device_crc_data);
	printU33Data(data_core, device_crc_data);
	return 0;
}
EXPORT_SYMBOL_GPL(axiom_cfg_checksum);

MODULE_AUTHOR("Karthik Choda <karthikchoda0110@gmail.com>");
MODULE_AUTHOR("Varun Rajesh <ee22b069@smail.iitm.ac.in>");
MODULE_DESCRIPTION("aXiom Configration update");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:axiom");
