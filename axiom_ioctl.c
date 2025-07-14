// SPDX-License-Identifier: GPL-2.0
#include <linux/firmware.h>
#include <linux/ioctl.h>
#include "axiom_core.h"

/* IOCTL command definitions for user-space communication */
#define AXIOM_CFG_UPDATE _IOW('a', 'a', char*)
#define AXIOM_CFG_CHECKSUM _IOR('a', 'b', int32_t*)
#define AXIOM_FW_UPDATE _IOW('a', 'c', char*)
#define AXIOM_FW_CHECKSUM _IOR('a', 'd', char*)
#define AXIOM_FILENAME_MAXLEN 50

dev_t dev;
static struct class *dev_class;
static struct axiom_cdev axiom_cdev;

static int axiom_dev_open(struct inode *inode, struct file *file);
static int axiom_dev_release(struct inode *inode, struct file *file);
static long axiom_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations fops = {
	.owner          = THIS_MODULE,
	.open           = axiom_dev_open,
	.unlocked_ioctl = axiom_dev_ioctl,
	.release        = axiom_dev_release,
};
/*
 * This function will be called when we open the Device file
 */
static int axiom_dev_open(struct inode *inode, struct file *file)
{
	struct axiom_cdev *axiom_dev = container_of(inode->i_cdev, struct axiom_cdev, ioctl_cdev); // Access axiom_data_core

	dev_info(axiom_dev->axiom_data_core->pDev, "Device File Opened...!!!\n");
	return 0;
}
/*
 * This function will be called when we close the Device file
 */
static int axiom_dev_release(struct inode *inode, struct file *file)
{
	struct axiom_cdev *axiom_dev = container_of(inode->i_cdev, struct axiom_cdev, ioctl_cdev); // Access axiom_data_core

	dev_info(axiom_dev->axiom_data_core->pDev, "Device File Closed...!!!\n");
	return 0;
}

/*
 * Handles IOCTL commands from user space.
 * Used for firmware/configuration flashing, checksum retrieval, and file length setup.
 */
static long axiom_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	// Get driver context from file pointer
	struct axiom_cdev *axiom_dev = container_of(file->f_inode->i_cdev, struct axiom_cdev, ioctl_cdev); // Access axiom_data_core

	dev_info(axiom_dev->axiom_data_core->pDev, "%s fn is called\n", __func__);
	switch (cmd) {
	case AXIOM_CFG_UPDATE:
	{
		int result;
		const struct firmware *cfg;
		char filename[AXIOM_FILENAME_MAXLEN];

		// Copy filename from user space
		if (copy_from_user(filename, (char __user *)arg, AXIOM_FILENAME_MAXLEN - 1))
			return -EFAULT;
		filename[AXIOM_FILENAME_MAXLEN - 1] = '\0'; // Ensure null-termination

		// Load configuration file from filesystem
		dev_err(axiom_dev->axiom_data_core->pDev, "Loading config file %s\n", filename);
		result = request_firmware(&cfg, filename, axiom_dev->axiom_data_core->pDev);
		if (result) {
			dev_err(axiom_dev->axiom_data_core->pDev, "Failed to load config file %s\n", filename);
			return result;
		}
		// Flash configuration to device
		result = axiom_cfg_update(axiom_dev->axiom_data_core, (unsigned char *)cfg->data, cfg->size);
		release_firmware(cfg);
		if (!result) {
			dev_err(axiom_dev->axiom_data_core->pDev, "Configuration flashing failed %d\n", result);
			return result;
		}
		dev_info(axiom_dev->axiom_data_core->pDev, "configuration flashed successfully\n");
		break;
	}
	case AXIOM_FW_UPDATE:
	{
		int result;
		const struct firmware *fw;
		char filename[AXIOM_FILENAME_MAXLEN];

		// Copy filename from user space
		if (copy_from_user(filename, (char __user *)arg, AXIOM_FILENAME_MAXLEN - 1))
			return -EFAULT;
		filename[AXIOM_FILENAME_MAXLEN - 1] = '\0';

		// Load firmware file from filesystem
		dev_err(axiom_dev->axiom_data_core->pDev, "Loading firmware file %s\n", filename);
		result = request_firmware(&fw, filename, axiom_dev->axiom_data_core->pDev);
		if (result) {
			dev_err(axiom_dev->axiom_data_core->pDev, "Failed to load firmware %s\n", filename);
			return result;
		}

		// Flash configuration to device
		result = axiom_fw_update(axiom_dev->axiom_data_core, (unsigned char *)fw->data, fw->size);
		release_firmware(fw);
		if (!result) {
			dev_err(axiom_dev->axiom_data_core->pDev, "Firmware flashing failed %d\n", result);
			return result;
		}
		dev_info(axiom_dev->axiom_data_core->pDev, "Firmware flashed successfully\n");
		break;
	}
	case AXIOM_CFG_CHECKSUM:
	{
		int checksum;

		// Get config checksum from device and send to user space
		checksum = axiom_cfg_checksum(axiom_dev->axiom_data_core);
		if (copy_to_user((int *)arg, &checksum, sizeof(checksum)))
			return -EFAULT;
		dev_info(axiom_dev->axiom_data_core->pDev, "Sent message to user: %d\n", checksum);
		break;
	}
	case AXIOM_FW_CHECKSUM:
	{
		const char *checksum;
		u8 fw_major;
		u8 fw_minor;

		// Compose firmware version string as checksum
		fw_major = axiom_dev->axiom_data_core->u31_Info.fw_major;
		fw_minor = axiom_dev->axiom_data_core->u31_Info.fw_minor;

		checksum = kasprintf(GFP_KERNEL, "%u.%02u", (unsigned int)fw_major, (unsigned int)fw_minor);

		if (!checksum)
			return -ENOMEM;

		if (copy_to_user((char *)arg, checksum, strlen(checksum) + 1)) {
			kfree(checksum);
			return -EFAULT;
		}

		dev_info(axiom_dev->axiom_data_core->pDev, "FW CRC Checksum sent to user space: %u.%02u\n", fw_major, fw_minor);
		break;
	}
	default:
	{
		dev_err(axiom_dev->axiom_data_core->pDev, "Default\n");
		break;
	}
	}
	return 0;
}

/*
 * Initializes the character device, registers it with the kernel, and creates the device node.
 */
int axiom_dev_init(struct axiom_data_core *data_core)
{
	/*Allocating Major number*/
	if ((alloc_chrdev_region(&dev, 0, 1, "axiom_dev")) < 0) {
		dev_info(data_core->pDev, "Couldn't allocate major number\n");
		return -ENOMEM;
	}
	dev_info(data_core->pDev, "Major = %d Minor = %d\n", MAJOR(dev), MINOR(dev));

	/*Creating cdev structure*/
	cdev_init(&axiom_cdev.ioctl_cdev, &fops);

	/*Adding character device to the system*/
	if ((cdev_add(&axiom_cdev.ioctl_cdev, dev, 1)) < 0) {
		dev_err(data_core->pDev, "Couldn't add the device to the system\n");
		goto r_class;
	}

	axiom_cdev.axiom_data_core = data_core;

	/*Creating struct class*/
	#if KERNEL_VERSION(6,4, 0) <= LINUX_VERSION_CODE
	dev_class = class_create("axiom_class");
	#else
	dev_class = class_create(THIS_MODULE, "axiom_class");
	#endif

	if (IS_ERR(dev_class)) {
		dev_err(data_core->pDev, "Couldn't create struct class\n");
		goto r_class;
	}
	/*Creating device*/
	if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "axiom_device"))) {
		dev_err(data_core->pDev, "Couldn't create Device 1\n");
		goto r_device;
	}
	dev_info(data_core->pDev, "Device Driver Insertion...Done!!!\n");
	return 0;

r_device:
	class_destroy(dev_class);

r_class:
	unregister_chrdev_region(dev, 1);
	return -ENOMEM;
}

/*
 * Cleans up and unregisters the character device and associated resources.
 */
void axiom_dev_exit(struct axiom_data_core *data_core)
{
	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&axiom_cdev.ioctl_cdev);
	unregister_chrdev_region(dev, 1);
	dev_info(data_core->pDev, "Device Driver Removal...Done!!!\n");
}

MODULE_AUTHOR("Karthik.Choda <karthikchoda0110@gmail.com>");
MODULE_AUTHOR("Varun Rajesh <ee22b069@smail.iitm.ac.in>");
MODULE_DESCRIPTION("aXiom Firmware and Configration update");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:axiom");
