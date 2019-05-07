/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
#include <device.h>

LOG_MODULE_REGISTER(sdhc_soc, CONFIG_DISK_LOG_LEVEL);

#include <disk_access.h>

extern status_t sd_write_blocks(void *hcard, const uint8_t *buffer,
	uint32_t startBlock, uint32_t blockCount);
extern int sd_read_blocks(void *hcard, uint8_t *buffer,
	uint32_t startBlock, uint32_t blockCount);

extern struct device *soc_sdhc_get_device(void);

static int soc_sdhc_read(void *data, u8_t *buf, u32_t sector,
		     u32_t count)
{
	int err;

	uint32_t usb_irq = __NVIC_GetEnableIRQ(USB_OTG1_IRQn);
	if (usb_irq)
		NVIC_DisableIRQ(USB_OTG1_IRQn);

	err = sd_read_blocks(data, buf, sector, count);
	if (usb_irq)
		NVIC_EnableIRQ(USB_OTG1_IRQn);

	return err;
}

static int soc_sdhc_write(void *data, const u8_t *buf, u32_t sector,
		      u32_t count)
{
	status_t ret;
	uint32_t usb_irq = __NVIC_GetEnableIRQ(USB_OTG1_IRQn);
	if (usb_irq)
		NVIC_DisableIRQ(USB_OTG1_IRQn);

	ret = sd_write_blocks(data, buf, sector, count);

	if (usb_irq)
		NVIC_EnableIRQ(USB_OTG1_IRQn);
	return ret;
}

static int disk_soc_sdhc_init(struct device *dev);

int soc_sdhc_init(struct device *dev)
{
	return disk_soc_sdhc_init(dev);
}

static int disk_soc_sdhc_status(struct disk_info *disk)
{
	return DISK_STATUS_OK;
}

static int disk_soc_sdhc_access_read(struct disk_info *disk, u8_t *buf,
				 u32_t sector, u32_t count)
{
	struct device *dev = soc_sdhc_get_device();
	int err;

	err = soc_sdhc_read(dev->driver_data, buf, sector, count);

	return err;
}

static int disk_soc_sdhc_access_write(struct disk_info *disk, const u8_t *buf,
				  u32_t sector, u32_t count)
{
	struct device *dev = soc_sdhc_get_device();
	int err;

	err = soc_sdhc_write(dev->driver_data, buf, sector, count);

	return err;
}

extern uint32_t sd_get_blk_count(void *card_info);
#define SOC_SDHC_SECTOR_SIZE 512

static int disk_soc_sdhc_access_ioctl(struct disk_info *disk, u8_t cmd, void *buf)
{
	struct device *dev = soc_sdhc_get_device();

	switch (cmd) {
	case DISK_IOCTL_CTRL_SYNC:
		break;
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(u32_t *)buf = sd_get_blk_count(dev->driver_data);
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(u32_t *)buf = SOC_SDHC_SECTOR_SIZE;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(u32_t *)buf = SOC_SDHC_SECTOR_SIZE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int disk_soc_sdhc_access_init(struct disk_info *disk)
{
	return 0;
}

static const struct disk_operations soc_sdhc_disk_ops = {
	.init = disk_soc_sdhc_access_init,
	.status = disk_soc_sdhc_status,
	.read = disk_soc_sdhc_access_read,
	.write = disk_soc_sdhc_access_write,
	.ioctl = disk_soc_sdhc_access_ioctl,
};

static struct disk_info soc_sdhc_disk = {
	.name = CONFIG_DISK_SOC_SDHC_VOLUME_NAME,
	.ops = &soc_sdhc_disk_ops,
};

static int disk_soc_sdhc_init(struct device *dev)
{
	return disk_access_register(&soc_sdhc_disk);
}
