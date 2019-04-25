/*
 * Copyright (c) 2019, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <camera.h>
#include "fsl_common.h"

#include <logging/log.h>

#define MCUX_CSI_POOL_BLOCK_MAX (512 * 512 * 2)
#define MCUX_CSI_POOL_BLOCK_MIN MCUX_CSI_POOL_BLOCK_MAX
#define MCUX_CSI_POOL_BLOCK_NUM 4
#define MCUX_CSI_POOL_BLOCK_ALIGN 64

#define MCUX_CSI_BPP 2
#define MCUX_CSI_CAMERA_HEIGHT 272
#define MCUX_CSI_CAMERA_WIDTH 480

#define CSI_CSICR1_INT_EN_MASK 0xFFFF0000U
#define CSI_CSICR3_INT_EN_MASK 0x000000FFU
#define CSI_CSICR18_INT_EN_MASK 0x0000FF00U


struct priv_csi_config {
	u32_t polarity;
	bool sensor_vsync;
	/*!< In CCIR656 progressive mode, set true to use external VSYNC signal, set false
	to use internal VSYNC signal decoded from SOF. */
};

enum priv_csi_polarity_flags {
	CSI_HSYNC_LOW = 0U,                           /*!< HSYNC is active low. */
	CSI_HSYNC_HIGH = CSI_CSICR1_HSYNC_POL_MASK,   /*!< HSYNC is active high. */
	CSI_RISING_LATCH = CSI_CSICR1_REDGE_MASK, /*!< Pixel data latched at rising edge of pixel clock. */
	CSI_FALLING_LATCH = 0U,                   /*!< Pixel data latched at falling edge of pixel clock. */
	CSI_VSYNC_HIGH = 0U,                          /*!< VSYNC is active high. */
	CSI_VSYNC_LOW = CSI_CSICR1_SOF_POL_MASK,      /*!< VSYNC is active low. */
};

enum priv_csi_fifo {
	CSI_RXFIFO = (1U << 0U),   /*!< RXFIFO. */
	CSI_STATFIFO = (1U << 1U), /*!< STAT FIFO. */
	CSI_ALLFIFO = (CSI_RXFIFO | CSI_STATFIFO)
};


K_MEM_POOL_DEFINE(mcux_csi_pool,
		MCUX_CSI_POOL_BLOCK_MIN,
		MCUX_CSI_POOL_BLOCK_MAX,
		MCUX_CSI_POOL_BLOCK_NUM,
		MCUX_CSI_POOL_BLOCK_ALIGN);

struct mcux_csi_config {
	CSI_Type *base;
	void (*irq_config_func)(struct device *dev);
};

struct mcux_csi_fb_attr {
	int width;
	int height;
	int bpp;
};

#define CSI_FB_NUM 2

struct mcux_csi_data {
	struct k_mem_block fb[CSI_FB_NUM];
	struct k_sem sem;
	volatile uint8_t current_idx;
	volatile uint8_t total_idx;
	struct mcux_csi_fb_attr fb_attr[CSI_FB_NUM];
	camera_capture_cb customer_cb;
	enum camera_pixel_format format;
};

static int csi_start(CSI_Type *base, struct mcux_csi_data *data)
{
	u32_t cr3 = 0U;

	base->CSICR18 = (base->CSICR18 & ~CSI_CSICR18_MASK_OPTION_MASK) | CSI_CSICR18_MASK_OPTION(0) |
		CSI_CSICR18_BASEADDR_SWITCH_SEL_MASK | CSI_CSICR18_BASEADDR_SWITCH_EN_MASK;

	base->CSIDMASA_FB1 = (u32_t)data->fb[0].data;
	base->CSIDMASA_FB2 = (u32_t)data->fb[1].data;
	

	/* After reflash DMA, the CSI saves frame to frame buffer 0. */
	cr3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;
	base->CSICR3 |= cr3;

	while (base->CSICR3 & cr3)
		;

	/*Enable isr*/
	base->CSICR1 |= (CSI_CSICR1_FB1_DMA_DONE_INTEN_MASK & CSI_CSICR1_INT_EN_MASK);
	base->CSICR3 |= (CSI_CSICR1_FB1_DMA_DONE_INTEN_MASK & CSI_CSICR3_INT_EN_MASK);
	base->CSICR18 |= ((CSI_CSICR1_FB1_DMA_DONE_INTEN_MASK & CSI_CSICR18_INT_EN_MASK) >> 6U);

	/*Start capture*/
	cr3 = CSI_CSICR3_DMA_REQ_EN_RFF_MASK;
	base->CSICR3 |= cr3;
	base->CSICR18 |= CSI_CSICR18_CSI_ENABLE_MASK;

	return 0;
}

static int mcux_csi_capture(const struct device *dev,
			camera_capture_cb cb)
{
	const struct mcux_csi_config *config = dev->config->config_info;
	struct mcux_csi_data *data = dev->driver_data;

	k_sem_take(&data->sem, K_FOREVER);
	data->customer_cb = (void *)cb;
	csi_start(config->base, data);

	return 0;
}

static int mcux_csi_set_format(const struct device *dev,
			enum camera_pixel_format format)
{
	struct mcux_csi_data *data = dev->driver_data;

	data->format = format;
	if (format == CAMERA_PIXEL_FORMAT_YUV) {
		data->fb_attr[0].bpp = 1;
		data->fb_attr[1].bpp = 1;
	} else if (format == CAMERA_PIXEL_FORMAT_RGB_565) {
		data->fb_attr[0].bpp = 2;
		data->fb_attr[1].bpp = 2;
	}
	return 0;
}

static void *mcux_csi_get_framebuffer(const struct device *dev, int *w, int *h, int *bpp)
{
	struct mcux_csi_data *data = dev->driver_data;

	k_sem_take(&data->sem, K_FOREVER);
	k_sem_give(&data->sem);
	assert(w && h && bpp);
	*w = data->fb_attr[0].width;
	*h = data->fb_attr[0].height;
	*bpp = data->fb_attr[0].bpp;
	return data->fb[0].data;
}

static void mcux_csi_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct mcux_csi_config *config = dev->config->config_info;
	struct mcux_csi_data *data = dev->driver_data;
	u32_t csisr = config->base->CSISR;

	/* Clear the error flags. */
	config->base->CSISR = csisr;

	/*Stop CSI*/
	config->base->CSICR18 &= ~CSI_CSICR18_CSI_ENABLE_MASK;
	config->base->CSICR3 &= ~CSI_CSICR3_DMA_REQ_EN_RFF_MASK;

	assert(csisr & (CSI_CSISR_DMA_TSF_DONE_FB2_MASK | CSI_CSISR_DMA_TSF_DONE_FB1_MASK));

	if (data->customer_cb)
		data->customer_cb(data->fb[data->current_idx].data, data->fb_attr[data->current_idx].width,
			data->fb_attr[data->current_idx].height, data->fb_attr[data->current_idx].bpp);

	k_sem_give(&data->sem);

}

static void csi_clear_fifo(CSI_Type *base, enum priv_csi_fifo fifo)
{
	u32_t cr1;
	u32_t mask = 0U;

	/* The FIFO could only be cleared when CSICR1[FCC] = 0, so first clear the FCC. */
	cr1 = base->CSICR1;
	base->CSICR1 = (cr1 & ~CSI_CSICR1_FCC_MASK);

	if ((u32_t)fifo & (u32_t)CSI_RXFIFO)
		mask |= CSI_CSICR1_CLR_RXFIFO_MASK;

	if ((u32_t)fifo & (u32_t)CSI_STATFIFO)
		mask |= CSI_CSICR1_CLR_STATFIFO_MASK;

	base->CSICR1 = (cr1 & ~CSI_CSICR1_FCC_MASK) | mask;

	/* Wait clear completed. */
	while (base->CSICR1 & mask)
		;

	/* Recover the FCC. */
	base->CSICR1 = cr1;
}

static void csi_reflash_fifo(CSI_Type *base, enum priv_csi_fifo fifo)
{
	u32_t cr3 = 0U;

	if ((u32_t)fifo & (u32_t)CSI_RXFIFO)
		cr3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;

	if ((u32_t)fifo & (u32_t)CSI_STATFIFO)
		cr3 |= CSI_CSICR3_DMA_REFLASH_SFF_MASK;

	base->CSICR3 |= cr3;

	/* Wait clear completed. */
	while (base->CSICR3 & cr3)
		;
}

static void csi_fifo_dma_enable(CSI_Type *base, enum priv_csi_fifo fifo, bool enable)
{
	u32_t cr3 = 0U;

	if ((u32_t)fifo & (u32_t)CSI_RXFIFO)
		cr3 |= CSI_CSICR3_DMA_REQ_EN_RFF_MASK;

	if ((u32_t)fifo & (u32_t)CSI_STATFIFO)
		cr3 |= CSI_CSICR3_DMA_REQ_EN_SFF_MASK;

	if (enable)
		base->CSICR3 |= cr3;
	else
		base->CSICR3 &= ~cr3;
}

static inline void csi_stop(CSI_Type *base)
{
	base->CSICR18 &= ~CSI_CSICR18_CSI_ENABLE_MASK;
	csi_fifo_dma_enable(base, CSI_RXFIFO, false);
}

static void csi_reset(CSI_Type *base)
{
	u32_t csisr;

	/* Disable transfer first. */
	csi_stop(base);

	/* Disable DMA request. */
	base->CSICR3 = 0U;

	/* Reset the fame count. */
	base->CSICR3 |= CSI_CSICR3_FRMCNT_RST_MASK;
	while (base->CSICR3 & CSI_CSICR3_FRMCNT_RST_MASK)
		;

	/* Clear the RX FIFO. */
	csi_clear_fifo(base, CSI_ALLFIFO);

	/* Reflash DMA. */
	csi_reflash_fifo(base, CSI_ALLFIFO);

	/* Clear the status. */
	csisr = base->CSISR;
	base->CSISR = csisr;

	/* Set the control registers to default value. */
	base->CSICR1 = CSI_CSICR1_HSYNC_POL_MASK | CSI_CSICR1_EXT_VSYNC_MASK;
	base->CSICR2 = 0U;
	base->CSICR3 = 0U;

	base->CSICR18 = CSI_CSICR18_AHB_HPROT(0x0DU);
	base->CSIFBUF_PARA = 0U;
	base->CSIIMAG_PARA = 0U;
}

static int csi_init(CSI_Type *base)
{
	u32_t reg;
	u32_t width_bytes;
	struct priv_csi_config csi_cfg;

	csi_cfg.sensor_vsync = true;

	csi_cfg.polarity = 0U;
	csi_cfg.polarity |= CSI_HSYNC_HIGH;
	csi_cfg.polarity |= CSI_RISING_LATCH;
	csi_cfg.polarity |= CSI_VSYNC_LOW;

	width_bytes = MCUX_CSI_CAMERA_WIDTH * MCUX_CSI_BPP;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
	CLOCK_EnableClock(kCLOCK_Csi);
#endif

	csi_reset(base);

	/*!< HSYNC, VSYNC, and PIXCLK signals are used. */
	reg = ((u32_t)CSI_CSICR1_GCLK_MODE(1U)) | csi_cfg.polarity | CSI_CSICR1_FCC_MASK;

	if (csi_cfg.sensor_vsync)
		reg |= CSI_CSICR1_EXT_VSYNC_MASK;

	base->CSICR1 = reg;

	/* Image parameter. */
	base->CSIIMAG_PARA = ((u32_t)(width_bytes) << CSI_CSIIMAG_PARA_IMAGE_WIDTH_SHIFT) |
		((u32_t)MCUX_CSI_CAMERA_HEIGHT << CSI_CSIIMAG_PARA_IMAGE_HEIGHT_SHIFT);

	/* The CSI frame buffer bus is 8-byte width. */
	base->CSIFBUF_PARA = 0;

	/* Enable auto ECC. */
	base->CSICR3 |= CSI_CSICR3_ECC_AUTO_EN_MASK;

	if (!(width_bytes % (8 * 16))) {
		base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
		base->CSICR3 = (base->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((2U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	} else if (!(width_bytes % (8 * 8))) {
		base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(2U);
		base->CSICR3 = (base->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((1U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	} else {
		base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(1U);
		base->CSICR3 = (base->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((0U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}

	/*Reflash DMA*/
	base->CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;

	/* Wait clear completed. */
	while (base->CSICR3 & CSI_CSICR3_DMA_REFLASH_RFF_MASK)
		;

	return 0;
}

static int mcux_csi_init(struct device *dev)
{
	const struct mcux_csi_config *config = dev->config->config_info;
	struct mcux_csi_data *data = dev->driver_data;
	int i;

	k_sem_init(&data->sem, 1, 1);
	data->current_idx = 0;
	data->total_idx = ARRAY_SIZE(data->fb);
	data->format = CAMERA_PIXEL_FORMAT_RGB_565;
	for (i = 0; i < ARRAY_SIZE(data->fb); i++) {
		if (k_mem_pool_alloc(&mcux_csi_pool, &data->fb[i],
				     MCUX_CSI_CAMERA_WIDTH * MCUX_CSI_CAMERA_HEIGHT * MCUX_CSI_BPP, K_NO_WAIT) != 0) {
			printk("Could not allocate frame buffer %d", i);
			return -ENOMEM;
		}
		data->fb_attr[i].width = MCUX_CSI_CAMERA_WIDTH;
		data->fb_attr[i].height = MCUX_CSI_CAMERA_HEIGHT;
		data->fb_attr[i].bpp = MCUX_CSI_BPP;
	}
	csi_init(config->base);
	config->irq_config_func(dev);

	return 0;
}

static const struct camera_driver_api mcux_camera_api = {
	.capture = mcux_csi_capture,
	.set_format = mcux_csi_set_format,
	.get_framebuffer = mcux_csi_get_framebuffer,
};

static void mcux_csi_config_func(struct device *dev);

static struct mcux_csi_config g_mcux_csi_config = {
	.base = (CSI_Type *)CSI,
	.irq_config_func = mcux_csi_config_func,
};

static struct mcux_csi_data g_mcux_csi_data;

DEVICE_AND_API_INIT(mcux_csi, CONFIG_CAMERA_DEV_NAME,
		&mcux_csi_init,
		&g_mcux_csi_data, &g_mcux_csi_config,
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		&mcux_camera_api);

static void mcux_csi_config_func(struct device *dev)
{
	IRQ_CONNECT(43, 0, mcux_csi_isr, DEVICE_GET(mcux_csi), 0);

	irq_enable(43);
}
