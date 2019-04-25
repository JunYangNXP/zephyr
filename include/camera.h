/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for camera drivers and applications
 */

#ifndef ZEPHYR_INCLUDE_CAMERA_H_
#define ZEPHYR_INCLUDE_CAMERA_H_

#include <device.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum camera_pixel_format {
	CAMERA_PIXEL_FORMAT_RGB_888		= BIT(0),
	CAMERA_PIXEL_FORMAT_YUV		= BIT(1),
	CAMERA_PIXEL_FORMAT_RGB_565		= BIT(2),
};

struct camera_buffer_descriptor {
	u16_t width;
	u16_t height;
	u16_t bpp;
};

typedef void (*camera_capture_cb)(void *fb, int w, int h, int bpp);

typedef int (*camera_capture_api)(const struct device *dev, camera_capture_cb cb);

typedef int (*camera_setformat_api)(const struct device *dev, enum camera_pixel_format format);

typedef void *(*camera_get_framebuffer_api)(const struct device *dev, int *w, int *h, int *bpp);

struct camera_driver_api {
	camera_capture_api capture;
	camera_setformat_api set_format;
	camera_get_framebuffer_api get_framebuffer;
};

static inline int camera_capture(const struct device *dev,
		camera_capture_cb cb)
{
	struct camera_driver_api *api =
		(struct camera_driver_api *)dev->driver_api;

	return api->capture(dev, cb);
}

static inline int camera_set_format(const struct device *dev,
		enum camera_pixel_format format)
{
	struct camera_driver_api *api =
		(struct camera_driver_api *)dev->driver_api;

	return api->set_format(dev, format);
}

static inline void *camera_get_framebuffer(const struct device *dev,
	int *width, int *height, int *bpp)
{
	struct camera_driver_api *api =
		(struct camera_driver_api *)dev->driver_api;

	return api->get_framebuffer(dev, width, height, bpp);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DISPLAY_H_*/
