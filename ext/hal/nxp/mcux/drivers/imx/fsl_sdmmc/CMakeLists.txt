#
# Copyright (c) 2018, NXP
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

zephyr_sources_ifdef(CONFIG_DISK_ACCESS_SDHC_CONTROLLER    fsl_sd.c)
zephyr_sources_ifdef(CONFIG_DISK_ACCESS_SDHC_CONTROLLER    fsl_sdmmc_common.c)
zephyr_sources_ifdef(CONFIG_DISK_ACCESS_SDHC_CONTROLLER    fsl_sdmmc_event.c)
zephyr_sources_ifdef(CONFIG_DISK_ACCESS_SDHC_CONTROLLER    fsl_sdmmc_host.c)
