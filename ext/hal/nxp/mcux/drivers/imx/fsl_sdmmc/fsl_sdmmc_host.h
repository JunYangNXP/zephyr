/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_SDMMC_HOST_H
#define _FSL_SDMMC_HOST_H

#include "fsl_common.h"
#if defined(FSL_FEATURE_SOC_SDHC_COUNT) && FSL_FEATURE_SOC_SDHC_COUNT > 0U
#include "fsl_sdhc.h"
#elif defined(FSL_FEATURE_SOC_SDIF_COUNT) && FSL_FEATURE_SOC_SDIF_COUNT > 0U
#include "fsl_sdif.h"
#elif defined(FSL_FEATURE_SOC_USDHC_COUNT) && FSL_FEATURE_SOC_USDHC_COUNT > 0U
#include "fsl_usdhc.h"
#if (FSL_FEATURE_SOC_IOMUXC_COUNT != 0U)
#include "fsl_iomuxc.h"
#else
#include "fsl_port.h"
#endif
#endif

/*!
 * @addtogroup CARD
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Common definition for support and not support macro */
#define SDMMCHOST_NOT_SUPPORT 0U /*!< use this define to indicate the host not support feature*/
#define SDMMCHOST_SUPPORT 1U     /*!< use this define to indicate the host support feature*/

/* Common definition for board support SDR104/HS200/HS400 frequency */
/* SDR104 mode freq */
#if defined BOARD_SD_HOST_SUPPORT_SDR104_FREQ
#define SDMMCHOST_SUPPORT_SDR104_FREQ BOARD_SD_HOST_SUPPORT_SDR104_FREQ
#else
#define SDMMCHOST_SUPPORT_SDR104_FREQ SD_CLOCK_208MHZ
#endif
/* HS200 mode freq */
#if defined BOARD_SD_HOST_SUPPORT_HS200_FREQ
#define SDMMCHOST_SUPPORT_HS200_FREQ BOARD_SD_HOST_SUPPORT_HS200_FREQ
#else
#define SDMMCHOST_SUPPORT_HS200_FREQ MMC_CLOCK_HS200
#endif
/* HS400 mode freq */
#if defined BOARD_SD_HOST_SUPPORT_HS400_FREQ
#define SDMMCHOST_SUPPORT_HS400_FREQ BOARD_SD_HOST_SUPPORT_HS400_FREQ
#else
#define SDMMCHOST_SUPPORT_HS400_FREQ MMC_CLOCK_HS400
#endif

/* Common definition for SDMMCHOST transfer complete timeout */
#define SDMMCHOST_TRANSFER_COMPLETE_TIMEOUT (1000U)
/* Common definition for card detect timeout */
#define SDMMCHOST_CARD_DETECT_TIMEOUT (~0U)

/* Common definition for IRQ */
#if defined(__CORTEX_M)
#define SDMMCHOST_SET_IRQ_PRIORITY(id, priority) (NVIC_SetPriority(id, priority))
#else
#define SDMMCHOST_SET_IRQ_PRIORITY(id, priority) (GIC_SetPriority(id, priority))
#endif

#define SDMMCHOST_ENABLE_IRQ(id) (EnableIRQ(id))

/*********************************************************SDHC**********************************************************/
#if(defined(FSL_FEATURE_SOC_SDHC_COUNT) && (FSL_FEATURE_SOC_SDHC_COUNT > 0U))
#error not support
/*********************************************************USDHC**********************************************************/
#elif(defined(FSL_FEATURE_SOC_USDHC_COUNT) && (FSL_FEATURE_SOC_USDHC_COUNT > 0U))

/*define host baseaddr ,clk freq, IRQ number*/
#define BOARD_USDHC1_BASEADDR USDHC1
#define BOARD_USDHC2_BASEADDR USDHC2
#define BOARD_USDHC_CD_GPIO_BASE GPIO2
#define BOARD_USDHC_CD_GPIO_PIN 28
#define BOARD_USDHC_CD_PORT_IRQ GPIO2_Combined_16_31_IRQn

#define BOARD_USDHC1_CLK_FREQ (CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) / (CLOCK_GetDiv(kCLOCK_Usdhc1Div) + 1U))


#define BOARD_SD_HOST_BASEADDR BOARD_USDHC1_BASEADDR
#define BOARD_SD_HOST_CLK_FREQ BOARD_USDHC1_CLK_FREQ
#define BOARD_SD_HOST_IRQ USDHC1_IRQn

#define SD_HOST_BASEADDR BOARD_SD_HOST_BASEADDR
#define SD_HOST_CLK_FREQ BOARD_SD_HOST_CLK_FREQ
#define SD_HOST_IRQ BOARD_SD_HOST_IRQ

#define SDMMCHOST_TYPE USDHC_Type
#define SDMMCHOST_CONFIG usdhc_host_t
#define SDMMCHOST_TRANSFER usdhc_transfer_t
#define SDMMCHOST_COMMAND usdhc_command_t
#define SDMMCHOST_DATA usdhc_data_t
#define SDMMCHOST_BOOT_CONFIG usdhc_boot_config_t
#define CARD_DATA0_STATUS_MASK kUSDHC_Data0LineLevelFlag
#define CARD_DATA1_STATUS_MASK kUSDHC_Data1LineLevelFlag
#define CARD_DATA2_STATUS_MASK kUSDHC_Data2LineLevelFlag
#define CARD_DATA3_STATUS_MASK kUSDHC_Data3LineLevelFlag
#define CARD_DATA0_NOT_BUSY kUSDHC_Data0LineLevelFlag

#define SDMMCHOST_BUS_WIDTH_TYPE usdhc_data_bus_width_t
#define SDMMCHOST_CAPABILITY usdhc_capability_t

#define kSDMMCHOST_DATABUSWIDTH1BIT kUSDHC_DataBusWidth1Bit /*!< 1-bit mode */
#define kSDMMCHOST_DATABUSWIDTH4BIT kUSDHC_DataBusWidth4Bit /*!< 4-bit mode */
#define kSDMMCHOST_DATABUSWIDTH8BIT kUSDHC_DataBusWidth8Bit /*!< 8-bit mode */

#define SDMMCHOST_STANDARD_TUNING_START (10U) /*!< standard tuning start point */
#define SDMMCHOST_TUINIG_STEP (2U)            /*!< standard tuning step */
#define SDMMCHOST_RETUNING_TIMER_COUNT (0U)   /*!< Re-tuning timer */
#define SDMMCHOST_TUNING_DELAY_MAX (0x7FU)
#define SDMMCHOST_RETUNING_REQUEST kStatus_USDHC_ReTuningRequest
#define SDMMCHOST_TUNING_ERROR kStatus_USDHC_TuningError
/* define for card bus speed/strength cnofig */
#define CARD_BUS_FREQ_50MHZ (0U)
#define CARD_BUS_FREQ_100MHZ0 (1U)
#define CARD_BUS_FREQ_100MHZ1 (2U)
#define CARD_BUS_FREQ_200MHZ (3U)

#define CARD_BUS_STRENGTH_0 (0U)
#define CARD_BUS_STRENGTH_1 (1U)
#define CARD_BUS_STRENGTH_2 (2U)
#define CARD_BUS_STRENGTH_3 (3U)
#define CARD_BUS_STRENGTH_4 (4U)
#define CARD_BUS_STRENGTH_5 (5U)
#define CARD_BUS_STRENGTH_6 (6U)
#define CARD_BUS_STRENGTH_7 (7U)

#define SDMMCHOST_STROBE_DLL_DELAY_TARGET (7U)
#define SDMMCHOST_STROBE_DLL_DELAY_UPDATE_INTERVAL (4U)

/* function pointer define */
#define SDMMCHOST_TRANSFER_FUNCTION usdhc_transfer_function_t
#define GET_SDMMCHOST_CAPABILITY(base, capability) (USDHC_GetCapability(base, capability))
#define GET_SDMMCHOST_STATUS(base) (USDHC_GetPresentStatusFlags(base))
#define SDMMCHOST_SET_CARD_CLOCK(base, sourceClock_HZ, busClock_HZ) \
	(USDHC_SetSdClock(base, sourceClock_HZ, busClock_HZ))

#define BOARD_SD_PIN_CONFIG(speed, strength)                                                      \
{                                                                                             \
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,                                      \
		IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
		IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,                                      \
		IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
		IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,                                    \
		IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
		IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,                                    \
		IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
		IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,                                    \
		IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
		IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,                                    \
		IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
		IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
		IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
}

#define SDMMCHOST_ENABLE_CARD_CLOCK(base, enable)
#define SDMMCHOST_FORCE_SDCLOCK_ON(base, enable) (USDHC_ForceClockOn(base, enable))
#define SDMMCHOST_SET_CARD_BUS_WIDTH(base, busWidth) (USDHC_SetDataBusWidth(base, busWidth))
#define SDMMCHOST_SEND_CARD_ACTIVE(base, timeout) (USDHC_SetCardActive(base, timeout))
#define SDMMCHOST_SWITCH_VOLTAGE180V(base, enable18v) (UDSHC_SelectVoltage(base, enable18v))
#define SDMMCHOST_SWITCH_VOLTAGE120V(base, enable12v)
#define SDMMCHOST_CONFIG_SD_IO(speed, strength) BOARD_SD_PIN_CONFIG(speed, strength)
#define SDMMCHOST_CONFIG_MMC_IO(speed, strength) BOARD_MMC_PIN_CONFIG(speed, strength)
#define SDMMCHOST_SWITCH_VCC_TO_180V()
#define SDMMCHOST_SWITCH_VCC_TO_330V()

#if defined(FSL_FEATURE_USDHC_HAS_SDR50_MODE) && (!FSL_FEATURE_USDHC_HAS_SDR50_MODE)
#define SDMMCHOST_EXECUTE_STANDARD_TUNING_STATUS(base) (0U)
#define SDMMCHOST_EXECUTE_STANDARD_TUNING_RESULT(base) (1U)
#define SDMMCHOST_AUTO_STANDARD_RETUNING_TIMER(base)
#define SDMMCHOST_EXECUTE_STANDARD_TUNING_ENABLE(base, flag)
#define SDMMCHOST_CHECK_TUNING_ERROR(base) (0U)
#define SDMMCHOST_ADJUST_TUNING_DELAY(base, delay)
#else
#define SDMMCHOST_EXECUTE_STANDARD_TUNING_ENABLE(base, flag) \
(USDHC_EnableStandardTuning(base, SDMMCHOST_STANDARD_TUNING_START, SDMMCHOST_TUINIG_STEP, flag))
#define SDMMCHOST_EXECUTE_STANDARD_TUNING_STATUS(base) (USDHC_GetExecuteStdTuningStatus(base))
#define SDMMCHOST_EXECUTE_STANDARD_TUNING_RESULT(base) (USDHC_CheckStdTuningResult(base))
#define SDMMCHOST_AUTO_STANDARD_RETUNING_TIMER(base) (USDHC_SetRetuningTimer(base, SDMMCHOST_RETUNING_TIMER_COUNT))
#define SDMMCHOST_EXECUTE_MANUAL_TUNING_ENABLE(base, flag) (USDHC_EnableManualTuning(base, flag))
#define SDMMCHOST_ADJUST_TUNING_DELAY(base, delay) (USDHC_AdjustDelayForManualTuning(base, delay))
#define SDMMCHOST_AUTO_TUNING_ENABLE(base, flag) (USDHC_EnableAutoTuning(base, flag))
#define SDMMCHOST_CHECK_TUNING_ERROR(base) (USDHC_CheckTuningError(base))
#endif

#define SDMMCHOST_AUTO_TUNING_CONFIG(base) (USDHC_EnableAutoTuningForCmdAndData(base))
#define SDMMCHOST_RESET_TUNING(base, timeout)                                                      \
{                                                                                              \
	(USDHC_Reset(base, kUSDHC_ResetTuning | kUSDHC_ResetData | kUSDHC_ResetCommand, timeout)); \
}

#define SDMMCHOST_ENABLE_DDR_MODE(base, flag, nibblePos) (USDHC_EnableDDRMode(base, flag, nibblePos))

#if FSL_FEATURE_USDHC_HAS_HS400_MODE
#define SDMMCHOST_ENABLE_HS400_MODE(base, flag) (USDHC_EnableHS400Mode(base, flag))
#define SDMMCHOST_RESET_STROBE_DLL(base) (USDHC_ResetStrobeDLL(base))
#define SDMMCHOST_ENABLE_STROBE_DLL(base, flag) (USDHC_EnableStrobeDLL(base, flag))
#define SDMMCHOST_CONFIG_STROBE_DLL(base, delay, updateInterval) (USDHC_ConfigStrobeDLL(base, delay, updateInterval))
#define SDMMCHOST_GET_STROBE_DLL_STATUS (base)(USDHC_GetStrobeDLLStatus(base))
#else
#define SDMMCHOST_ENABLE_HS400_MODE(base, flag)
#define SDMMCHOST_RESET_STROBE_DLL(base)
#define SDMMCHOST_ENABLE_STROBE_DLL(base, flag)
#define SDMMCHOST_CONFIG_STROBE_DLL(base, delay, updateInterval)
#define SDMMCHOST_GET_STROBE_DLL_STATUS(base)
#endif

#define SDMMCHOST_ENABLE_MMC_BOOT(base, flag) (USDHC_EnableMmcBoot(base, flag))
#define SDMMCHOST_SETMMCBOOTCONFIG(base, config) (USDHC_SetMmcBootConfig(base, config))
/* sd card power */
#define BOARD_HAS_SDCARD (1U)
#define BOARD_SD_POWER_RESET_GPIO (GPIO1)
#define BOARD_SD_POWER_RESET_GPIO_PIN (5U)

#define BOARD_USDHC_CARD_INSERT_CD_LEVEL (0U)

#define BOARD_USDHC_MMCCARD_POWER_CONTROL(state)

#define BOARD_USDHC_MMCCARD_POWER_CONTROL_INIT()                                            \
{                                                                                       \
	gpio_pin_config_t sw_config = {                                                     \
		kGPIO_DigitalOutput, 0, kGPIO_NoIntmode,                                        \
	};                                                                                  \
	GPIO_PinInit(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, &sw_config); \
	GPIO_PinWrite(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, true);      \
}

#define BOARD_USDHC_SDCARD_POWER_CONTROL_INIT()                                             \
{                                                                                       \
	gpio_pin_config_t sw_config = {                                                     \
		kGPIO_DigitalOutput, 0, kGPIO_NoIntmode,                                        \
	};                                                                                  \
	GPIO_PinInit(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, &sw_config); \
}

extern void GPIO_PinWrite(GPIO_Type *base, uint32_t pin, uint8_t output);

#define BOARD_USDHC_SDCARD_POWER_CONTROL(state) \
	(GPIO_PinWrite(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, state))

#define SDMMCHOST_INIT_SD_POWER() BOARD_USDHC_SDCARD_POWER_CONTROL_INIT()
#define SDMMCHOST_ENABLE_SD_POWER(enable) BOARD_USDHC_SDCARD_POWER_CONTROL(enable)
/* mmc card power */
#define SDMMCHOST_INIT_MMC_POWER() BOARD_USDHC_MMCCARD_POWER_CONTROL_INIT()
#define SDMMCHOST_ENABLE_MMC_POWER(enable) BOARD_USDHC_MMCCARD_POWER_CONTROL(enable)
/* sd card detect through gpio */
#define BOARD_USDHC_CD_STATUS() (GPIO_PinRead(BOARD_USDHC_CD_GPIO_BASE, BOARD_USDHC_CD_GPIO_PIN))
#define BOARD_USDHC_CD_INTERRUPT_STATUS() (GPIO_PortGetInterruptFlags(BOARD_USDHC_CD_GPIO_BASE))
#define BOARD_USDHC_CD_CLEAR_INTERRUPT(flag) (GPIO_PortClearInterruptFlags(BOARD_USDHC_CD_GPIO_BASE, flag))

#define BOARD_USDHC_CD_GPIO_INIT()                                                          \
{                                                                                       \
	gpio_pin_config_t sw_config = {                                                     \
		kGPIO_DigitalInput, 0, kGPIO_IntRisingOrFallingEdge,                                    \
	};                                                                                  \
	GPIO_PinInit(BOARD_USDHC_CD_GPIO_BASE, BOARD_USDHC_CD_GPIO_PIN, &sw_config);        \
	GPIO_PortEnableInterrupts(BOARD_USDHC_CD_GPIO_BASE, 1U << BOARD_USDHC_CD_GPIO_PIN); \
	GPIO_PortClearInterruptFlags(BOARD_USDHC_CD_GPIO_BASE, ~0);                         \
}

#define SDMMCHOST_CARD_DETECT_GPIO_STATUS() BOARD_USDHC_CD_STATUS()
#define SDMMCHOST_CARD_DETECT_GPIO_INIT() BOARD_USDHC_CD_GPIO_INIT()
#define SDMMCHOST_CARD_DETECT_GPIO_INTERRUPT_STATUS() BOARD_USDHC_CD_INTERRUPT_STATUS()
#define SDMMCHOST_CARD_DETECT_GPIO_INTERRUPT_STATUS_CLEAR(flag) BOARD_USDHC_CD_CLEAR_INTERRUPT(flag)
#define SDMMCHOST_CARD_DETECT_GPIO_IRQ BOARD_USDHC_CD_PORT_IRQ
/* sd card detect through host CD */
#define SDMMCHOST_CARD_DETECT_INSERT_ENABLE(base) (USDHC_EnableInterruptStatus(base, kUSDHC_CardInsertionFlag))
#define SDMMCHOST_CARD_DETECT_REMOVE_ENABLE(base) (USDHC_EnableInterruptStatus(base, kUSDHC_CardRemovalFlag))
#define SDMMCHOST_CARD_DETECT_INSERT_STATUS(base) (USDHC_DetectCardInsert(base))
#define SDMMCHOST_CARD_DETECT_REMOVE_STATUS(base) (USDHC_GetInterruptStatusFlags(base, kUSDHC_CardRemovalFlag))
#define SDMMCHOST_CARD_DETECT_INSERT_INTERRUPT_ENABLE(base) \
	(USDHC_EnableInterruptSignal(base, kUSDHC_CardInsertionFlag))
#define SDMMCHOST_CARD_DETECT_REMOVE_INTERRUPT_ENABLE(base) (USDHC_EnableInterruptSignal(base, kUSDHC_CardRemovalFlag))
#define SDMMCHOST_CARD_DETECT_DATA3_ENABLE(base, flag) (USDHC_CardDetectByData3(base, flag))

/* define card detect pin voltage level when card inserted */
#if defined BOARD_USDHC_CARD_INSERT_CD_LEVEL
#define SDMMCHOST_CARD_INSERT_CD_LEVEL BOARD_USDHC_CARD_INSERT_CD_LEVEL
#else
#define SDMMCHOST_CARD_INSERT_CD_LEVEL (0U)
#endif
#define SDMMCHOST_ENABLE_TUNING_FLAG(data) (data.dataType = kUSDHC_TransferDataTuning)
#define SDMMCHOST_ENABLE_BOOT_FLAG(data) (data.dataType = kUSDHC_TransferDataBoot)
#define SDMMCHOST_ENABLE_BOOT_CONTINOUS_FLAG(data) (data.dataType = kUSDHC_TransferDataBootcontinous)
#define SDMMCHOST_GET_HOST_CONFIG_BLOCK_SIZE(config) (config->blockSize)
#define SDMMCHOST_GET_HOST_CONFIG_BLOCK_COUNT(config) (config->blockCount)
#define SDMMCHOST_GET_HOST_CONFIG_BOOT_MODE(config) (config->bootMode)
#define SDMMCHOST_EMPTY_CMD_FLAG(command) (command.type = kCARD_CommandTypeEmpty)
/*! @brief USDHC host capability*/
enum _host_capability
{
	kSDMMCHOST_SupportAdma = kUSDHC_SupportAdmaFlag,
	kSDMMCHOST_SupportHighSpeed = kUSDHC_SupportHighSpeedFlag,
	kSDMMCHOST_SupportDma = kUSDHC_SupportDmaFlag,
	kSDMMCHOST_SupportSuspendResume = kUSDHC_SupportSuspendResumeFlag,
	kSDMMCHOST_SupportV330 = kUSDHC_SupportV330Flag, /* this define should depend on your board config */
	kSDMMCHOST_SupportV300 = kUSDHC_SupportV300Flag, /* this define should depend on your board config */
#if defined(BOARD_SD_SUPPORT_180V) && !BOARD_SD_SUPPORT_180V
	kSDMMCHOST_SupportV180 = SDMMCHOST_NOT_SUPPORT, /* this define should depend on you board config */
#else
	kSDMMCHOST_SupportV180 = kUSDHC_SupportV180Flag, /* this define should depend on you board config */
#endif
	kSDMMCHOST_SupportV120 = SDMMCHOST_NOT_SUPPORT,
	kSDMMCHOST_Support4BitBusWidth = kUSDHC_Support4BitFlag,
#if defined(BOARD_MMC_SUPPORT_8BIT_BUS)
#if BOARD_MMC_SUPPORT_8BIT_BUS
	kSDMMCHOST_Support8BitBusWidth = kUSDHC_Support8BitFlag,
#else
	kSDMMCHOST_Support8BitBusWidth = SDMMCHOST_NOT_SUPPORT,
#endif
#else
	kSDMMCHOST_Support8BitBusWidth = kUSDHC_Support8BitFlag,
#endif
	kSDMMCHOST_SupportDDR50 = kUSDHC_SupportDDR50Flag,
	kSDMMCHOST_SupportSDR104 = kUSDHC_SupportSDR104Flag,
	kSDMMCHOST_SupportSDR50 = kUSDHC_SupportSDR50Flag,
	kSDMMCHOST_SupportHS200 = kUSDHC_SupportSDR104Flag,
#if FSL_FEATURE_USDHC_HAS_HS400_MODE
	kSDMMCHOST_SupportHS400 = SDMMCHOST_SUPPORT
#else
	kSDMMCHOST_SupportHS400 = SDMMCHOST_NOT_SUPPORT,
#endif
};

/* Endian mode. */
#define USDHC_ENDIAN_MODE kUSDHC_EndianModeLittle

/* DMA mode */
#define USDHC_DMA_MODE kUSDHC_DmaModeAdma2
/* address align */
#define SDMMCHOST_DMA_BUFFER_ADDR_ALIGN (USDHC_ADMA2_ADDRESS_ALIGN)

/* Read/write watermark level. The bigger value indicates DMA has higher read/write performance. */
#define USDHC_READ_WATERMARK_LEVEL (0x80U)
#define USDHC_WRITE_WATERMARK_LEVEL (0x80U)

/* ADMA table length united as word.
 *
 * One ADMA2 table item occupy two words which can transfer maximum 0xFFFFU bytes one time.
 * The more data to be transferred in one time, the bigger value of SDHC_ADMA_TABLE_WORDS need to be set.
 */
#define USDHC_ADMA_TABLE_WORDS (8U) /* define the ADMA descriptor table length */
#define USDHC_ADMA2_ADDR_ALIGN (4U) /* define the ADMA2 descriptor table addr align size */
#define USDHC_READ_BURST_LEN (8U)   /*!< number of words USDHC read in a single burst */
#define USDHC_WRITE_BURST_LEN (8U)  /*!< number of words USDHC write in a single burst */
#define USDHC_DATA_TIMEOUT (0xFU)   /*!< data timeout counter value */

#endif /* (defined(FSL_FEATURE_SOC_SDHC_COUNT) && (FSL_FEATURE_SOC_SDHC_COUNT > 0U)) */

/*! @brief card detect callback definition */
typedef void (*sdmmchost_cd_callback_t)(bool isInserted, void *userData);

/*! @brief host Endian mode
* corresponding to driver define
*/
enum _sdmmchost_endian_mode {
	kSDMMCHOST_EndianModeBig = 0U,         /*!< Big endian mode */
	kSDMMCHOST_EndianModeHalfWordBig = 1U, /*!< Half word big endian mode */
	kSDMMCHOST_EndianModeLittle = 2U,      /*!< Little endian mode */
};

/*! @brief sd card detect type */
typedef enum _sdmmchost_detect_card_type {
	kSDMMCHOST_DetectCardByGpioCD,    /*!< sd card detect by CD pin through GPIO */
	kSDMMCHOST_DetectCardByHostCD,    /*!< sd card detect by CD pin through host */
	kSDMMCHOST_DetectCardByHostDATA3, /*!< sd card detect by DAT3 pin through host */
} sdmmchost_detect_card_type_t;

/*! @brief sd card detect */
typedef struct _sdmmchost_detect_card {
	sdmmchost_detect_card_type_t cdType; /*!< card detect type */
	uint32_t cdTimeOut_ms; /*!< card detect timeout which allow 0 - 0xFFFFFFF, value 0 will return immediately, value
		0xFFFFFFFF will block until card is insert */

	sdmmchost_cd_callback_t cardInserted; /*!< card inserted callback which is meaningful for interrupt case */
	sdmmchost_cd_callback_t cardRemoved;  /*!< card removed callback which is meaningful for interrupt case */

	void *userData; /*!< user data */
} sdmmchost_detect_card_t;

/*! @brief card power control function pointer */
typedef void (*sdmmchost_pwr_t)(void);

/*! @brief card power control */
typedef struct _sdmmchost_pwr_card {
	sdmmchost_pwr_t powerOn;  /*!< power on function pointer */
	uint32_t powerOnDelay_ms; /*!< power on delay */

	sdmmchost_pwr_t powerOff;  /*!< power off function pointer */
	uint32_t powerOffDelay_ms; /*!< power off delay */
} sdmmchost_pwr_card_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name adaptor function
 * @{
 */

/*!
 * @brief host not support function, this function is used for host not support feature
 * @param  void parameter ,used to avoid build warning
 * @retval kStatus_Fail ,host do not suppport
 */
static inline status_t SDMMCHOST_NotSupport(void *parameter)
{
	parameter = parameter;
	return kStatus_Success;
}

/*!
 * @brief Detect card insert, only need for SD cases.
 * @param base the pointer to host base address
 * @param cd card detect configuration
 * @param waitCardStatus status which user want to wait
 * @retval kStatus_Success detect card insert
 * @retval kStatus_Fail card insert event fail
 */
status_t SDMMCHOST_WaitCardDetectStatus(SDMMCHOST_TYPE *hostBase,
		const sdmmchost_detect_card_t *cd,
		bool waitCardStatus);

/*!
 * @brief check card is present or not.
 * @retval true card is present
 * @retval false card is not present
 */
bool SDMMCHOST_IsCardPresent(void);

/*!
 * @brief Init host controller.
 * @param host the pointer to host structure in card structure.
 * @param userData specific user data
 * @retval kStatus_Success host init success
 * @retval kStatus_Fail event fail
 */
status_t SDMMCHOST_Init(SDMMCHOST_CONFIG *host, void *userData);

/*!
 * @brief reset host controller.
 * @param host base address.
 */
void SDMMCHOST_Reset(SDMMCHOST_TYPE *base);

/*!
 * @brief host controller error recovery.
 * @param host base address.
 */
void SDMMCHOST_ErrorRecovery(SDMMCHOST_TYPE *base);

/*!
 * @brief Deinit host controller.
 * @param host the pointer to host structure in card structure.
 */
void SDMMCHOST_Deinit(void *host);

/*!
 * @brief host power off card function.
 * @param base host base address.
 * @param pwr depend on user define power configuration.
 */
void SDMMCHOST_PowerOffCard(SDMMCHOST_TYPE *base, const sdmmchost_pwr_card_t *pwr);

/*!
 * @brief host power on card function.
 * @param base host base address.
 * @param pwr depend on user define power configuration.
 */
void SDMMCHOST_PowerOnCard(SDMMCHOST_TYPE *base, const sdmmchost_pwr_card_t *pwr);

/*!
 * @brief SDMMC host delay function.
 * @param milliseconds delay counter.
 */
void SDMMCHOST_Delay(uint32_t milliseconds);

/* @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_SDMMC_HOST_H */
