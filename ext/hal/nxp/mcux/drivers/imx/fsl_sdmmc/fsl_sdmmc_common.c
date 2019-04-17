/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "fsl_sdmmc_common.h"

extern char __start_sd_global_seg[];
extern char __end_sd_global_seg;
uint32_t *g_sdmmc = (uint32_t *)__start_sd_global_seg;

extern usdhc_transfer_t *g_sd_content;
extern usdhc_command_t *g_sd_cmd;
extern usdhc_data_t *g_sd_data;

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t SDMMC_SelectCard(SDMMCHOST_TYPE *base,
		SDMMCHOST_TRANSFER_FUNCTION transfer,
		uint32_t relativeAddress,
		bool isSelected)
{
	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSDMMC_SelectCard;
	if (isSelected) {
		g_sd_cmd->argument = relativeAddress << 16U;
		g_sd_cmd->responseType = kCARD_ResponseTypeR1;
	} else {
		g_sd_cmd->argument = 0U;
		g_sd_cmd->responseType = kCARD_ResponseTypeNone;
	}

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = NULL;
	if ((kStatus_Success != transfer(base, g_sd_content)) ||
		(g_sd_cmd->response[0U] & kSDMMC_R1ErrorAllFlag))
		return kStatus_SDMMC_TransferFailed;

	/* Wait until card to transfer state */
	return kStatus_Success;
}

extern void printk(const char *fmt_s, ...);
status_t SDMMC_SendApplicationCommand(SDMMCHOST_TYPE *base,
		SDMMCHOST_TRANSFER_FUNCTION transfer,
		uint32_t relativeAddress)
{
	status_t ret;

	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSDMMC_ApplicationCommand;
	g_sd_cmd->argument = (relativeAddress << 16U);
	g_sd_cmd->responseType = kCARD_ResponseTypeR1;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = 0U;
	ret = transfer(base, g_sd_content);

	if ((kStatus_Success != ret) || (g_sd_cmd->response[0U] & kSDMMC_R1ErrorAllFlag))
		return kStatus_SDMMC_TransferFailed;

	if (!(g_sd_cmd->response[0U] & kSDMMC_R1ApplicationCommandFlag))
		return kStatus_SDMMC_CardNotSupport;

	return kStatus_Success;
}

status_t SDMMC_SetBlockCount(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER_FUNCTION transfer, uint32_t blockCount)
{
	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSDMMC_SetBlockCount;
	g_sd_cmd->argument = blockCount;
	g_sd_cmd->responseType = kCARD_ResponseTypeR1;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = 0U;
	if ((kStatus_Success != transfer(base, g_sd_content)) || (g_sd_cmd->response[0U] & kSDMMC_R1ErrorAllFlag))
		return kStatus_SDMMC_TransferFailed;

	return kStatus_Success;
}

status_t SDMMC_GoIdle(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER_FUNCTION transfer)
{
	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSDMMC_GoIdleState;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = 0U;
	if (kStatus_Success != transfer(base, g_sd_content))
		return kStatus_SDMMC_TransferFailed;

	return kStatus_Success;
}

status_t SDMMC_SetBlockSize(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER_FUNCTION transfer, uint32_t blockSize)
{
	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSDMMC_SetBlockLength;
	g_sd_cmd->argument = blockSize;
	g_sd_cmd->responseType = kCARD_ResponseTypeR1;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = 0U;
	if ((kStatus_Success != transfer(base, g_sd_content)) || (g_sd_cmd->response[0U] & kSDMMC_R1ErrorAllFlag))
		return kStatus_SDMMC_TransferFailed;

	return kStatus_Success;
}

status_t SDMMC_SetCardInactive(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER_FUNCTION transfer)
{
	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSDMMC_GoInactiveState;
	g_sd_cmd->argument = 0U;
	g_sd_cmd->responseType = kCARD_ResponseTypeNone;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = 0U;
	if ((kStatus_Success != transfer(base, g_sd_content)))
		return kStatus_SDMMC_TransferFailed;

	return kStatus_Success;
}

status_t SDMMC_SwitchVoltage(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER_FUNCTION transfer)
{
	assert(transfer);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));

	g_sd_cmd->index = kSD_VoltageSwitch;
	g_sd_cmd->argument = 0U;
	g_sd_cmd->responseType = kCARD_ResponseTypeR1;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = NULL;
	if (kStatus_Success != transfer(base, g_sd_content))
		return kStatus_SDMMC_TransferFailed;
	/* disable card clock */
	SDMMCHOST_ENABLE_CARD_CLOCK(base, false);

	/* check data line and cmd line status */
	if ((GET_SDMMCHOST_STATUS(base) &
		(CARD_DATA1_STATUS_MASK | CARD_DATA2_STATUS_MASK | CARD_DATA3_STATUS_MASK | CARD_DATA0_NOT_BUSY)) != 0U)
		return kStatus_SDMMC_SwitchVoltageFail;

	/* host switch to 1.8V */
	SDMMCHOST_SWITCH_VOLTAGE180V(base, true);

	SDMMCHOST_Delay(100U);

	/*enable sd clock*/
	SDMMCHOST_ENABLE_CARD_CLOCK(base, true);
	/*enable force clock on*/
	SDMMCHOST_FORCE_SDCLOCK_ON(base, true);
	/* dealy 1ms,not exactly correct when use while */
	SDMMCHOST_Delay(10U);
	/*disable force clock on*/
	SDMMCHOST_FORCE_SDCLOCK_ON(base, false);

	/* check data line and cmd line status */
	if ((GET_SDMMCHOST_STATUS(base) &
		(CARD_DATA1_STATUS_MASK | CARD_DATA2_STATUS_MASK | CARD_DATA3_STATUS_MASK | CARD_DATA0_NOT_BUSY)) == 0U)
		return kStatus_SDMMC_SwitchVoltageFail;

	return kStatus_Success;
}

status_t SDMMC_ExecuteTuning(SDMMCHOST_TYPE *base,
		SDMMCHOST_TRANSFER_FUNCTION transfer,
		uint32_t tuningCmd,
		uint32_t blockSize)
{
	uint32_t *buffer = (uint32_t *)((char *)g_sd_data + 512);
	bool tuningError = true;

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
	memset((char *)g_sd_data, 0, sizeof(struct _usdhc_data));

	g_sd_cmd->index = tuningCmd;
	g_sd_cmd->argument = 0U;
	g_sd_cmd->responseType = kCARD_ResponseTypeR1;

	g_sd_data->blockSize = blockSize;
	g_sd_data->blockCount = 1U;
	g_sd_data->rxData = buffer;
	/* add this macro for adpter to different driver */

	g_sd_data->dataType = kUSDHC_TransferDataTuning;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = g_sd_data;

	/* enable the standard tuning */
	SDMMCHOST_EXECUTE_STANDARD_TUNING_ENABLE(base, true);

	while (true) {
		/* send tuning block */
		if ((kStatus_Success != transfer(base, g_sd_content)))
			return kStatus_SDMMC_TransferFailed;
		SDMMCHOST_Delay(1U);

		/*wait excute tuning bit clear*/
		if ((SDMMCHOST_EXECUTE_STANDARD_TUNING_STATUS(base) != 0U))
			continue;

		/* if tuning error , re-tuning again */
		if ((SDMMCHOST_CHECK_TUNING_ERROR(base) != 0U) && tuningError) {
			tuningError = false;
			/* enable the standard tuning */
			SDMMCHOST_EXECUTE_STANDARD_TUNING_ENABLE(base, true);
			SDMMCHOST_ADJUST_TUNING_DELAY(base, SDMMCHOST_STANDARD_TUNING_START);
		} else {
			break;
		}
	}

	/* delay to wait the host controller stable */
	SDMMCHOST_Delay(100U);

	/* check tuning result*/
	if (SDMMCHOST_EXECUTE_STANDARD_TUNING_RESULT(base) == 0U)
		return kStatus_SDMMC_TuningFail;

	SDMMCHOST_AUTO_STANDARD_RETUNING_TIMER(base);

	return kStatus_Success;
}
