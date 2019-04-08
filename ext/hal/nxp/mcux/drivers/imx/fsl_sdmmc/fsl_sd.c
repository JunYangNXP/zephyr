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

#include "fsl_sd.h"
#include <misc/printk.h>
#include "fsl_common.h"


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Send SELECT_CARD command to set the card to be transfer state or not.
 *
 * @param card Card descriptor.
 * @param isSelected True to set the card into transfer state.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t inline SD_SelectCard(sd_card_t *card, bool isSelected);

/*!
 * @brief Wait write process complete.
 *
 * @param card Card descriptor.
 * @retval kStatus_Timeout Send command timeout.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_WaitWriteComplete(sd_card_t *card);

/*!
 * @brief Send SEND_APPLICATION_COMMAND command.
 *
 * @param card Card descriptor.
 * @param relativeaddress
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_CardNotSupport Card doesn't support.
 * @retval kStatus_Success Operate successfully.
 */
static status_t inline SD_SendApplicationCmd(sd_card_t *card, uint32_t relativeAddress);

/*!
 * @brief Send GO_IDLE command to set the card to be idle state.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t inline SD_GoIdle(sd_card_t *card);

/*!
 * @brief Send STOP_TRANSMISSION command after multiple blocks read/write.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_StopTransmission(sd_card_t *card);

/*!
 * @brief Send SET_BLOCK_SIZE command.
 *
 * @param card Card descriptor.
 * @param blockSize Block size.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t inline SD_SetBlockSize(sd_card_t *card, uint32_t blockSize);

/*!
 * @brief Send GET_RCA command to get card relative address.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SendRca(sd_card_t *card);

/*!
 * @brief Send SWITCH_FUNCTION command to switch the card function group.
 *
 * @param card Card descriptor.
 * @param mode 0 to check function group. 1 to switch function group
 * @param group Function group
 * @param number Function number in the function group.
 * @param status Switch function status.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SwitchFunction(sd_card_t *card, uint32_t mode, uint32_t group, uint32_t number, uint32_t *status);

/*!
 * @brief Decode raw SCR register content in the data blocks.
 *
 * @param card Card descriptor.
 * @param rawScr Raw SCR register content.
 */
static void SD_DecodeScr(sd_card_t *card, uint32_t *rawScr);

/*!
 * @brief Send GET_SCR command.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_SendApplicationCommandFailed Send application command failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_NotSupportYet Not support yet.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SendScr(sd_card_t *card);

/*!
 * @brief Switch the card to be high speed mode.
 *
 * @param card Card descriptor.
 * @param group Group number.
 * @param functio Function number.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_SwitchFailed Switch failed.
 * @retval kStatus_SDMMC_NotSupportYet Not support yet.
 * @retval kStatus_Fail Switch failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SelectFunction(sd_card_t *card, uint32_t group, uint32_t function);

/*!
 * @brief Send SET_DATA_WIDTH command to set SD bus width.
 *
 * @param card Card descriptor.
 * @param width Data bus width.
 * @retval kStatus_SDMMC_SendApplicationCommandFailed Send application command failed.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SetDataBusWidth(sd_card_t *card, sd_data_bus_width_t width);

/*!
 * @brief Decode raw CSD register content in the data blocks.
 *
 * @param card Card descriptor.
 * @param rawCsd Raw CSD register content.
 */
static void SD_DecodeCsd(sd_card_t *card, uint32_t *rawCsd);

/*!
 * @brief Send SEND_CSD command to get CSD register content from Card.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SendCsd(sd_card_t *card);

/*!
 * @brief Decode raw CID register content in the data blocks.
 *
 * @param rawCid raw CID register content.
 * @param card Card descriptor.
 */
static void SD_DecodeCid(sd_card_t *card, uint32_t *rawCid);

/*!
 * @brief Send GET_CID command to get CID from card.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_AllSendCid(sd_card_t *card);

/*!
 * @brief Send SEND_OPERATION_CONDITION command.
 *
 * This function sends host capacity support information and asks the accessed card to send its operating condition
 * register content.
 *
 * @param card Card descriptor.
 * @param argument The argument of the send operation condition ncomamnd.
 * @retval kStatus_SDMMC_SendApplicationCommandFailed Send application command failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Timeout Timeout.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_ApplicationSendOperationCondition(sd_card_t *card, uint32_t argument);

/*!
 * @brief Send GET_INTERFACE_CONDITION command to get card interface condition.
 *
 * This function checks card interface condition, which includes host supply voltage information and asks the card
 * whether card supports the specified host voltage.
 *
 * @param card Card descriptor.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_CardNotSupport Card doesn't support.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_SendInterfaceCondition(sd_card_t *card);

/*!
 * @brief Send switch voltage command
 * switch card voltage to 1.8v
 *
 * @param card Card descriptor.
 */
static status_t SD_SwitchVoltage(sd_card_t *card);

/*!
 * @brief select bus timing
 * select card timing
 * @param card Card descriptor.
 */
static status_t SD_SelectBusTiming(sd_card_t *card);

/*!
 * @brief select card driver strength
 * select card driver strength
 * @param card Card descriptor.
 * @param driverStrength Driver strength
 */
static status_t SD_SetDriverStrength(sd_card_t *card, sd_driver_strength_t driverStrength);

/*!
 * @brief select max current
 * select max operation current
 * @param card Card descriptor.
 * @param maxCurrent Max current
 */
static status_t SD_SetMaxCurrent(sd_card_t *card, sd_max_current_t maxCurrent);

/*!
 * @brief Read data from specific SD card.
 *
 * @param card Card descriptor.
 * @param buffer Buffer to save data blocks read.
 * @param startBlock Card start block number to be read.
 * @param blockSize Block size.
 * @param blockCount Block count.
 * @retval kStatus_SDMMC_CardNotSupport Card doesn't support.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Wait write complete failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_Read(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, uint32_t blockCount);

/*!
 * @brief Write data to specific card
 *
 * @param card Card descriptor.
 * @param buffer Buffer to be sent.
 * @param startBlock Card start block number to be written.
 * @param blockSize Block size.
 * @param blockCount Block count.
 * @retval kStatus_SDMMC_CardNotSupport Card doesn't support.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_Write(
    sd_card_t *card, const uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, uint32_t blockCount);

/*!
 * @brief Erase data for the given block range.
 *
 * @param card Card descriptor.
 * @param startBlock Card start block number to be erased.
 * @param blockCount The block count to be erased.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t SD_Erase(sd_card_t *card, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief card transfer function.
 *
 * @param card Card descriptor.
 * @param content Transfer content.
 * @param retry Retry times
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 * @retval kStatus_SDMMC_TuningFail tuning fail
 */
static status_t SD_Transfer(sd_card_t *card, SDMMCHOST_TRANSFER *content, uint32_t retry);

/*!
 * @brief card execute tuning function.
 *
 * @param card Card descriptor.
 * @retval kStatus_Success Operate successfully.
 * @retval kStatus_SDMMC_TuningFail tuning fail.
 * @retval kStatus_SDMMC_TransferFailed transfer fail
 */
static status_t inline SD_ExecuteTuning(sd_card_t *card);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* g_sdmmc statement */
//extern uint32_t g_sdmmc[SDK_SIZEALIGN(SDMMC_GLOBAL_BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)];
extern uint32_t *g_sdmmc;

/*******************************************************************************
 * Code
 ******************************************************************************/
char __start_sd_global_seg[4096] DMA_INIT_DATA_ALIGN(64);

usdhc_transfer_t *g_sd_content = (usdhc_transfer_t *)(&__start_sd_global_seg[0] + FSL_SDMMC_DEFAULT_BLOCK_SIZE);
usdhc_command_t *g_sd_cmd = (usdhc_command_t *)(&__start_sd_global_seg[0] + 2 * FSL_SDMMC_DEFAULT_BLOCK_SIZE);
usdhc_data_t *g_sd_data = (usdhc_data_t *)(&__start_sd_global_seg[0] + 3 * FSL_SDMMC_DEFAULT_BLOCK_SIZE);

// g_sd must NOT be put in cacheable RAM
sd_card_t g_sd;

static status_t inline SD_SelectCard(sd_card_t *card, bool isSelected)
{
    assert(card);

    return SDMMC_SelectCard(card->host.base, card->host.transfer, card->relativeAddress, isSelected);
}

static status_t inline SD_SendApplicationCmd(sd_card_t *card, uint32_t relativeAddress)
{
    assert(card);

    return SDMMC_SendApplicationCommand(card->host.base, card->host.transfer, relativeAddress);
}

static status_t inline SD_GoIdle(sd_card_t *card)
{
    assert(card);

    return SDMMC_GoIdle(card->host.base, card->host.transfer);
}

static status_t inline SD_SetBlockSize(sd_card_t *card, uint32_t blockSize)
{
    assert(card);

    return SDMMC_SetBlockSize(card->host.base, card->host.transfer, blockSize);
}

static status_t inline SD_ExecuteTuning(sd_card_t *card)
{
    assert(card);

    return SDMMC_ExecuteTuning(card->host.base, card->host.transfer, kSD_SendTuningBlock, 64U);
}

static status_t SD_SwitchVoltage(sd_card_t *card)
{
    assert(card);

    return SDMMC_SwitchVoltage(card->host.base, card->host.transfer);
}

static status_t SD_Transfer(sd_card_t *card, SDMMCHOST_TRANSFER *content, uint32_t retry)
{
    assert(card->host.transfer);
    assert(content);
    status_t error;

    do
    {
        error = card->host.transfer(card->host.base, content);
        if (((error == SDMMCHOST_RETUNING_REQUEST) || (error == SDMMCHOST_TUNING_ERROR) ||
             (content->command->response[0U] & kSDMMC_R1ErrorAllFlag)) &&
            ((card->currentTiming == kSD_TimingSDR104Mode) || (card->currentTiming == kSD_TimingSDR50Mode)))
        {
            /* tuning error need reset tuning circuit */
            if (error == SDMMCHOST_TUNING_ERROR)
            {
                SDMMCHOST_RESET_TUNING(card->host.base, 100U);
            }

            /* execute re-tuning */
            if (SD_ExecuteTuning(card) != kStatus_Success)
            {
                error = kStatus_SDMMC_TuningFail;
            }
            else
            {
                continue;
            }
        }
        else if (error != kStatus_Success)
        {
            error = kStatus_SDMMC_TransferFailed;
        }

        if (retry != 0U)
        {
            retry--;
        }
        else
        {
            break;
        }

    } while ((error != kStatus_Success) && (error != kStatus_SDMMC_TuningFail));

    return error;
}

static status_t SD_WaitWriteComplete(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    g_sd_cmd->index = kSDMMC_SendStatus;
    g_sd_cmd->argument = card->relativeAddress << 16U;
    g_sd_cmd->responseType = kCARD_ResponseTypeR1;
    g_sd_cmd->responseErrorFlags = kSDMMC_R1ErrorAllFlag;

    do
    {
        g_sd_content->command = g_sd_cmd;
        g_sd_content->data = 0U;
        if (kStatus_Success != SD_Transfer(card, g_sd_content, 2U))
        {
            return kStatus_SDMMC_TransferFailed;
        }

        if ((g_sd_cmd->response[0U] & kSDMMC_R1ReadyForDataFlag) &&
            (SDMMC_R1_CURRENT_STATE(g_sd_cmd->response[0U]) != kSDMMC_R1StateProgram))
        {
            break;
        }
    } while (true);

    return kStatus_Success;
}

static status_t SD_StopTransmission(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    g_sd_cmd->index = kSDMMC_StopTransmission;
    g_sd_cmd->argument = 0U;
    g_sd_cmd->type = kCARD_CommandTypeAbort;
    g_sd_cmd->responseType = kCARD_ResponseTypeR1b;
    g_sd_cmd->responseErrorFlags = kSDMMC_R1ErrorAllFlag;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = 0U;
    if (kStatus_Success != SD_Transfer(card, g_sd_content, 1U))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    return kStatus_Success;
}

static status_t SD_SendRca(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    g_sd_cmd->index = kSD_SendRelativeAddress;
    g_sd_cmd->argument = 0U;
    g_sd_cmd->responseType = kCARD_ResponseTypeR6;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    if (kStatus_Success == card->host.transfer(card->host.base, g_sd_content))
    {
        card->relativeAddress = (g_sd_cmd->response[0U] >> 16U);
        return kStatus_Success;
    }

    return kStatus_SDMMC_TransferFailed;
}

static status_t SD_SwitchFunction(sd_card_t *card, uint32_t mode, uint32_t group, uint32_t number, uint32_t *status)
{
    assert(card);
    assert(status);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
	memset((char *)g_sd_data, 0, sizeof(struct _usdhc_data));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};
    //SDMMCHOST_DATA data = {0};

    g_sd_cmd->index = kSD_Switch;
    g_sd_cmd->argument = (mode << 31U | 0x00FFFFFFU);
    g_sd_cmd->argument &= ~((uint32_t)(0xFU) << (group * 4U));
    g_sd_cmd->argument |= (number << (group * 4U));
    g_sd_cmd->responseType = kCARD_ResponseTypeR1;

    g_sd_data->blockSize = 64U;
    g_sd_data->blockCount = 1U;
    g_sd_data->rxData = status;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = g_sd_data;
    if ((kStatus_Success != card->host.transfer(card->host.base, g_sd_content)) ||
        ((g_sd_cmd->response[0U]) & kSDMMC_R1ErrorAllFlag))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    return kStatus_Success;
}

static void SD_DecodeScr(sd_card_t *card, uint32_t *rawScr)
{
    assert(card);
    assert(rawScr);

    sd_scr_t *scr;

    scr = &(card->scr);
    scr->scrStructure = (uint8_t)((rawScr[0U] & 0xF0000000U) >> 28U);
    scr->sdSpecification = (uint8_t)((rawScr[0U] & 0xF000000U) >> 24U);
    if ((uint8_t)((rawScr[0U] & 0x800000U) >> 23U))
    {
        scr->flags |= kSD_ScrDataStatusAfterErase;
    }
    scr->sdSecurity = (uint8_t)((rawScr[0U] & 0x700000U) >> 20U);
    scr->sdBusWidths = (uint8_t)((rawScr[0U] & 0xF0000U) >> 16U);
    if ((uint8_t)((rawScr[0U] & 0x8000U) >> 15U))
    {
        scr->flags |= kSD_ScrSdSpecification3;
    }
    scr->extendedSecurity = (uint8_t)((rawScr[0U] & 0x7800U) >> 10U);
    scr->commandSupport = (uint8_t)(rawScr[0U] & 0x3U);
    scr->reservedForManufacturer = rawScr[1U];
    /* Get specification version. */
    switch (scr->sdSpecification)
    {
        case 0U:
            card->version = kSD_SpecificationVersion1_0;
            break;
        case 1U:
            card->version = kSD_SpecificationVersion1_1;
            break;
        case 2U:
            card->version = kSD_SpecificationVersion2_0;
            if (card->scr.flags & kSD_ScrSdSpecification3)
            {
                card->version = kSD_SpecificationVersion3_0;
            }
            break;
        default:
            break;
    }
    if (card->scr.sdBusWidths & 0x4U)
    {
        card->flags |= kSD_Support4BitWidthFlag;
    }
    /* speed class control cmd */
    if (card->scr.commandSupport & 0x01U)
    {
        card->flags |= kSD_SupportSpeedClassControlCmd;
    }
    /* set block count cmd */
    if (card->scr.commandSupport & 0x02U)
    {
        card->flags |= kSD_SupportSetBlockCountCmd;
    }
}

static status_t SD_SendScr(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
	memset((char *)g_sd_data, 0, sizeof(struct _usdhc_data));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};
    //SDMMCHOST_DATA data = {0};
    uint32_t *rawScr = g_sdmmc;

    /* memset the global buffer */
    memset(g_sdmmc, 0U, FSL_SDMMC_DEFAULT_BLOCK_SIZE);

    if (kStatus_Success != SD_SendApplicationCmd(card, card->relativeAddress))
    {
        return kStatus_SDMMC_SendApplicationCommandFailed;
    }

    g_sd_cmd->index = kSD_ApplicationSendScr;
    g_sd_cmd->responseType = kCARD_ResponseTypeR1;
    g_sd_cmd->argument = 0U;

    g_sd_data->blockSize = 8U;
    g_sd_data->blockCount = 1U;
    g_sd_data->rxData = rawScr;

    g_sd_content->data = g_sd_data;
    g_sd_content->command = g_sd_cmd;
    if ((kStatus_Success != card->host.transfer(card->host.base, g_sd_content)) ||
        ((g_sd_cmd->response[0U]) & kSDMMC_R1ErrorAllFlag))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    /* SCR register data byte sequence from card is big endian(MSB first). */
    switch (card->host.config.endianMode)
    {
        case kSDMMCHOST_EndianModeLittle:
            /* In little endian mode, SD bus byte transferred first is the byte stored in lowest byte position in a
            word which will cause 4 byte's sequence in a word is not consistent with their original sequence from
            card. So the sequence of 4 bytes received in a word should be converted. */
            rawScr[0U] = SWAP_WORD_BYTE_SEQUENCE(rawScr[0U]);
            rawScr[1U] = SWAP_WORD_BYTE_SEQUENCE(rawScr[1U]);
            break;
        case kSDMMCHOST_EndianModeBig:
            break; /* Doesn't need to switch byte sequence when decodes bytes as big endian sequence. */
        case kSDMMCHOST_EndianModeHalfWordBig:
            rawScr[0U] = SWAP_HALF_WROD_BYTE_SEQUENCE(rawScr[0U]);
            rawScr[1U] = SWAP_HALF_WROD_BYTE_SEQUENCE(rawScr[1U]);
            break;
        default:
            return kStatus_SDMMC_NotSupportYet;
    }
    memcpy(card->rawScr, rawScr, sizeof(card->rawScr));
    SD_DecodeScr(card, rawScr);

    return kStatus_Success;
}

static status_t SD_SelectFunction(sd_card_t *card, uint32_t group, uint32_t function)
{
    assert(card);

    uint32_t *functionStatus = g_sdmmc;
    uint16_t functionGroupInfo[6U] = {0};
    uint32_t currentFunctionStatus = 0U;

    /* memset the global buffer */
    memset(g_sdmmc, 0, FSL_SDMMC_DEFAULT_BLOCK_SIZE);

    /* check if card support CMD6 */
    if ((card->version <= kSD_SpecificationVersion1_0) || (!(card->csd.cardCommandClass & kSDMMC_CommandClassSwitch)))
    {
        return kStatus_SDMMC_NotSupportYet;
    }

    /* Check if card support high speed mode. */
    if (kStatus_Success != SD_SwitchFunction(card, kSD_SwitchCheck, group, function, functionStatus))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    /* Switch function status byte sequence from card is big endian(MSB first). */
    switch (card->host.config.endianMode)
    {
        case kSDMMCHOST_EndianModeLittle:
            /* In little endian mode, SD bus byte transferred first is the byte stored in lowest byte position in
            a word which will cause 4 byte's sequence in a word is not consistent with their original sequence from
            card. So the sequence of 4 bytes received in a word should be converted. */
            functionStatus[0U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[0U]);
            functionStatus[1U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[1U]);
            functionStatus[2U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[2U]);
            functionStatus[3U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[3U]);
            functionStatus[4U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[4U]);
            break;
        case kSDMMCHOST_EndianModeBig:
            break; /* Doesn't need to switch byte sequence when decodes bytes as big endian sequence. */
        case kSDMMCHOST_EndianModeHalfWordBig:
            functionStatus[0U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[0U]);
            functionStatus[1U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[1U]);
            functionStatus[2U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[2U]);
            functionStatus[3U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[3U]);
            functionStatus[4U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[4U]);
            break;
        default:
            return kStatus_SDMMC_NotSupportYet;
    }
    /* -functionStatus[0U]---bit511~bit480;
       -functionStatus[1U]---bit479~bit448;
       -functionStatus[2U]---bit447~bit416;
       -functionStatus[3U]---bit415~bit384;
       -functionStatus[4U]---bit383~bit352;
       According to the "switch function status[bits 511~0]" return by switch command in mode "check function":
       -Check if function 1(high speed) in function group 1 is supported by checking if bit 401 is set;
       -check if function 1 is ready and can be switched by checking if bits 379~376 equal value 1;
     */
    functionGroupInfo[5U] = (uint16_t)functionStatus[0U];
    functionGroupInfo[4U] = (uint16_t)(functionStatus[1U] >> 16U);
    functionGroupInfo[3U] = (uint16_t)(functionStatus[1U]);
    functionGroupInfo[2U] = (uint16_t)(functionStatus[2U] >> 16U);
    functionGroupInfo[1U] = (uint16_t)(functionStatus[2U]);
    functionGroupInfo[0U] = (uint16_t)(functionStatus[3U] >> 16U);
    currentFunctionStatus = ((functionStatus[3U] & 0xFFU) << 8U) | (functionStatus[4U] >> 24U);

    /* check if function is support */
    if (((functionGroupInfo[group] & (1 << function)) == 0U) ||
        ((currentFunctionStatus >> (group * 4U)) & 0xFU) != function)
    {
        return kStatus_SDMMC_NotSupportYet;
    }

    /* Switch to high speed mode. */
    if (kStatus_Success != SD_SwitchFunction(card, kSD_SwitchSet, group, function, functionStatus))
    {
        return kStatus_SDMMC_TransferFailed;
    }
    /* Switch function status byte sequence from card is big endian(MSB first). */
    switch (card->host.config.endianMode)
    {
        case kSDMMCHOST_EndianModeLittle:
            /* In little endian mode is little endian, SD bus byte transferred first is the byte stored in lowest byte
            position in a word which will cause 4 byte's sequence in a word is not consistent with their original
            sequence from card. So the sequence of 4 bytes received in a word should be converted. */
            functionStatus[3U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[3U]);
            functionStatus[4U] = SWAP_WORD_BYTE_SEQUENCE(functionStatus[4U]);
            break;
        case kSDMMCHOST_EndianModeBig:
            break; /* Doesn't need to switch byte sequence when decodes bytes as big endian sequence. */
        case kSDMMCHOST_EndianModeHalfWordBig:
            functionStatus[3U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[3U]);
            functionStatus[4U] = SWAP_HALF_WROD_BYTE_SEQUENCE(functionStatus[4U]);
            break;
        default:
            return kStatus_SDMMC_NotSupportYet;
    }
    /* According to the "switch function status[bits 511~0]" return by switch command in mode "set function":
       -check if group 1 is successfully changed to function 1 by checking if bits 379~376 equal value 1;
     */
    currentFunctionStatus = ((functionStatus[3U] & 0xFFU) << 8U) | (functionStatus[4U] >> 24U);

    if (((currentFunctionStatus >> (group * 4U)) & 0xFU) != function)
    {
        return kStatus_SDMMC_SwitchFailed;
    }

    return kStatus_Success;
}

static status_t SD_SetDataBusWidth(sd_card_t *card, sd_data_bus_width_t width)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    if (kStatus_Success != SD_SendApplicationCmd(card, card->relativeAddress))
    {
        return kStatus_SDMMC_SendApplicationCommandFailed;
    }

    g_sd_cmd->index = kSD_ApplicationSetBusWdith;
    g_sd_cmd->responseType = kCARD_ResponseTypeR1;
    switch (width)
    {
        case kSD_DataBusWidth1Bit:
            g_sd_cmd->argument = 0U;
            break;
        case kSD_DataBusWidth4Bit:
            g_sd_cmd->argument = 2U;
            break;
        default:
            return kStatus_InvalidArgument;
    }

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    if ((kStatus_Success != card->host.transfer(card->host.base, g_sd_content)) ||
        ((g_sd_cmd->response[0U]) & kSDMMC_R1ErrorAllFlag))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    return kStatus_Success;
}

static void SD_DecodeCsd(sd_card_t *card, uint32_t *rawCsd)
{
    assert(card);
    assert(rawCsd);

    sd_csd_t *csd;

    csd = &(card->csd);
    csd->csdStructure = (uint8_t)((rawCsd[3U] & 0xC0000000U) >> 30U);
    csd->dataReadAccessTime1 = (uint8_t)((rawCsd[3U] & 0xFF0000U) >> 16U);
    csd->dataReadAccessTime2 = (uint8_t)((rawCsd[3U] & 0xFF00U) >> 8U);
    csd->transferSpeed = (uint8_t)(rawCsd[3U] & 0xFFU);
    csd->cardCommandClass = (uint16_t)((rawCsd[2U] & 0xFFF00000U) >> 20U);
    csd->readBlockLength = (uint8_t)((rawCsd[2U] & 0xF0000U) >> 16U);
    if (rawCsd[2U] & 0x8000U)
    {
        csd->flags |= kSD_CsdReadBlockPartialFlag;
    }
    if (rawCsd[2U] & 0x4000U)
    {
        csd->flags |= kSD_CsdReadBlockPartialFlag;
    }
    if (rawCsd[2U] & 0x2000U)
    {
        csd->flags |= kSD_CsdReadBlockMisalignFlag;
    }
    if (rawCsd[2U] & 0x1000U)
    {
        csd->flags |= kSD_CsdDsrImplementedFlag;
    }
    switch (csd->csdStructure)
    {
        case 0:
            csd->deviceSize = (uint32_t)((rawCsd[2U] & 0x3FFU) << 2U);
            csd->deviceSize |= (uint32_t)((rawCsd[1U] & 0xC0000000U) >> 30U);
            csd->readCurrentVddMin = (uint8_t)((rawCsd[1U] & 0x38000000U) >> 27U);
            csd->readCurrentVddMax = (uint8_t)((rawCsd[1U] & 0x7000000U) >> 24U);
            csd->writeCurrentVddMin = (uint8_t)((rawCsd[1U] & 0xE00000U) >> 20U);
            csd->writeCurrentVddMax = (uint8_t)((rawCsd[1U] & 0x1C0000U) >> 18U);
            csd->deviceSizeMultiplier = (uint8_t)((rawCsd[1U] & 0x38000U) >> 15U);

            /* Get card total block count and block size. */
            card->blockCount = ((csd->deviceSize + 1U) << (csd->deviceSizeMultiplier + 2U));
            card->blockSize = (1U << (csd->readBlockLength));
            if (card->blockSize != FSL_SDMMC_DEFAULT_BLOCK_SIZE)
            {
                card->blockCount = (card->blockCount * card->blockSize);
                card->blockSize = FSL_SDMMC_DEFAULT_BLOCK_SIZE;
                card->blockCount = (card->blockCount / card->blockSize);
            }
            break;
        case 1:
            card->blockSize = FSL_SDMMC_DEFAULT_BLOCK_SIZE;

            csd->deviceSize = (uint32_t)((rawCsd[2U] & 0x3FU) << 16U);
            csd->deviceSize |= (uint32_t)((rawCsd[1U] & 0xFFFF0000U) >> 16U);
            if (csd->deviceSize >= 0xFFFFU)
            {
                card->flags |= kSD_SupportSdxcFlag;
            }

            card->blockCount = ((csd->deviceSize + 1U) * 1024U);
            break;
        default:
            break;
    }
    if ((uint8_t)((rawCsd[1U] & 0x4000U) >> 14U))
    {
        csd->flags |= kSD_CsdEraseBlockEnabledFlag;
    }
    csd->eraseSectorSize = (uint8_t)((rawCsd[1U] & 0x3F80U) >> 7U);
    csd->writeProtectGroupSize = (uint8_t)(rawCsd[1U] & 0x7FU);
    if ((uint8_t)(rawCsd[0U] & 0x80000000U))
    {
        csd->flags |= kSD_CsdWriteProtectGroupEnabledFlag;
    }
    csd->writeSpeedFactor = (uint8_t)((rawCsd[0U] & 0x1C000000U) >> 26U);
    csd->writeBlockLength = (uint8_t)((rawCsd[0U] & 0x3C00000U) >> 22U);
    if ((uint8_t)((rawCsd[0U] & 0x200000U) >> 21U))
    {
        csd->flags |= kSD_CsdWriteBlockPartialFlag;
    }
    if ((uint8_t)((rawCsd[0U] & 0x8000U) >> 15U))
    {
        csd->flags |= kSD_CsdFileFormatGroupFlag;
    }
    if ((uint8_t)((rawCsd[0U] & 0x4000U) >> 14U))
    {
        csd->flags |= kSD_CsdCopyFlag;
    }
    if ((uint8_t)((rawCsd[0U] & 0x2000U) >> 13U))
    {
        csd->flags |= kSD_CsdPermanentWriteProtectFlag;
    }
    if ((uint8_t)((rawCsd[0U] & 0x1000U) >> 12U))
    {
        csd->flags |= kSD_CsdTemporaryWriteProtectFlag;
    }
    csd->fileFormat = (uint8_t)((rawCsd[0U] & 0xC00U) >> 10U);
}

static status_t SD_SendCsd(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    g_sd_cmd->index = kSDMMC_SendCsd;
    g_sd_cmd->argument = (card->relativeAddress << 16U);
    g_sd_cmd->responseType = kCARD_ResponseTypeR2;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    if (kStatus_Success == card->host.transfer(card->host.base, g_sd_content))
    {
        memcpy(card->rawCsd, g_sd_cmd->response, sizeof(card->rawCsd));
        /* The response is from bit 127:8 in R2, corrisponding to g_sd_cmd->response[3U]:command.response[0U][31U:8]. */
        SD_DecodeCsd(card, g_sd_cmd->response);

        return kStatus_Success;
    }

    return kStatus_SDMMC_TransferFailed;
}

static void SD_DecodeCid(sd_card_t *card, uint32_t *rawCid)
{
    assert(card);
    assert(rawCid);

    sd_cid_t *cid;

    cid = &(card->cid);
    cid->manufacturerID = (uint8_t)((rawCid[3U] & 0xFF000000U) >> 24U);
    cid->applicationID = (uint16_t)((rawCid[3U] & 0xFFFF00U) >> 8U);

    cid->productName[0U] = (uint8_t)((rawCid[3U] & 0xFFU));
    cid->productName[1U] = (uint8_t)((rawCid[2U] & 0xFF000000U) >> 24U);
    cid->productName[2U] = (uint8_t)((rawCid[2U] & 0xFF0000U) >> 16U);
    cid->productName[3U] = (uint8_t)((rawCid[2U] & 0xFF00U) >> 8U);
    cid->productName[4U] = (uint8_t)((rawCid[2U] & 0xFFU));

    cid->productVersion = (uint8_t)((rawCid[1U] & 0xFF000000U) >> 24U);

    cid->productSerialNumber = (uint32_t)((rawCid[1U] & 0xFFFFFFU) << 8U);
    cid->productSerialNumber |= (uint32_t)((rawCid[0U] & 0xFF000000U) >> 24U);

    cid->manufacturerData = (uint16_t)((rawCid[0U] & 0xFFF00U) >> 8U);
}

static status_t SD_AllSendCid(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    g_sd_cmd->index = kSDMMC_AllSendCid;
    g_sd_cmd->argument = 0U;
    g_sd_cmd->responseType = kCARD_ResponseTypeR2;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    if (kStatus_Success == card->host.transfer(card->host.base, g_sd_content))
    {
        memcpy(card->rawCid, g_sd_cmd->response, sizeof(card->rawCid));
        SD_DecodeCid(card, g_sd_cmd->response);

        return kStatus_Success;
    }

    return kStatus_SDMMC_TransferFailed;
}

static status_t SD_ApplicationSendOperationCondition(sd_card_t *card, uint32_t argument)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};
    status_t error = kStatus_Fail;
    uint32_t i = FSL_SDMMC_MAX_VOLTAGE_RETRIES;

    g_sd_cmd->index = kSD_ApplicationSendOperationCondition;
    g_sd_cmd->argument = argument;
    g_sd_cmd->responseType = kCARD_ResponseTypeR3;

    while (i--)
    {
        if (kStatus_Success != SD_SendApplicationCmd(card, 0U))
        {
            continue;
        }
		memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
		memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
		g_sd_cmd->index = kSD_ApplicationSendOperationCondition;
		g_sd_cmd->argument = argument;
		g_sd_cmd->responseType = kCARD_ResponseTypeR3;

        g_sd_content->command = g_sd_cmd;
        g_sd_content->data = NULL;
        if (kStatus_Success != card->host.transfer(card->host.base, g_sd_content))
        {
            return kStatus_SDMMC_TransferFailed;
        }

        /* Wait until card exit busy state. */
        if (g_sd_cmd->response[0U] & kSD_OcrPowerUpBusyFlag)
        {
            /* high capacity check */
            if (g_sd_cmd->response[0U] & kSD_OcrCardCapacitySupportFlag)
            {
                card->flags |= kSD_SupportHighCapacityFlag;
            }
            /* 1.8V support */
            if (g_sd_cmd->response[0U] & kSD_OcrSwitch18AcceptFlag)
            {
                card->flags |= kSD_SupportVoltage180v;
            }
            error = kStatus_Success;
            card->ocr = g_sd_cmd->response[0U];
            break;
        }
        error = kStatus_Timeout;
    }

    return error;
}

static status_t SD_SendInterfaceCondition(sd_card_t *card)
{
    assert(card);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};
    uint32_t i = FSL_SDMMC_MAX_CMD_RETRIES;
    status_t error;

    g_sd_cmd->index = kSD_SendInterfaceCondition;
    g_sd_cmd->argument = 0x1AAU;
    g_sd_cmd->responseType = kCARD_ResponseTypeR7;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    do
    {
        if (kStatus_Success != card->host.transfer(card->host.base, g_sd_content))
        {
            error = kStatus_SDMMC_TransferFailed;
        }
        else
        {
            if ((g_sd_cmd->response[0U] & 0xFFU) != 0xAAU)
            {
                error = kStatus_SDMMC_CardNotSupport;
            }
            else
            {
                error = kStatus_Success;
            }
        }
    } while (--i && (error != kStatus_Success));

    return error;
}

static status_t SD_SelectBusTiming(sd_card_t *card)
{
    assert(card);

    status_t error = kStatus_SDMMC_SwitchBusTimingFailed;

    if (card->operationVoltage != kCARD_OperationVoltage180V)
    {
        /* Switch the card to high speed mode */
        if (card->host.capability.flags & kSDMMCHOST_SupportHighSpeed)
        {
            /* group 1, function 1 ->high speed mode*/
            error = SD_SelectFunction(card, kSD_GroupTimingMode, kSD_FunctionSDR25HighSpeed);
            /* If the result isn't "switching to high speed mode(50MHZ) successfully or card doesn't support high speed
             * mode". Return failed status. */
            if (error == kStatus_Success)
            {
                card->currentTiming = kSD_TimingSDR25HighSpeedMode;
                card->busClock_Hz =
                    SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz, SD_CLOCK_50MHZ);
            }
            else if (error == kStatus_SDMMC_NotSupportYet)
            {
                /* if not support high speed, keep the card work at default mode */
                return kStatus_Success;
            }
        }
        else
        {
            /* if not support high speed, keep the card work at default mode */
            return kStatus_Success;
        }
    }
    /* card is in UHS_I mode */
    else if ((kSDMMCHOST_SupportSDR104 != SDMMCHOST_NOT_SUPPORT) ||
             (kSDMMCHOST_SupportSDR50 != SDMMCHOST_NOT_SUPPORT) || (kSDMMCHOST_SupportDDR50 != SDMMCHOST_NOT_SUPPORT))
    {
        switch (card->currentTiming)
        {
            /* if not select timing mode, sdmmc will handle it automatically*/
            case kSD_TimingSDR12DefaultMode:
            case kSD_TimingSDR104Mode:
                error = SD_SelectFunction(card, kSD_GroupTimingMode, kSD_FunctionSDR104);
                if (error == kStatus_Success)
                {
                    card->currentTiming = kSD_TimingSDR104Mode;
                    card->busClock_Hz = SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz,
                                                                 SDMMCHOST_SUPPORT_SDR104_FREQ);
                    break;
                }
            case kSD_TimingDDR50Mode:
                error = SD_SelectFunction(card, kSD_GroupTimingMode, kSD_FunctionDDR50);
                if (error == kStatus_Success)
                {
                    card->currentTiming = kSD_TimingDDR50Mode;
                    card->busClock_Hz =
                        SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz, SD_CLOCK_50MHZ);
                    SDMMCHOST_ENABLE_DDR_MODE(card->host.base, true, 0U);
                }
                break;
            case kSD_TimingSDR50Mode:
                error = SD_SelectFunction(card, kSD_GroupTimingMode, kSD_FunctionSDR50);
                if (error == kStatus_Success)
                {
                    card->currentTiming = kSD_TimingSDR50Mode;
                    card->busClock_Hz =
                        SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz, SD_CLOCK_100MHZ);
                }
                break;
            case kSD_TimingSDR25HighSpeedMode:
                error = SD_SelectFunction(card, kSD_GroupTimingMode, kSD_FunctionSDR25HighSpeed);
                if (error == kStatus_Success)
                {
                    card->currentTiming = kSD_TimingSDR25HighSpeedMode;
                    card->busClock_Hz =
                        SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz, SD_CLOCK_50MHZ);
                }
                break;

            default:
                break;
        }
    }
    else
    {
    }

    /* SDR50 and SDR104 mode need tuning */
    if ((card->currentTiming == kSD_TimingSDR50Mode) || (card->currentTiming == kSD_TimingSDR104Mode))
    {
        /* config IO strength in IOMUX*/
        if (card->currentTiming == kSD_TimingSDR50Mode)
        {
            SDMMCHOST_CONFIG_SD_IO(CARD_BUS_FREQ_100MHZ1, CARD_BUS_STRENGTH_7);
        }
        else
        {
            SDMMCHOST_CONFIG_SD_IO(CARD_BUS_FREQ_200MHZ, CARD_BUS_STRENGTH_7);
        }
        /* execute tuning */
        if (SD_ExecuteTuning(card) != kStatus_Success)
        {
            return kStatus_SDMMC_TuningFail;
        }
    }
    else
    {
        /* set default IO strength to 4 to cover card adapter driver strength difference */
        SDMMCHOST_CONFIG_SD_IO(CARD_BUS_FREQ_100MHZ1, CARD_BUS_STRENGTH_4);
    }

    return error;
}

static status_t SD_SetDriverStrength(sd_card_t *card, sd_driver_strength_t driverStrength)
{
    assert(card);

    status_t error;
    uint32_t strength = driverStrength;

    error = SD_SelectFunction(card, kSD_GroupDriverStrength, strength);

    return error;
}

static status_t SD_SetMaxCurrent(sd_card_t *card, sd_max_current_t maxCurrent)
{
    assert(card);

    status_t error;
    uint32_t current = maxCurrent;

    error = SD_SelectFunction(card, kSD_GroupCurrentLimit, current);

    return error;
}

static status_t SD_Read(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, uint32_t blockCount)
{
	status_t error;

	assert(card);
	assert(buffer);
	assert(blockCount);
	assert(blockSize == FSL_SDMMC_DEFAULT_BLOCK_SIZE);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
	memset((char *)g_sd_data, 0, sizeof(struct _usdhc_data));

	if (((card->flags & kSD_SupportHighCapacityFlag) && (blockSize != 512U)) || (blockSize > card->blockSize) ||
		(blockSize > card->host.capability.maxBlockLength) || (blockSize % 4)) {
		printk("SD_Read err 1111\r\n");
		return kStatus_SDMMC_CardNotSupport;
	}

	/* Wait for the card write process complete because of that card read process and write process use one buffer. */
	if (kStatus_Success != SD_WaitWriteComplete(card)) {
		printk("SD_Read err 2222\r\n");
		return kStatus_SDMMC_WaitWriteCompleteFailed;
	}

	g_sd_data->blockSize = blockSize;
	g_sd_data->blockCount = blockCount;
	g_sd_data->rxData = (uint32_t *)buffer;
	g_sd_data->enableAutoCommand12 = true;


	g_sd_cmd->index = kSDMMC_ReadMultipleBlock;
	if (g_sd_data->blockCount == 1U)
		g_sd_cmd->index = kSDMMC_ReadSingleBlock;

	g_sd_cmd->argument = startBlock;
	if (!(card->flags & kSD_SupportHighCapacityFlag))
		g_sd_cmd->argument *= g_sd_data->blockSize;

	g_sd_cmd->responseType = kCARD_ResponseTypeR1;
	g_sd_cmd->responseErrorFlags = kSDMMC_R1ErrorAllFlag;

	g_sd_content->command = g_sd_cmd;
	g_sd_content->data = g_sd_data;

	error = SD_Transfer(card, g_sd_content, 1U);
	if (kStatus_Success != error)
		return error;

	/* Send STOP_TRANSMISSION command in multiple block transmission and host's AUTO_COMMAND12 isn't enabled. */
	if ((g_sd_data->blockCount > 1U) && (!(g_sd_data->enableAutoCommand12))) {
		if (kStatus_Success != SD_StopTransmission(card)) {
			printk("SD_Read err 3333\r\n");
			return kStatus_SDMMC_StopTransmissionFailed;
		}
	}

	return kStatus_Success;
}

static status_t SD_Write(
    sd_card_t *card, const uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, uint32_t blockCount)
{
    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize == FSL_SDMMC_DEFAULT_BLOCK_SIZE);

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
	memset((char *)g_sd_data, 0, sizeof(struct _usdhc_data));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};
    //SDMMCHOST_DATA data = {0};
    status_t error;

    if (((card->flags & kSD_SupportHighCapacityFlag) && (blockSize != 512U)) || (blockSize > card->blockSize) ||
        (blockSize > card->host.capability.maxBlockLength) || (blockSize % 4U))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    /* Wait for the card's buffer to be not full to write to improve the write performance. */
    while ((GET_SDMMCHOST_STATUS(card->host.base) & CARD_DATA0_STATUS_MASK) != CARD_DATA0_NOT_BUSY)
    {
    }

    /* Wait for the card write process complete because of that card read process and write process use one buffer.*/
    if (kStatus_Success != SD_WaitWriteComplete(card))
    {
        return kStatus_SDMMC_WaitWriteCompleteFailed;
    }

    g_sd_data->blockSize = blockSize;
    g_sd_data->blockCount = blockCount;
    g_sd_data->txData = (const uint32_t *)buffer;
    g_sd_data->enableAutoCommand12 = true;

    g_sd_cmd->index = kSDMMC_WriteMultipleBlock;
    if (g_sd_data->blockCount == 1U)
    {
        g_sd_cmd->index = kSDMMC_WriteSingleBlock;
    }
    g_sd_cmd->argument = startBlock;
    if (!(card->flags & kSD_SupportHighCapacityFlag))
    {
        g_sd_cmd->argument *= g_sd_data->blockSize;
    }
    g_sd_cmd->responseType = kCARD_ResponseTypeR1;
    g_sd_cmd->responseErrorFlags = kSDMMC_R1ErrorAllFlag;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = g_sd_data;

    error = SD_Transfer(card, g_sd_content, 1U);
    if (kStatus_Success != error)
    {
        return error;
    }

    /* Send STOP_TRANSMISSION command in multiple block transmission and host's AUTO_COMMAND12 isn't enabled. */
    if ((g_sd_data->blockCount > 1U) && (!(g_sd_data->enableAutoCommand12)))
    {
        if (kStatus_Success != SD_StopTransmission(card))
        {
            return kStatus_SDMMC_StopTransmissionFailed;
        }
    }

    return kStatus_Success;
}

static status_t SD_Erase(sd_card_t *card, uint32_t startBlock, uint32_t blockCount)
{
    assert(card);
    assert(blockCount);

    uint32_t eraseBlockStart;
    uint32_t eraseBlockEnd;

	memset((char *)g_sd_content, 0, sizeof(struct _usdhc_transfer));
	memset((char *)g_sd_cmd, 0, sizeof(struct _usdhc_command));
    //SDMMCHOST_TRANSFER content = {0};
    //SDMMCHOST_COMMAND command = {0};

    /* Wait for the card's buffer to be not full to write to improve the write performance. */
    while ((GET_SDMMCHOST_STATUS(card->host.base) & CARD_DATA0_STATUS_MASK) != CARD_DATA0_NOT_BUSY)
    {
    }

    /* Wait for the card write process complete because of that card read process and write process use one buffer.*/
    if (kStatus_Success != SD_WaitWriteComplete(card))
    {
        return kStatus_SDMMC_WaitWriteCompleteFailed;
    }

    eraseBlockStart = startBlock;
    eraseBlockEnd = eraseBlockStart + blockCount - 1U;
    if (!(card->flags & kSD_SupportHighCapacityFlag))
    {
        eraseBlockStart = eraseBlockStart * FSL_SDMMC_DEFAULT_BLOCK_SIZE;
        eraseBlockEnd = eraseBlockEnd * FSL_SDMMC_DEFAULT_BLOCK_SIZE;
    }

    /* Send ERASE_WRITE_BLOCK_START command to set the start block number to erase. */
    g_sd_cmd->index = kSD_EraseWriteBlockStart;
    g_sd_cmd->argument = eraseBlockStart;
    g_sd_cmd->responseType = kCARD_ResponseTypeR1;
    g_sd_cmd->responseErrorFlags = kSDMMC_R1ErrorAllFlag;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;

    if (kStatus_Success != SD_Transfer(card, g_sd_content, 1U))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    /* Send ERASE_WRITE_BLOCK_END command to set the end block number to erase. */
    g_sd_cmd->index = kSD_EraseWriteBlockEnd;
    g_sd_cmd->argument = eraseBlockEnd;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    if (kStatus_Success != SD_Transfer(card, g_sd_content, 0U))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    /* Send ERASE command to start erase process. */
    g_sd_cmd->index = kSDMMC_Erase;
    g_sd_cmd->argument = 0U;
    g_sd_cmd->responseType = kCARD_ResponseTypeR1b;
    g_sd_cmd->responseErrorFlags = kSDMMC_R1ErrorAllFlag;

    g_sd_content->command = g_sd_cmd;
    g_sd_content->data = NULL;
    if (kStatus_Success != SD_Transfer(card, g_sd_content, 0U))
    {
        return kStatus_SDMMC_TransferFailed;
    }

    return kStatus_Success;
}

bool SD_CheckReadOnly(sd_card_t *card)
{
    assert(card);

    return ((card->csd.flags & kSD_CsdPermanentWriteProtectFlag) ||
            (card->csd.flags & kSD_CsdTemporaryWriteProtectFlag));
}

status_t ums_sd_read_blocks(void *hcard, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
	uint32_t blockCountOneTime;
	uint32_t blockLeft;
	uint32_t blockDone = 0U;
	uint8_t *nextBuffer = buffer;
	bool dataAddrAlign = true;
	sd_card_t *card = hcard;

	if (!card)
		card = &g_sd;
	assert(card);
	assert(buffer);
	assert(blockCount);
	assert((blockCount + startBlock) <= card->blockCount);

	blockLeft = blockCount;

	while (blockLeft) {
		nextBuffer = (buffer + blockDone * FSL_SDMMC_DEFAULT_BLOCK_SIZE);
		if (!card->noInteralAlign && (!dataAddrAlign || (((uint32_t)nextBuffer) & (sizeof(uint32_t) - 1U)))) {
			blockLeft--;
			blockCountOneTime = 1U;
			memset(g_sdmmc, 0U, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
			dataAddrAlign = false;
		} else {
			if (blockLeft > card->host.capability.maxBlockCount) {
				blockLeft = (blockLeft - card->host.capability.maxBlockCount);
				blockCountOneTime = card->host.capability.maxBlockCount;
			} else {
				blockCountOneTime = blockLeft;
				blockLeft = 0U;
			}
		}

		if (kStatus_Success != SD_Read(card, dataAddrAlign ? nextBuffer : (uint8_t *)g_sdmmc, (startBlock + blockDone),
			FSL_SDMMC_DEFAULT_BLOCK_SIZE, blockCountOneTime)) {
			return kStatus_SDMMC_TransferFailed;
		}

		blockDone += blockCountOneTime;

		if (!card->noInteralAlign && (!dataAddrAlign))
			memcpy(nextBuffer, (uint8_t *)&g_sdmmc, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
	}

	return kStatus_Success;
}

int sd_read_blocks(void *hcard, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
	uint32_t blockCountOneTime;
	uint32_t blockLeft;
	uint32_t blockDone = 0U;
	uint8_t *nextBuffer = buffer;
	bool dataAddrAlign = true;
	sd_card_t *card = hcard;

	assert(card);
	assert(buffer);
	assert(blockCount);
	assert((blockCount + startBlock) <= card->blockCount);

	blockLeft = blockCount;

	while (blockLeft) {
		nextBuffer = (buffer + blockDone * FSL_SDMMC_DEFAULT_BLOCK_SIZE);
		if (!card->noInteralAlign && (!dataAddrAlign || (((uint32_t)nextBuffer) & (sizeof(uint32_t) - 1U)))) {
			blockLeft--;
			blockCountOneTime = 1U;
			memset(g_sdmmc, 0U, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
			dataAddrAlign = false;
		} else {
			if (blockLeft > card->host.capability.maxBlockCount) {
				blockLeft = (blockLeft - card->host.capability.maxBlockCount);
				blockCountOneTime = card->host.capability.maxBlockCount;
			} else {
				blockCountOneTime = blockLeft;
				blockLeft = 0U;
			}
		}
#if 1
		if (blockCountOneTime <= 1 && dataAddrAlign && nextBuffer != (uint8_t *)g_sdmmc) {
			status_t ret;

			memset(g_sdmmc, 0U, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
			ret = SD_Read(card, (uint8_t *)g_sdmmc, (startBlock + blockDone),
				FSL_SDMMC_DEFAULT_BLOCK_SIZE, blockCountOneTime);
			if (ret != kStatus_Success)
				return kStatus_SDMMC_TransferFailed;

			memcpy(nextBuffer, (uint8_t *)g_sdmmc, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
		}
		else if (kStatus_Success != SD_Read(card, dataAddrAlign ? nextBuffer : (uint8_t *)g_sdmmc, (startBlock + blockDone),
			FSL_SDMMC_DEFAULT_BLOCK_SIZE, blockCountOneTime))
			return kStatus_SDMMC_TransferFailed;
#else
		if (kStatus_Success != SD_Read(card, (uint8_t *)g_sdmmc, (startBlock + blockDone),
			FSL_SDMMC_DEFAULT_BLOCK_SIZE, blockCountOneTime)) {
			return kStatus_SDMMC_TransferFailed;
		} else {
			if (dataAddrAlign)
				memcpy(nextBuffer, g_sdmmc, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
		}
#endif
		blockDone += blockCountOneTime;

		if (!card->noInteralAlign && (!dataAddrAlign))
			memcpy(nextBuffer, (uint8_t *)g_sdmmc, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
	}

	return kStatus_Success;
}

status_t sd_write_blocks(void *hcard, const uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
	uint32_t blockCountOneTime; /* The block count can be wrote in one time sending WRITE_BLOCKS command. */
	uint32_t blockLeft; 		/* Left block count to be wrote. */
	uint32_t blockDone = 0U;	/* The block count has been wrote. */
	const uint8_t *nextBuffer;
	bool dataAddrAlign = true;
	sd_card_t *card = hcard;

	assert(card);
	assert(buffer);
	assert(blockCount);
	assert((blockCount + startBlock) <= card->blockCount);

	blockLeft = blockCount;
	while (blockLeft) {
		nextBuffer = (buffer + blockDone * FSL_SDMMC_DEFAULT_BLOCK_SIZE);
		if (!card->noInteralAlign && (!dataAddrAlign || (((uint32_t)nextBuffer) & (sizeof(uint32_t) - 1U)))) {
			blockLeft--;
			blockCountOneTime = 1U;
			memcpy((uint8_t *)g_sdmmc, nextBuffer, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
			dataAddrAlign = false;
		} else {
			if (blockLeft > card->host.capability.maxBlockCount) {
				blockLeft = (blockLeft - card->host.capability.maxBlockCount);
				blockCountOneTime = card->host.capability.maxBlockCount;
			} else {
				blockCountOneTime = blockLeft;
				blockLeft = 0U;
			}
		}

		if (kStatus_Success != SD_Write(card, dataAddrAlign ? nextBuffer : (uint8_t *)g_sdmmc, (startBlock + blockDone),
			FSL_SDMMC_DEFAULT_BLOCK_SIZE, blockCountOneTime)) {
			return kStatus_SDMMC_TransferFailed;
		}

		blockDone += blockCountOneTime;
		if ((!card->noInteralAlign) && !dataAddrAlign)
			memset(g_sdmmc, 0U, FSL_SDMMC_DEFAULT_BLOCK_SIZE);
	}

	return kStatus_Success;
}

status_t SD_EraseBlocks(sd_card_t *card, uint32_t startBlock, uint32_t blockCount)
{
	uint32_t blockCountOneTime; /* The block count can be erased in one time sending ERASE_BLOCKS command. */
	uint32_t blockDone = 0U;	/* The block count has been erased. */
	uint32_t blockLeft; 		/* Left block count to be erase. */
	status_t error;

	assert(card);
	assert(blockCount);
	assert((blockCount + startBlock) <= card->blockCount);

	blockLeft = blockCount;
	while (blockLeft) {
		if (blockLeft > (card->csd.eraseSectorSize + 1U)) {
			blockCountOneTime = card->csd.eraseSectorSize + 1U;
			blockLeft = blockLeft - blockCountOneTime;
		} else {
			blockCountOneTime = blockLeft;
			blockLeft = 0U;
		}

		error = SD_Erase(card, (startBlock + blockDone), blockCountOneTime);
		if (error != kStatus_Success) {
			return error;
		}
		blockDone += blockCountOneTime;
	}

	return kStatus_Success;
}

status_t SD_CardInit(sd_card_t *card)
{
	uint32_t applicationCommand41Argument = 0U;

	assert(card);

	if (!card->isHostReady)
		return kStatus_SDMMC_HostNotReady;
	/* reset variables */
	card->flags = 0U;
	/* set DATA bus width */
	SDMMCHOST_SET_CARD_BUS_WIDTH(card->host.base, kSDMMCHOST_DATABUSWIDTH1BIT);
	/*set card freq to 400KHZ*/
	card->busClock_Hz = SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz, SDMMC_CLOCK_400KHZ);
	/* send card active */
	SDMMCHOST_SEND_CARD_ACTIVE(card->host.base, 100U);
	/* Get host capability. */
	GET_SDMMCHOST_CAPABILITY(card->host.base, &(card->host.capability));

	/* card go idle */
	if (kStatus_Success != SD_GoIdle(card))
		return kStatus_SDMMC_GoIdleFailed;

	if (kSDMMCHOST_SupportV330 != SDMMCHOST_NOT_SUPPORT) {
		applicationCommand41Argument |= (kSD_OcrVdd32_33Flag | kSD_OcrVdd33_34Flag);
		card->operationVoltage = kCARD_OperationVoltage330V;
	} else if (kSDMMCHOST_SupportV300 != SDMMCHOST_NOT_SUPPORT) {
		applicationCommand41Argument |= kSD_OcrVdd29_30Flag;
		card->operationVoltage = kCARD_OperationVoltage330V;
	}

	/* allow user select the work voltage, if not select, sdmmc will handle it automatically */
	if (kSDMMCHOST_SupportV180 != SDMMCHOST_NOT_SUPPORT)
		applicationCommand41Argument |= kSD_OcrSwitch18RequestFlag;

	/* Check card's supported interface condition. */
	if (kStatus_Success == SD_SendInterfaceCondition(card)) {
		/* SDHC or SDXC card */
		applicationCommand41Argument |= kSD_OcrHostCapacitySupportFlag;
		card->flags |= kSD_SupportSdhcFlag;
	} else {
		/* SDSC card */
		if (kStatus_Success != SD_GoIdle(card))
			return kStatus_SDMMC_GoIdleFailed;
	}
	/* Set card interface condition according to SDHC capability and card's supported interface condition. */
	if (kStatus_Success != SD_ApplicationSendOperationCondition(card, applicationCommand41Argument))
		return kStatus_SDMMC_HandShakeOperationConditionFailed;

	/* check if card support 1.8V */
	if ((card->flags & kSD_SupportVoltage180v)) {
		if (kStatus_Success != SD_SwitchVoltage(card))
			return kStatus_SDMMC_InvalidVoltage;
		card->operationVoltage = kCARD_OperationVoltage180V;
	}

	/* Initialize card if the card is SD card. */
	if (kStatus_Success != SD_AllSendCid(card))
		return kStatus_SDMMC_AllSendCidFailed;
	if (kStatus_Success != SD_SendRca(card))
		return kStatus_SDMMC_SendRelativeAddressFailed;
	if (kStatus_Success != SD_SendCsd(card))
		return kStatus_SDMMC_SendCsdFailed;
	if (kStatus_Success != SD_SelectCard(card, true))
		return kStatus_SDMMC_SelectCardFailed;

	if (kStatus_Success != SD_SendScr(card))
		return kStatus_SDMMC_SendScrFailed;

	/* Set to max frequency in non-high speed mode. */
	card->busClock_Hz = SDMMCHOST_SET_CARD_CLOCK(card->host.base, card->host.sourceClock_Hz, SD_CLOCK_25MHZ);

	/* Set to 4-bit data bus mode. */
	if (((card->host.capability.flags) & kSDMMCHOST_Support4BitBusWidth) &&
		(card->flags & kSD_Support4BitWidthFlag)) {
		if (kStatus_Success != SD_SetDataBusWidth(card, kSD_DataBusWidth4Bit))
			return kStatus_SDMMC_SetDataBusWidthFailed;
		SDMMCHOST_SET_CARD_BUS_WIDTH(card->host.base, kSDMMCHOST_DATABUSWIDTH4BIT);
	}

	/* set sd card driver strength */
	SD_SetDriverStrength(card, card->driverStrength);
	/* set sd card current limit */
	SD_SetMaxCurrent(card, card->maxCurrent);

	/* set block size */
	if (SD_SetBlockSize(card, FSL_SDMMC_DEFAULT_BLOCK_SIZE))
		return kStatus_SDMMC_SetCardBlockSizeFailed;

	/* select bus timing */
	if (kStatus_Success != SD_SelectBusTiming(card))
		return kStatus_SDMMC_SwitchBusTimingFailed;

	return kStatus_Success;
}

void SD_CardDeinit(sd_card_t *card)
{
	assert(card);

	SD_SelectCard(card, false);
}

extern void printk(const char *fmt, ...);
status_t SD_HostInit(sd_card_t *card)
{
	assert(card);

	if ((!card->isHostReady) &&
		SDMMCHOST_Init(&(card->host), (void *)(card->usrParam.cd)) != kStatus_Success)
		return kStatus_Fail;

	/* set the host status flag, after the card re-plug in, don't need init host again */
	card->isHostReady = true;
	return kStatus_Success;
}

void SD_HostDeinit(sd_card_t *card)
{
	assert(card);

	SDMMCHOST_Deinit(&(card->host));
	/* should re-init host */
	card->isHostReady = false;
}

void SD_HostReset(SDMMCHOST_CONFIG *host)
{
	SDMMCHOST_Reset(host->base);
}

void SD_PowerOnCard(SDMMCHOST_TYPE *base, const sdmmchost_pwr_card_t *pwr)
{
	SDMMCHOST_PowerOnCard(base, pwr);
}

void SD_PowerOffCard(SDMMCHOST_TYPE *base, const sdmmchost_pwr_card_t *pwr)
{
	SDMMCHOST_PowerOffCard(base, pwr);
}

status_t SD_WaitCardDetectStatus(SDMMCHOST_TYPE *hostBase, const sdmmchost_detect_card_t *cd, bool waitCardStatus)
{
	return SDMMCHOST_WaitCardDetectStatus(hostBase, cd, waitCardStatus);
}

bool SD_IsCardPresent(sd_card_t *card)
{
	return SDMMCHOST_IsCardPresent();
}

status_t SD_Init(sd_card_t *card)
{
	assert(card);

	if (!card->isHostReady)	{
		if (SD_HostInit(card) != kStatus_Success)
		return kStatus_SDMMC_HostNotReady;
	} else {
		SD_HostReset(&(card->host));
	}
	SD_PowerOffCard(card->host.base, card->usrParam.pwr);

	if (SD_WaitCardDetectStatus(card->host.base, card->usrParam.cd, true) != kStatus_Success)
		return kStatus_SDMMC_CardDetectFailed;
	SD_PowerOnCard(card->host.base, card->usrParam.pwr);

	return SD_CardInit(card);
}

void SD_Deinit(sd_card_t *card)
{
	/* card deinitialize */
	SD_CardDeinit(card);
	/* host deinitialize */
	SD_HostDeinit(card);
}

uint32_t sd_get_blk_count(void *card_info)
{
	sd_card_t *card = card_info;

	return card->blockCount;
}

#include <device.h>

extern int soc_sdhc_init(struct device *dev);

DEVICE_AND_API_INIT(sdhc_soc, "sdhc_soc", soc_sdhc_init, &g_sd, NULL,
		    APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

struct device *soc_sdhc_get_device(void)
{
	return DEVICE_GET(sdhc_soc);
}

/* State in Card driver. */

/*! @brief SDMMC host detect card configuration */

bool s_cardInserted = false;
static void sd_detect_cb(bool isInserted, void *userData)
{
	s_cardInserted = isInserted;
}

static const sdmmchost_detect_card_t s_sdCardDetect = {
	.cdType = kSDMMCHOST_DetectCardByGpioCD,
	.cdTimeOut_ms = (~0U),
	.cardInserted = sd_detect_cb,
	.cardRemoved = sd_detect_cb,
};

void _sdcard_pin_init(void)
{
	uint32_t padDrv = 0x89;//1<<0 | 3<<6 | 5<<3;

	CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

	IOMUXC_SetPinMux(
		IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,        /* GPIO_AD_B0_05 is configured as GPIO1_IO05 */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */                              /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(  // SD_CD
		IOMUXC_GPIO_B1_12_GPIO2_IO28,           /* GPIO_B1_12 is configured as GPIO2_IO28 */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_B1_14_USDHC1_VSELECT,       /* GPIO_B1_14 is configured as USDHC1_VSELECT */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 is configured as USDHC1_CMD */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 is configured as USDHC1_CLK */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 is configured as USDHC1_DATA0 */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 is configured as USDHC1_DATA1 */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 is configured as USDHC1_DATA2 */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 is configured as USDHC1_DATA3 */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,        /* GPIO_AD_B0_05 PAD functional properties : */
		0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */

	IOMUXC_SetPinConfig(	// SD0_CD_SW
		IOMUXC_GPIO_B1_12_GPIO2_IO28,           /* GPIO_B1_12 PAD functional properties : */
		0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
	IOMUXC_SetPinConfig( // SD0_VSELECT
		IOMUXC_GPIO_B1_14_USDHC1_VSELECT,       /* GPIO_B1_14 PAD functional properties : */
		0x0170A1u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/4
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 PAD functional properties : */
		0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */

	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 PAD functional properties : */
		0x014000u | padDrv);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Disabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 PAD functional properties : */
		0x017000u | padDrv);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 PAD functional properties : */
		0x017000u | padDrv);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 PAD functional properties : */
		0x017000u | padDrv);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 PAD functional properties : */
		0x017000u | padDrv);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
}

static void USDHCClockConfiguration(void)
{
	/*configure system pll PFD2 fractional divider to 18*/
	CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
	/* Configure USDHC clock source and divider */
	CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
	CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
}

#define IRQ_PRI_SDIO            4

status_t sdcard_init(void)
{
	status_t error = kStatus_Success;

	g_sd.isHostReady = 0;

	_sdcard_pin_init();
	USDHCClockConfiguration();
	NVIC_SetPriority(SD_HOST_IRQ, IRQ_PRI_SDIO);
	g_sd.host.base = SD_HOST_BASEADDR;
	g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
	g_sd.usrParam.cd = &s_sdCardDetect;

	/* Init card. */
	if (SD_HostInit(&g_sd)) {
		error = kStatus_Fail;
		printk("\n SD card init failed \n");
	}

	SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);

	/* wait card insert */
	while (!s_cardInserted)
		;

	printk("\r\nCard inserted.\r\n");
	/* reset host once card re-plug in */
	SD_HostReset(&(g_sd.host));
	/* Init card. */
	if (SD_CardInit(&g_sd)) {
		printk("\r\nSD card init failed!!!!!!!!!\r\n");
		return -1;
	}
	printk("SD_CardInit success error:%d\r\n", (int)error);
	return error;
}

status_t sdcard_deinit(void)
{
	SD_Deinit(&g_sd);
	return kStatus_Success;
}

bool sdcard_is_present(void)
{
	if (SD_IsCardPresent(&g_sd) && g_sd.isHostReady)
		return 1;
	return 0;
}

bool sdcard_power_on(void)
{
	if (!sdcard_is_present()) {
		return false;
	}
	if (g_sd.isHostReady) {
		return true;
	}
	if (sdcard_init() == kStatus_Success)
		return 1;
	sdcard_deinit();
	return 0;
}

void sdcard_power_off(void)
{
	if (!g_sd.isHostReady) {
		return;
	}
	SD_Deinit(&g_sd);
}

uint32_t sdcard_get_lba_count(void)
{
	return g_sd.blockCount;
}

uint64_t sdcard_get_capacity_in_bytes(void)
{
	if (g_sd.isHostReady == 0)
		return 0;

	return (uint64_t)g_sd.blockCount * g_sd.blockSize;
}

status_t sdcard_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks)
{
	// check that SD card is initialised
	status_t ret;
	uint32_t usbIrqEn;

	if (g_sd.isHostReady == 0)
		return kStatus_Fail;

	usbIrqEn = __NVIC_GetEnableIRQ(USB_OTG1_IRQn);
	if (usbIrqEn)
		NVIC_DisableIRQ(USB_OTG1_IRQn);

	ret = sd_read_blocks(&g_sd, dest, block_num, num_blocks);
	if (usbIrqEn)
		NVIC_EnableIRQ(USB_OTG1_IRQn);
	return ret;
}

status_t sdcard_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks)
{
	status_t ret;
	uint32_t usbIrqEn;

	if (g_sd.isHostReady == 0)
		return kStatus_Fail;
	usbIrqEn = __NVIC_GetEnableIRQ(USB_OTG1_IRQn);
	if (usbIrqEn)
		NVIC_DisableIRQ(USB_OTG1_IRQn);

	ret = sd_write_blocks(&g_sd, src, block_num, num_blocks);
	if (usbIrqEn)
		NVIC_EnableIRQ(USB_OTG1_IRQn);
	return ret;
}

int sdcard_info(uint32_t *total_size, uint32_t *block_size, uint32_t *busclk_hz)
{
	if (g_sd.isHostReady == 0 || !SD_IsCardPresent(&g_sd))
			return -1;
	*total_size = g_sd.blockCount * g_sd.blockSize;
	*block_size = g_sd.blockSize;
	*busclk_hz = g_sd.blockSize;
	return 0;
}
