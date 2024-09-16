/*******************************************************************************
 * File Name: cycfg_peripherals.c
 *
 * Description:
 * Peripheral Hardware Block configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.20.0
 * device-db 4.17.0.6514
 * mtb-pdl-cat1 3.11.1.35176
 *
 *******************************************************************************
 * Copyright 2024 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#include "cycfg_peripherals.h"

#define CANFD_STD_ID_FILTER_0 \
{\
    .sfid2 = 0x20U, \
    .sfid1 = 0x10U, \
    .sfec = CY_CANFD_SFEC_DISABLE, \
    .sft = CY_CANFD_SFT_RANGE_SFID1_SFID2, \
 }
#define CANFD_EXT_ID_FILTER_0 \
{\
    .f0_f = &CANFD_extIdFilterF0Config_0, \
    .f1_f = &CANFD_extIdFilterF1Config_0, \
 }

void canfd_tx_callback(void);
void canfd_rx_callback(bool rxFIFOMsg, uint8_t msgBufOrRxFIFONum, cy_stc_canfd_rx_buffer_t* basemsg);
cy_stc_canfd_bitrate_t CANFD_nominalBitrateConfig =
{
    .prescaler = 1U - 1U,
    .timeSegment1 = 27U - 1U,
    .timeSegment2 = 4U - 1U,
    .syncJumpWidth = 2U - 1U,
};
cy_stc_canfd_bitrate_t CANFD_dataBitrateConfig =
{
    .prescaler = 2U - 1U,
    .timeSegment1 = 5U - 1U,
    .timeSegment2 = 2U - 1U,
    .syncJumpWidth = 2U - 1U,
};
cy_stc_canfd_transceiver_delay_compensation_t CANFD_tdcConfig =
{
    .tdcEnabled = false,
    .tdcOffset = 0U,
    .tdcFilterWindow = 0U,
};
cy_stc_id_filter_t CANFD_stdIdFilter_0 =
{
    .sfid2 = 0x20U,
    .sfid1 = 0x10U,
    .sfec = CY_CANFD_SFEC_DISABLE,
    .sft = CY_CANFD_SFT_RANGE_SFID1_SFID2,
};
cy_stc_id_filter_t CANFD_stdIdFilters[] =
{
    [0] = CANFD_STD_ID_FILTER_0,
};
cy_stc_canfd_sid_filter_config_t CANFD_sidFiltersConfig =
{
    .numberOfSIDFilters = 1U,
    .sidFilter = CANFD_stdIdFilters,
};
cy_stc_canfd_f0_t CANFD_extIdFilterF0Config_0 =
{
    .efid1 = 0U,
    .efec = CY_CANFD_EFEC_DISABLE,
};
cy_stc_canfd_f1_t CANFD_extIdFilterF1Config_0 =
{
    .efid2 = 0U,
    .eft = CY_CANFD_EFT_RANGE_EFID1_EFID2,
};
cy_stc_extid_filter_t CANFD_extIdFilter_0 =
{
    .f0_f = &CANFD_extIdFilterF0Config_0,
    .f1_f = &CANFD_extIdFilterF1Config_0,
};
cy_stc_extid_filter_t CANFD_extIdFilters[] =
{
    [0] = CANFD_EXT_ID_FILTER_0,
};
cy_stc_canfd_extid_filter_config_t CANFD_extIdFiltersConfig =
{
    .numberOfEXTIDFilters = 1U,
    .extidFilter = (cy_stc_extid_filter_t*)&CANFD_extIdFilters,
    .extIDANDMask = 536870911UL,
};
cy_stc_canfd_global_filter_config_t CANFD_globalFilterConfig =
{
    .nonMatchingFramesStandard = CY_CANFD_ACCEPT_IN_RXFIFO_0,
    .nonMatchingFramesExtended = CY_CANFD_ACCEPT_IN_RXFIFO_0,
    .rejectRemoteFramesStandard = false,
    .rejectRemoteFramesExtended = false,
};
cy_en_canfd_fifo_config_t CANFD_rxFifo0Config =
{
    .mode = CY_CANFD_FIFO_MODE_BLOCKING,
    .watermark = 0U,
    .numberOfFIFOElements = 8U,
    .topPointerLogicEnabled = false,
};
cy_en_canfd_fifo_config_t CANFD_rxFifo1Config =
{
    .mode = CY_CANFD_FIFO_MODE_BLOCKING,
    .watermark = 0U,
    .numberOfFIFOElements = 8U,
    .topPointerLogicEnabled = false,
};
cy_stc_canfd_config_t CANFD_config =
{
    .txCallback = canfd_tx_callback,
    .rxCallback = canfd_rx_callback,
    .errorCallback = NULL,
    .canFDMode = true,
    .bitrate = &CANFD_nominalBitrateConfig,
    .fastBitrate = &CANFD_dataBitrateConfig,
    .tdcConfig = &CANFD_tdcConfig,
    .sidFilterConfig = &CANFD_sidFiltersConfig,
    .extidFilterConfig = &CANFD_extIdFiltersConfig,
    .globalFilterConfig = &CANFD_globalFilterConfig,
    .rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .rxFIFO1DataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .rxFIFO0DataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .rxFIFO0Config = &CANFD_rxFifo0Config,
    .rxFIFO1Config = &CANFD_rxFifo1Config,
    .noOfRxBuffers = 1U,
    .noOfTxBuffers = 1U,
    .messageRAMaddress = CY_CAN0MRAM_BASE + 0U,
    .messageRAMsize = 8192U,
};
cy_stc_canfd_t0_t CANFD_T0RegisterBuffer_0 =
{
    .id = 0x22U,
    .rtr = CY_CANFD_RTR_DATA_FRAME,
    .xtd = CY_CANFD_XTD_STANDARD_ID,
    .esi = CY_CANFD_ESI_ERROR_ACTIVE,
};
cy_stc_canfd_t1_t CANFD_T1RegisterBuffer_0 =
{
    .dlc = 8U,
    .brs = true,
    .fdf = CY_CANFD_FDF_CAN_FD_FRAME,
    .efc = false,
    .mm = 0U,
};
uint32_t CANFD_dataBuffer_0[] =
{
    [CANFD_DATA_0] = 0x04030201U,
    [CANFD_DATA_1] = 0x08070605U,
};
cy_stc_canfd_tx_buffer_t CANFD_txBuffer_0 =
{
    .t0_f = &CANFD_T0RegisterBuffer_0,
    .t1_f = &CANFD_T1RegisterBuffer_0,
    .data_area_f = CANFD_dataBuffer_0,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CANFD_obj =
{
    .type = CYHAL_RSC_CAN,
    .block_num = 0U,
    .channel_num = 1U,
};
#endif /* defined (CY_USING_HAL) */

__WEAK void canfd_tx_callback(void)
{
    
}
__WEAK void canfd_rx_callback(bool rxFIFOMsg, uint8_t msgBufOrRxFIFONum, cy_stc_canfd_rx_buffer_t* basemsg)
{
    (void)rxFIFOMsg;
    (void)msgBufOrRxFIFONum;
    (void)basemsg;
}
void init_cycfg_peripherals(void)
{
#if defined (CY_DEVICE_CONFIGURATOR_IP_ENABLE_FEATURE)
    Cy_SysClk_PeriGroupSlaveInit(CY_MMIO_CANFD0_PERI_NR , CY_MMIO_CANFD0_GROUP_NR, CY_MMIO_CANFD0_SLAVE_NR, CY_MMIO_CANFD0_CLK_HF_NR);
#endif /* defined (CY_DEVICE_CONFIGURATOR_IP_ENABLE_FEATURE) */
    Cy_SysClk_PeriPclkAssignDivider(PCLK_CANFD0_CLOCK_CAN1, CY_SYSCLK_DIV_24_5_BIT, 0U);
}
void reserve_cycfg_peripherals(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&CANFD_obj);
#endif /* defined (CY_USING_HAL) */
}
