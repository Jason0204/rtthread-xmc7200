/*******************************************************************************
 * File Name: cycfg_peripherals.h
 *
 * Description:
 * Peripheral Hardware Block configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.20.0
 * device-db 4.15.0.5746
 * mtb-pdl-cat1 3.10.0.32115
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

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "cy_canfd.h"
#include "cy_sysclk.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#endif /* defined (CY_USING_HAL) */

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define CANFD_ENABLED 1U
#define CANFD_HW CANFD0
#define CANFD_CHANNEL CANFD0_CH1
#define CANFD_STD_ID_FILTER_ID_0 0
#define CANFD_EXT_ID_FILTER_ID_0 0
#define CANFD_DATA_0 0
#define CANFD_DATA_1 1
#define CANFD_DATA_2 2
#define CANFD_DATA_3 3
#define CANFD_DATA_4 4
#define CANFD_DATA_5 5
#define CANFD_DATA_6 6
#define CANFD_DATA_7 7
#define CANFD_DATA_8 8
#define CANFD_DATA_9 9
#define CANFD_DATA_10 10
#define CANFD_DATA_11 11
#define CANFD_DATA_12 12
#define CANFD_DATA_13 13
#define CANFD_DATA_14 14
#define CANFD_DATA_15 15
#define CANFD_IRQ_0 canfd_0_interrupts0_1_IRQn
#define CANFD_IRQ_1 canfd_0_interrupts1_1_IRQn

extern void canfd_rx_callback(bool rxFIFOMsg, uint8_t msgBufOrRxFIFONum, cy_stc_canfd_rx_buffer_t* basemsg);
extern cy_stc_canfd_bitrate_t CANFD_nominalBitrateConfig;
extern cy_stc_canfd_bitrate_t CANFD_dataBitrateConfig;
extern cy_stc_canfd_transceiver_delay_compensation_t CANFD_tdcConfig;
extern cy_stc_id_filter_t CANFD_stdIdFilter_0;
extern cy_stc_id_filter_t CANFD_stdIdFilters[];
extern cy_stc_canfd_sid_filter_config_t CANFD_sidFiltersConfig;
extern cy_stc_canfd_f0_t CANFD_extIdFilterF0Config_0;
extern cy_stc_canfd_f1_t CANFD_extIdFilterF1Config_0;
extern cy_stc_extid_filter_t CANFD_extIdFilter_0;
extern cy_stc_extid_filter_t CANFD_extIdFilters[];
extern cy_stc_canfd_extid_filter_config_t CANFD_extIdFiltersConfig;
extern cy_stc_canfd_global_filter_config_t CANFD_globalFilterConfig;
extern cy_en_canfd_fifo_config_t CANFD_rxFifo0Config;
extern cy_en_canfd_fifo_config_t CANFD_rxFifo1Config;
extern cy_stc_canfd_config_t CANFD_config;
extern cy_stc_canfd_t0_t CANFD_T0RegisterBuffer_0;
extern cy_stc_canfd_t1_t CANFD_T1RegisterBuffer_0;
extern uint32_t CANFD_dataBuffer_0[];
extern cy_stc_canfd_tx_buffer_t CANFD_txBuffer_0;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CANFD_obj;
#endif /* defined (CY_USING_HAL) */

void init_cycfg_peripherals(void);
void reserve_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PERIPHERALS_H */
