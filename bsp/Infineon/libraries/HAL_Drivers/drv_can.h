/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-13     Administrator       the first version
 */
#ifndef __DRV_CAN_H_
#define __DRV_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "cy_canfd.h"



/* xmc7200 can device */
struct xmc_canfd
{
    char *name;
    cy_stc_canfd_config_t canfd_conf;
    struct rt_can_device device;    /* inherit from can device */
};

int rt_hw_can_init(void);


#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_INCLUDE_DRV_CAN_H_ */
