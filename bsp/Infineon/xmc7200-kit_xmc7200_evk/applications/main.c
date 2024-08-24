/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-22     LZero        first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"

#define LED1_PIN     GET_PIN(16, 1)
#define LED2_PIN     GET_PIN(16, 2)
#define LED3_PIN     GET_PIN(16, 3)


int main(void)
{
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
    for (;;)
    {
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(LED3_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(200);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(200);
        rt_pin_write(LED3_PIN, PIN_LOW);
        rt_thread_mdelay(200);
    }
}
