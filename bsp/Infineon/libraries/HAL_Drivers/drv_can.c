/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-13     Administrator       the first version
 */
#include "drv_common.h"
#include "drv_can.h"
#include "cycfg_peripherals.h"
#include "cy_sysint.h"
#include "cy_canfd.h"

#ifdef RT_USING_CAN
#include <rtdevice.h>

//#define DRV_DEBUG
#define DBG_TAG              "drv.can_fd"

#ifdef DRV_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif /* DRV_DEBUG */

#include <rtdbg.h>

/* CAN node definition */
#define CAN_NODE_1              1
#define CAN_NODE_2              2
/* Set the example CAN node */
#define USE_CAN_NODE            CAN_NODE_1
#define CAN_HW_CHANNEL          1


/* canfd interrupt handler */
void isr_canfd (void);

/* This structure initializes the CANFD interrupt for the NVIC */
cy_stc_sysint_t canfd_irq_cfg =
{
    .intrSrc  = (NvicMux2_IRQn << 16) | CANFD_IRQ_0, /* Source of interrupt signal */
    .intrPriority = 1U, /* Interrupt priority */
};
/* This is a shared context structure, unique for each canfd channel */
cy_stc_canfd_context_t canfd_context; 


static struct xmc_canfd drv_can0 =
{
    .name = "xmc_can0",
};

static rt_err_t xmc_canfd_config(struct rt_can_device *can, struct can_configure *cfg)
{
    cy_en_canfd_status_t status;
    
    /* Hook the interrupt service routine and enable the interrupt */
    (void) Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);
    NVIC_EnableIRQ(NvicMux2_IRQn);
    
    status = Cy_CANFD_Init(CANFD_HW, CAN_HW_CHANNEL, &CANFD_config,
                           &canfd_context);
    if(status != CY_CANFD_SUCCESS) return -RT_ERROR;
    
    return RT_EOK;

}

static rt_err_t xmc_canfd_ctrl(struct rt_can_device *can, int cmd, void *arg)
{
    rt_uint32_t argval;
    struct rt_can_filter_config *filter_cfg;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_CLR_INT:

        break;

        case RT_DEVICE_CTRL_SET_INT:

        break;

        case RT_CAN_CMD_SET_FILTER:

        break;

        case RT_CAN_CMD_SET_MODE:

        break;

        case RT_CAN_CMD_SET_BAUD:

        break;

        case RT_CAN_CMD_SET_PRIV:

        break;

        case RT_CAN_CMD_GET_STATUS:

        break;
    }

    return RT_EOK;
}

static rt_ssize_t xmc_canfd_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{

    return RT_EOK;
}

static rt_ssize_t xmc_canfd_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    cy_en_canfd_status_t status;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    /* Array to store the data bytes of the CANFD frame */
    uint32_t canfd_dataBuffer[] =
    {
        [CANFD_DATA_0] = 0x04030201U,
        [CANFD_DATA_1] = 0x08070605U,
    };

    /* Setting CAN node identifier */
    CANFD_T0RegisterBuffer_0.id = USE_CAN_NODE;

    /* Assign the user defined data buffer to CANFD data area */
    CANFD_txBuffer_0.data_area_f = canfd_dataBuffer;
    status = Cy_CANFD_UpdateAndTransmitMsgBuffer( CANFD_HW,
                                                  CAN_HW_CHANNEL,
                                                  &CANFD_txBuffer_0,
                                                  0,
                                                  &canfd_context );
    if(status != CY_CANFD_SUCCESS) return -RT_ERROR;
    
    return RT_EOK;
}


static const struct rt_can_ops xmc_canfd_ops =
{
    .configure = xmc_canfd_config,
    .control   = xmc_canfd_ctrl,
    .recvmsg   = xmc_canfd_recvmsg,
    .sendmsg   = xmc_canfd_sendmsg,
};


static void _can_rx_isr(struct rt_can_device *can, rt_uint32_t fifo)
{

}
/*******************************************************************************
* Function Name: canfd_rx_callback
********************************************************************************
* Summary:
* This is the callback function for canfd reception
*
* Parameters:
*    rxFIFOMsg                      Message was received in Rx FIFO (0/1)
*    msgBufOrRxFIFONum              RxFIFO number of the received message
*    basemsg                        Message buffer
*
*******************************************************************************/
void canfd_rx_callback (bool                        rxFIFOMsg, 
                        uint8_t                     msgBufOrRxFIFONum, 
                        cy_stc_canfd_rx_buffer_t*   basemsg)
{
    /* Array to hold the data bytes of the CANFD frame */
    uint8_t canfd_data_buffer[8];
    /* Variable to hold the data length code of the CANFD frame */
    int canfd_dlc;
    /* Variable to hold the Identifier of the CANFD frame */
    int canfd_id;

    /* Message was received in Rx FIFO */
    if (rxFIFOMsg == true)
    {
        /* Checking whether the frame received is a data frame */
        if(CY_CANFD_RTR_DATA_FRAME == basemsg->r0_f->rtr) 
        {
            /* Toggle the user LED */
//            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
            /* Get the CAN DLC and ID from received message */
            canfd_dlc = basemsg->r1_f->dlc;
            canfd_id  = basemsg->r0_f->id;
            /* Print the received message by UART */
            rt_kprintf("%d bytes received from Node-%d with identifier %d\r\n\r\n",
                                                        canfd_dlc,
                                                        canfd_id,
                                                        canfd_id);
            memcpy(canfd_data_buffer, basemsg->data_area_f, canfd_dlc);
            rt_kprintf("Rx Data : ");
            for (uint8_t msg_idx = 0; msg_idx < canfd_dlc ; msg_idx++)
            {
                rt_kprintf(" %d ", canfd_data_buffer[msg_idx]);
            }
            rt_kprintf("\r\n\r\n");
        }
    }
    /* These parameters are not used in this snippet */
    (void)msgBufOrRxFIFONum;
}
/*******************************************************************************
* Function Name: isr_canfd
********************************************************************************
* Summary:
* This is the interrupt handler function for the canfd interrupt.
*
* Parameters:
*  none
*    
*
*******************************************************************************/
void isr_canfd(void)
{
    rt_interrupt_enter();
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD_HW, CAN_HW_CHANNEL, &canfd_context);
    rt_interrupt_leave();
}

int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks   = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 14;
#endif    


    /* register CAN_FD device */
    rt_hw_can_register(&drv_can0.device,
                       drv_can0.name,
                       &xmc_canfd_ops,
                       &drv_can0);

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_can_init);


#endif