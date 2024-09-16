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
#define DBG_TAG              "drv.canfd"

#ifdef DRV_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif /* DRV_DEBUG */

#include <rtdbg.h>

#ifdef BSP_USING_CANFD0
static struct xmc_canfd xmcDrvCAN0 = 
{
    .name = "xmcCAN0",
    .conf = &CANFD_config,
};

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

static const cy_stc_canfd_bitrate_t xmc_fdcan_bitrate[] = 
/* Baud rate prescaler ; Time segment before sample point; Time segment after sample point; Synchronization jump width */
{
    { 1U - 1U, 27U - 1U, 4U - 1U, 2U - 1U },    // 1000kbps
    { 1U - 1U, 34U - 1U, 5U - 1U, 2U - 1U },    // 800kbps
    { 2U - 1U, 27U - 1U, 4U - 1U, 2U - 1U },    // 500kbps
    { 4U - 1U, 27U - 1U, 4U - 1U, 2U - 1U },    // 250kbps
    { 8U - 1U, 24U - 1U, 4U - 1U, 2U - 1U },    // 125kbps
    { 4U - 1U, 69U - 1U, 10U - 1U, 2U - 1U },    // 100kbps
    { 8U - 1U, 69U - 1U, 10U - 1U, 2U - 1U },    // 50kbps
    { 8U - 1U, 174U - 1U, 25U - 1U, 2U - 1U },    // 20kbps
    { 16U - 1U, 174U - 1U, 25U - 1U, 2U - 1U },    // 10kbps
};



static rt_err_t xmc_canfd_config(struct rt_can_device *can, struct can_configure *cfg)
{
    cy_en_canfd_status_t status;
    struct xmc_canfd *pdrv_can;
    
    RT_ASSERT(can);
	RT_ASSERT(cfg);
    
//    rt_kprintf("xmc_canfd_config is called\r\n");
    pdrv_can = (struct xmc_canfd *)can->parent.user_data;
    RT_ASSERT(pdrv_can);
    
//    status = Cy_CANFD_Disable(CANFD_HW, 1);
//    if(status != CY_CANFD_SUCCESS) return -RT_ERROR;
    
    pdrv_can->conf->canFDMode = false;  // CAN_CLASSIC_MODE

    switch(cfg->mode)
    {
        case RT_CAN_MODE_NORMAL:
            Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_DISABLE);
        break;
        case RT_CAN_MODE_LISTEN:
            Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_BUS_MONITORING);
        break;
        case RT_CAN_MODE_LOOPBACK:
            Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_INTERNAL_LOOP_BACK);
        break;
        default: 
            Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_DISABLE);
        break;
    }
    
    switch(cfg->baud_rate)
    {
        case CAN1MBaud:   Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[0]); break;
        case CAN800kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[1]); break;
        case CAN500kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[2]); break;
        case CAN250kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[3]); break;
        case CAN125kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[4]); break;
        case CAN100kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[5]); break;
        case CAN50kBaud:  Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[6]); break;
        case CAN20kBaud:  Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[7]); break;
        case CAN10kBaud:  Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[8]); break;
        default:          Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[0]); break;
    }

//    rt_kprintf("xmc_canfd_config : CANFD_config %d\r\n", CANFD_config.canFDMode);
    status = Cy_CANFD_Init(CANFD_HW, 1, &CANFD_config,
                           &canfd_context);
    if(status != CY_CANFD_SUCCESS) return -RT_ERROR;
//    Cy_CANFD_Enable(CANFD_HW, 1);
    (void) Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);
    NVIC_EnableIRQ(NvicMux2_IRQn);   
                
    return RT_EOK;

}

static rt_err_t xmc_canfd_ctrl(struct rt_can_device *can, int cmd, void *arg)
{
    rt_uint32_t argval;
    struct xmc_canfd *pdrv_can;
    struct rt_can_filter_config *filter_cfg;
//rt_kprintf("xmc_canfd_ctrl is called\r\n");
    switch (cmd)
    {
        case RT_DEVICE_CTRL_CLR_INT:
            rt_kprintf("xmc_canfd_ctrl is called --- 1\r\n");
            argval = (rt_uint32_t) arg;
            if (argval == RT_DEVICE_FLAG_INT_RX)
            {
                (void) Cy_SysInt_Init(&canfd_irq_cfg, NULL);
                NVIC_DisableIRQ(NvicMux2_IRQn);
            }
            else if (argval == RT_DEVICE_FLAG_INT_TX)
            {
                (void) Cy_SysInt_Init(&canfd_irq_cfg, NULL);
                NVIC_DisableIRQ(NvicMux2_IRQn);
            }
            else if (argval == RT_DEVICE_CAN_INT_ERR)
            {
                (void) Cy_SysInt_Init(&canfd_irq_cfg, NULL);
                NVIC_DisableIRQ(NvicMux2_IRQn);
                
            }
        break;

        case RT_DEVICE_CTRL_SET_INT:
        rt_kprintf("xmc_canfd_ctrl is called --- 2\r\n");
#if 1
            argval = (rt_uint32_t) arg;
//            Cy_CANFD_Disable(CANFD_HW, 1);
            if( argval == RT_DEVICE_FLAG_INT_RX ) {
                 
                Cy_CANFD_SetInterruptMask( CANFD_HW, 1, CANFD_CH_M_TTCAN_IE_DRXE_Msk  |  /* Message stored to Rx Buffer */\
            											CANFD_CH_M_TTCAN_IE_RF1NE_Msk |  /* Rx FIFO 1 New Message */\
                                                        CANFD_CH_M_TTCAN_IE_RF0NE_Msk );
                /* Hook the interrupt service routine and enable the interrupt */
//                (void) Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);
//                NVIC_EnableIRQ(NvicMux2_IRQn);                                         
            }
            else if( argval == RT_DEVICE_FLAG_INT_TX ) {
                 
                Cy_CANFD_SetInterruptMask( CANFD_HW, 1, CANFD_CH_M_TTCAN_IE_DRXE_Msk  |  /* Message stored to Rx Buffer */\
            											CANFD_CH_M_TTCAN_IE_RF1NE_Msk |  /* Rx FIFO 1 New Message */\
                                                        CANFD_CH_M_TTCAN_IE_RF0NE_Msk | CANFD_CH_M_TTCAN_IE_TCE_Msk );
                
//                Cy_CANFD_SetInterruptMask( CANFD_HW, 1, CANFD_CH_M_TTCAN_IE_TCE_Msk );     
                /* Hook the interrupt service routine and enable the interrupt */
//                (void) Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);
//                NVIC_EnableIRQ(NvicMux2_IRQn);
            }
            else if (argval == RT_DEVICE_CAN_INT_ERR)
            {
                
            }
#endif           
//            Cy_CANFD_Enable(CANFD_HW, 1);
        break;

        case RT_CAN_CMD_SET_FILTER:
            rt_kprintf("xmc_canfd_ctrl is called --- 3\r\n");
        break;

        case RT_CAN_CMD_SET_MODE:
        rt_kprintf("xmc_canfd_ctrl is called --- 4\r\n");
            argval = (rt_uint32_t) arg;
            if (argval != RT_CAN_MODE_NORMAL &&
                argval != RT_CAN_MODE_LISTEN &&
                argval != RT_CAN_MODE_LOOPBACK &&
                argval != RT_CAN_MODE_LOOPBACKANLISTEN)
            {
                return -RT_ERROR;
            }
            if (argval != pdrv_can->device.config.mode)
            {
                pdrv_can->device.config.mode = argval;
                Cy_CANFD_Disable(CANFD_HW, 1);
                switch(argval)
                {
                    case RT_CAN_MODE_NORMAL:
                        Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_DISABLE);
                    break;
                    case RT_CAN_MODE_LISTEN:
                        Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_BUS_MONITORING);
                    break;
                    case RT_CAN_MODE_LOOPBACK:
                        Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_INTERNAL_LOOP_BACK);
                    break;
                    default: 
                        Cy_CANFD_TestModeConfig(CANFD_HW, 1, CY_CANFD_TEST_MODE_DISABLE);
                    break;
                }
                Cy_CANFD_Enable(CANFD_HW, 1);
            }
        break;

        case RT_CAN_CMD_SET_BAUD:
        rt_kprintf("xmc_canfd_ctrl is called --- 5\r\n");
            argval = (rt_uint32_t ) arg;
            /*just low to 10kbit/s*/
            if (argval != CAN1MBaud &&
                argval != CAN800kBaud &&
                argval != CAN500kBaud &&
                argval != CAN250kBaud &&
                argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                argval != CAN50kBaud  &&
                argval != CAN20kBaud  &&
                argval != CAN10kBaud)
            {
                return -RT_ERROR;
            }
            if (argval != pdrv_can->device.config.baud_rate)
            {
                pdrv_can->device.config.baud_rate = argval;
                Cy_CANFD_Disable(CANFD_HW, 1);
                switch(argval)
                {
                    case CAN1MBaud:   Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[0]); break;
                    case CAN800kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[1]); break;
                    case CAN500kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[2]); break;
                    case CAN250kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[3]); break;
                    case CAN125kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[4]); break;
                    case CAN100kBaud: Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[5]); break;
                    case CAN50kBaud:  Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[6]); break;
                    case CAN20kBaud:  Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[7]); break;
                    case CAN10kBaud:  Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[8]); break;
                    default:          Cy_CANFD_SetBitrate(CANFD_HW, 1, &xmc_fdcan_bitrate[0]); break;
                }
                Cy_CANFD_Enable(CANFD_HW, 1);
            }
        break;

        case RT_CAN_CMD_SET_PRIV:
        rt_kprintf("xmc_canfd_ctrl is called --- 6\r\n");
            argval = (rt_uint32_t) arg;
            if (argval != RT_CAN_MODE_PRIV &&
                    argval != RT_CAN_MODE_NOPRIV)
            {
                return -RT_ERROR;
            }
            if (argval != pdrv_can->device.config.privmode)
            {
                pdrv_can->device.config.privmode = argval;

                return RT_EOK;
            }
        break;

        case RT_CAN_CMD_GET_STATUS:

        break;
    }

    return RT_EOK;
}

static rt_ssize_t xmc_canfd_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    cy_en_canfd_status_t status;
//    struct xmc_canfd *pdrv_can;
    cy_stc_canfd_rx_buffer_t *prxbuf;
    struct rt_can_msg *pmsg;
    
    rt_kprintf("xmc_canfd_recvmsg is called\r\n");
    RT_ASSERT(can);
    RT_ASSERT(buf);

//    pdrv_can = (struct xmc_canfd *)can->parent.user_data;
    pmsg = (struct rt_can_msg *) buf;
    
//    status = Cy_CANFD_GetRxBuffer(CANFD_HW, 1, fifo, prxbuf);
    
    
    return RT_EOK;
}

static rt_ssize_t xmc_canfd_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t box_num)
{
    cy_en_canfd_status_t status;
    uint32_t tmp_u32DataLen;
//    struct xmc_canfd *pdrv_can;
    const cy_stc_canfd_tx_buffer_t *pbuf = &CANFD_txBuffer_0;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    
    RT_ASSERT(can);
	RT_ASSERT(buf);

//    pdrv_can = (struct xmc_canfd *)can->parent.user_data;
//	RT_ASSERT(pdrv_can);
    rt_kprintf("xmc_canfd_sendmsg is called\r\n");

	pmsg = (struct rt_can_msg *) buf;
    
    /* Check the parameters */
	if(pmsg->len > 8)
	{
		tmp_u32DataLen = 8;    // only support classical CAN
	}
    pbuf->t1_f->dlc = tmp_u32DataLen;
    
    if(pmsg->ide == RT_CAN_EXTID) {
        pbuf->t0_f->xtd = CY_CANFD_XTD_EXTENDED_ID;
    } else {
        pbuf->t0_f->xtd = CY_CANFD_XTD_STANDARD_ID;
    }
    
    if (RT_CAN_DTR == pmsg->rtr)
	{
		pbuf->t0_f->rtr = CY_CANFD_RTR_DATA_FRAME;
	}
	else
	{
		pbuf->t0_f->rtr = CY_CANFD_RTR_REMOTE_FRAME;
	}
    
    pbuf->t0_f->id = pmsg->id;
    memcpy(pbuf->data_area_f, pmsg->data, tmp_u32DataLen);
    rt_kprintf("xmc_canfd_sendmsg is called--1\r\n");
    status = Cy_CANFD_UpdateAndTransmitMsgBuffer( CANFD_HW, 1, pbuf, 0, &canfd_context);
    rt_kprintf("xmc_canfd_sendmsg is called--2\r\n");
    if(status != CY_CANFD_SUCCESS) return -RT_ERROR;
    rt_kprintf("xmc_canfd_sendmsg is called--3\r\n");
    return RT_EOK;
}


static const struct rt_can_ops xmc_canfd_ops =
{
    .configure = xmc_canfd_config,
    .control   = xmc_canfd_ctrl,
    .recvmsg   = xmc_canfd_recvmsg,
    .sendmsg   = xmc_canfd_sendmsg,
};


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
#if 1
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
#else

#endif

}

void canfd_tx_callback(void)
{
#ifdef BSP_USING_CANFD0
    rt_hw_can_isr(&xmcDrvCAN0.device, RT_CAN_EVENT_TX_DONE | 0 );
#endif
}

#endif
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
    Cy_CANFD_IrqHandler(CANFD_HW, 1, &canfd_context);
    rt_interrupt_leave();
}


int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
//    cy_en_canfd_status_t status;
    config.baud_rate = CAN1MBaud;
    config.msgboxsz = 48;
    config.sndboxnumber = 1;
    config.mode = RT_CAN_MODE_NORMAL;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks   = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 14;
#endif    
    
//    status = Cy_CANFD_Init(CANFD_HW, 1, &CANFD_config,
//                           &canfd_context);
//    if(status != CY_CANFD_SUCCESS) return -RT_ERROR;
    
//    (void) Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);
//    NVIC_EnableIRQ(NvicMux2_IRQn);
//    Cy_CANFD_SetInterruptMask( CANFD_HW, 1, CANFD_CH_M_TTCAN_IE_DRXE_Msk  |  /* Message stored to Rx Buffer */\
//                                        CANFD_CH_M_TTCAN_IE_RF1NE_Msk |  /* Rx FIFO 1 New Message */\
//                                        CANFD_CH_M_TTCAN_IE_RF0NE_Msk | CANFD_CH_M_TTCAN_IE_TCE_Msk );
    xmcDrvCAN0.device.config = config;                                   
    /* register CAN_FD device */
    rt_hw_can_register(&xmcDrvCAN0.device,
                       xmcDrvCAN0.name,
                       &xmc_canfd_ops,
                       &xmcDrvCAN0);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_can_init);

#endif
