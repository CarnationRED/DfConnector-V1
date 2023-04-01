/*!
    \file    gd32c10x_it.c
    \brief   interrupt service routines
    
    \version 2020-12-31, V1.0.0, firmware for GD32C10x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32c10x_it.h"
#include "systick.h"
#include "delay.h"
#include "Messages.h"
#include "main.h"
#include "M8266WIFIDrv.h"
#include "VCI.h"

extern can_receive_message_struct recvmsg;
extern FlagStatus can0_receive_flag;
extern FlagStatus can1_receive_flag;

volatile u32 last_wifi_time=0;

extern u8 current_channel;
extern u8 can_setting_fd_flag[CAN_NUMBERS];
extern u8 can_setting_auto_retrans[CAN_NUMBERS];
extern u8 can_setting_fd_iso_bosch[CAN_NUMBERS];
extern u32 can_setting_buadrate[CAN_NUMBERS];
extern u32 can_setting_buadrate_fd[CAN_NUMBERS];
extern VCI_CTL_STRUCT vci_ctl_type_header;
/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
volatile bit_status systick_state;
volatile u16 systick_cycles;
void SysTick_Handler(void)
{
		systick_state = SET;
		systick_cycles++;
//    delay_decrement();
}


/*!
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN0_RX0_IRQHandler(void)
{
		can_msg_recv(current_channel, CAN0, CAN_FIFO0);
//	set_vci_status(VCI_STATUS_DFU);
}

/*!
    \brief      this function handles CAN1 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN1_RX0_IRQHandler(void)
{
		can_msg_recv(current_channel, CAN1, CAN_FIFO0);
}

void EXTI4_IRQHandler(void)
{
		u16 dlen, status;
		u8 chnl;
		exti_interrupt_flag_clear(EXTI_4);
//		if(M8266WIFI_SPI_Has_DataReceived())
		{
				vci_ctl_type_header.type = 0xffffffff;
				dlen = M8266WIFI_SPI_RecvData((u8*)&vci_ctl_type_header, sizeof(vci_ctl_type_header), 1, &chnl, &status);
				if(dlen > 0)
				{
						last_wifi_time = current_time01ms();
						if(chnl == WIFI_CAN_MSG_CHNL)
						{
								switch(vci_ctl_type_header.type)
								{
										case CAN_CTL_CAN_CMD:
												can_cmd_recv();
												break;
										default:
												break;
								}
						}
						else if(chnl == WIFI_CAN_CTL_CHNL || chnl == WIFI_VCI_CTL_CHNL)
						{
								switch(vci_ctl_type_header.type)
								{
										case CAN_CTL_CAN_CMD:
										case CAN_CTL_TYPE_SETCHANNEL:
										case CAN_CTL_TYPE_SETFILTER:
										case CAN_CTL_TYPE_CLEARFILTER:
										case CAN_CTL_TYPE_CLEARBUFFER:
										case CAN_CTL_TYPE_REINIT:
										case CAN_CTL_TYPE_STOP:
										case CAN_CTL_TYPE_SETTING:	
										case VCI_CTL_TYPE_REINIT:
										case VCI_CTL_TYPE_TIMESYNC:
										case VCI_CTL_TYPE_DFU:
												vci_ctl_recv(vci_ctl_type_header.type);
												break;
										case VCI_CTL_TYPE_HEARTBEAT:
												clear_heartbeat_timeout();
												break;
										default:
												break;
								}
						}
				}
		}
}

void TIMER4_IRQHandler(void)
{
		tim4_it();
		/* clear update interrupt bit */
		timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_UP);
}

void TIMER3_IRQHandler(void)
{
		tim3_it();
		/* clear update interrupt bit */
		timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
}
