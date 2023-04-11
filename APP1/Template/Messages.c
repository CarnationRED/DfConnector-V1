#ifndef __MSG_C__
#define __MSG_C__
#include "Messages.h"
#include "M8266WIFIDrv.h"
#include "main.h"
#include "VCI.h"

volatile MESSAGING_STATUS messaging_status;
static u8 can_recv_serial = 0;
static u8 can_send_serial = 0;
CAN_RECV_MSG can_recv_queue[CAN_RECV_BUFFER];
CAN_SEND_CMD can_send_queue[CAN_SEND_BUFFER];
static u8 can_msg_recv_start;
static u8 can_msg_recv_count;
static u8 can_cmd_recv_start;
static u8 can_cmd_recv_count;
static u8 can_cmd_send_ack;
void can_cmd_sendonce(CAN_SEND_CMD* cmd);
volatile u8 can_traffic_indicator;

//steps:
//1, in can recieve interrupt, call can_msg_recv
//2, in main loop, call can_msg_send2wifi to send msg via tcp
/*!
    \brief	stores can msg to queue's tail, add one to recv_count
*/
void can_msg_recv(u8 channel,u32 can, u8 fifo)
{
		CAN_RECV_MSG *msg;
		u8 id;
		if(can_msg_recv_count == CAN_RECV_BUFFER)
		{
				messaging_status = MSG_STATUS_ERR_RECVBUFFER_FULL;
				if(fifo == CAN_FIFO0)
						CAN_RFIFO0(can) |= CAN_RFIFO0_RFD0;
				else
						CAN_RFIFO1(can) |= CAN_RFIFO1_RFD1;
				return;
		}
		id = can_msg_recv_start + can_msg_recv_count;//new message position
		if(id >= CAN_RECV_BUFFER) id -= CAN_RECV_BUFFER;
		
		msg = &(can_recv_queue[id]);
		msg->channel = channel;
		msg->serial = can_recv_serial++;
		can_message_receive(can, fifo, (can_receive_message_struct*)msg);
		msg->time = current_time01ms();
		can_msg_recv_count++;
		
		if(can_traffic_indicator < 250)
				can_traffic_indicator += 5;
}

/*!
    \brief	check recv_count, if non-zero, send the data, offset recv_start, subsctract recv_count
*/
void can_msg_send2wifi(void)
{
		u16 size; u8 count;
		u8 recved = can_msg_recv_count;
		if(recved == 0) return;
		count = recved < MAX_MSG_PER_SEND? recved : MAX_MSG_PER_SEND;
		if(can_msg_recv_start + count > CAN_RECV_BUFFER)
				count = CAN_RECV_BUFFER - can_msg_recv_start;
		size = count * sizeof(CAN_RECV_MSG);
		if(wifi_SendData((u8*)&can_recv_queue[can_msg_recv_start], size, WIFI_CAN_MSG_CHNL) != size)
				messaging_status = MSG_STATUS_ERR_SEND_DATA_JAMMED;
		can_msg_recv_start += count;
		if(can_msg_recv_start >= CAN_RECV_BUFFER)
				can_msg_recv_start -= CAN_RECV_BUFFER;
		can_msg_recv_count -= count;
}
u16 can_cmd_recv_send_iterate(u8 iterate0)
{
		u16 status,dlen;
		CAN_SEND_CMD cmd;
		dlen = M8266WIFI_SPI_RecvData((u8*)(&cmd) + (iterate0? 1:0), sizeof(CAN_SEND_CMD) - (iterate0?1:0), 2, NULL, &status);
	
	if(dlen + 8 >= sizeof(CAN_SEND_CMD))
	{
			if(iterate0)
					dlen++;
		
			if(dlen != sizeof(CAN_SEND_CMD))
			{
				volatile u8 rr=1;
			}else{
				can_cmd_sendonce(&cmd);
			}
	}
		return status;
}

volatile u8 offset=8;
u16 can_cmd_recv_iterate(u8 iterate0)
{
		CAN_SEND_CMD *cmd;
		u8 id, continous_space;
		u16	dlen;
		u16 status;
		if(can_cmd_recv_count == CAN_SEND_BUFFER)
		{
				messaging_status = MSG_STATUS_ERR_SENDBUFFER_FULL;
				return 0xffff;
		}
		id = can_cmd_recv_start + can_cmd_recv_count;//new cmd position
		continous_space = CAN_SEND_BUFFER - can_cmd_recv_count;
		if(id >= CAN_SEND_BUFFER) id -= CAN_SEND_BUFFER;
		else continous_space -= can_cmd_recv_start;
				
		if(continous_space < 1)
		{
				messaging_status = MSG_STATUS_ERR_SENDBUFFER_FULL;
				return 0xffff;
		}
		
		cmd = &(can_send_queue[id]);
		dlen = M8266WIFI_SPI_RecvData(((u8*)cmd) + (iterate0? 1:0) + offset - 8, continous_space * sizeof(CAN_SEND_CMD), 2, NULL, &status);
		if(iterate0 && dlen + 1 >= sizeof(CAN_SEND_CMD))
			dlen++;
		id = (dlen+8) / sizeof(CAN_SEND_CMD);
		if(dlen != (u16)id * sizeof(CAN_SEND_CMD))
		{
				offset = dlen + 8 - (u16)id * sizeof(CAN_SEND_CMD);
				//messaging_status = MSG_STATUS_ERR_SEND_DATA_ERROR;
		}
		else offset = 8;
				can_cmd_send_ack = cmd->cmd_type == 1;
		can_cmd_recv_count += id;
		return status;
}
//steps:
//1, in wifi recieve interrupt, populate tcp data into pointer provided by get_can_send_queue, 
//2, in main loop, send all 
//3, call corresponding methods to set can filters and send can messages
void can_cmd_recv(void)
{
		u16 status;
		u8 ret, iterate0 = 1;
		can_cmd_send_ack = 0;
		while(1)
		{
				status = can_cmd_recv_send_iterate(iterate0) & 0x00ff;
				iterate0 = 0;
				if(status == 0x24 || status == 0x23)
						continue;
				break;
		}
		if(can_cmd_send_ack)
		{
				ret = CAN_CTL_CAN_CMD;
				wifi_SendData(&ret, 1, WIFI_CAN_CTL_CHNL);
		}
}
void can_cmd_sendonce(CAN_SEND_CMD* cmd)
{
				can_message_transmit(set_can_channel(cmd->channel), &(cmd->msg));
				if(can_traffic_indicator < 250)
					can_traffic_indicator += 5;
}
void can_cmd_send2can(void)
{
		while(can_cmd_recv_count != 0)
		{
				CAN_SEND_CMD cmd = can_send_queue[can_cmd_recv_start];
				can_cmd_sendonce(&cmd);
				can_cmd_recv_start++;
				if(can_cmd_recv_start >= CAN_SEND_BUFFER)
						can_cmd_recv_start -= CAN_SEND_BUFFER;
				can_cmd_recv_count--;
		}
}

void can_msg_buffer_clear()
{
		can_msg_recv_start=0;
	  can_msg_recv_count=0;
	  can_cmd_recv_start=0;
	  can_cmd_recv_count=0;
}
#endif

