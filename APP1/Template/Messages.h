#ifndef __MSG__
#define __MSG__
#include "WIFI.h"
#include "delay.h"
#define CAN_RECV_BUFFER 64
#define CAN_SEND_BUFFER 64
#define MAX_MSG_PER_SEND (1400 / sizeof(CAN_RECV_MSG))

typedef struct//sizeof=88 byte, actual size = 85
{
    uint32_t rx_sfid;                                 //4  				  /*!< standard format frame identifier */
    uint32_t rx_efid;                                 //4  					/*!< extended format frame identifier */
    uint8_t rx_ff;                                    //1  					/*!< format of frame, standard or extended format */
    uint8_t rx_ft;                                    //1  					/*!< type of frame, data or remote */
    uint8_t rx_dlen;                                  //1  					/*!< data length */
    uint8_t rx_data[64];                              //64 					/*!< receive data */
    uint8_t rx_fi;                                    //1  					/*!< filtering index */
    uint8_t fd_flag;                                  //1  					/*!< CAN FD frame flag */
    uint8_t fd_brs;                                   //1  					/*!< bit rate of data switch */
    uint8_t fd_esi;                                   //1  					/*!< error status indicator */
																											//79 above, 9 bellow
    u8 msg_type;//channel															//1
    u32 time;																					//4
    u8 serial;//CAN报文在接收时的序列号								//1
    u8 channel;//channel															//1 (2 padding)
}CAN_RECV_MSG;


typedef struct//sizeof=80 byte, actual size = 78
{
    u32 tx_sfid;                                      //4          /*!< standard format frame identifier */
    u32 tx_efid;                                      //4          /*!< extended format frame identifier */
    u8 	tx_ff;                                        //1          /*!< format of frame, standard or extended format */
    u8 	tx_ft;                                        //1          /*!< type of frame, data or remote */
    u8 	tx_dlen;                                      //1          /*!< data length */
    u8 	tx_data[64];                                  //64         /*!< transmit data */
    u8 	fd_flag;                                      //1          /*!< CAN FD frame flag */
    u8 	fd_brs;                                       //1          /*!< bit rate of data switch */
    u8 	fd_esi;                                       //1
																											//   78 above, 6 bellow
		u8 cmd_type;//指令类型,0:回传acknowledge，1:无sck	//1
    u8 serial;//CAN报文发送的序列号										//1
    u8 channel;//channel															//1 (3 padding)
}CAN_SEND_CMD;


typedef enum
{	
	MSG_STATUS_NORMAL,
	MSG_STATUS_ERR_RECVBUFFER_FULL,
	MSG_STATUS_ERR_SENDBUFFER_FULL,
	MSG_STATUS_ERR_SEND_DATA_JAMMED,
	MSG_STATUS_ERR_SEND_DATA_ERROR,
}MESSAGING_STATUS;

/*!
    \brief      configure BSP
*/

can_receive_message_struct *get_can_recv_ptr(u8 channel);
void can_msg_recv(u8 channel,u32 can, u8 fifo);
void can_msg_send2wifi(void);

void can_cmd_recv(void);
void can_cmd_send2can(void);

void can_msg_buffer_clear(void);

#endif
