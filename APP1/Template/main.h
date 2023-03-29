#ifndef __MAINH__
#define __MAINH__
#include "stdint.h"
#include "gd32c10x.h"
#include "WIFI.h"
#define WIFI_CAN_MSG_PORT			1112
#define WIFI_CAN_CTL_PORT			1113
#define WIFI_VCI_CTL_PORT			1114
#define WIFI_CAN_MSG_CHNL			1
#define WIFI_CAN_CTL_CHNL			2
#define WIFI_VCI_CTL_CHNL			3
#define CAN_NUMBERS						2

typedef enum
{
		VCI_STATUS_RUN,
		VCI_STATUS_RESET,
		VCI_STATUS_DFU,
}VCI_STATUS;

u32 set_can_channel(u8 channel);


void can_set_id_filter_standard(u8 filter_no, uint16_t id1, uint16_t id2, uint16_t id3, uint16_t id4, uint32_t can_number, ControlStatus enable);
void can_set_id_filter_extended(u8 filter_no, uint32_t id1, uint32_t id2, uint32_t can_number, ControlStatus enable);
void can_clear_filter(uint32_t can_number);
void vci_can_init(void);
void vci_can_stop(void);

void set_vci_status(VCI_STATUS s);
VCI_STATUS get_vci_status(void);

#endif
