#ifndef __VCI__
#define __VCI__
#include "stdint.h"
#include "WIFI.h"
#include "main.h"
#include "Messages.h"

#define TOTAL_ROM_SIZE (128*1024U)
#define BT_ROM_SIZE (4*1024U)
#define APP_ROM_SIZE ((TOTAL_ROM_SIZE - BT_ROM_SIZE) / 2U)
#define ROM_BASE_ADDR 0x8000000U
#define APP_ROM1_ADDR (ROM_BASE_ADDR + BT_ROM_SIZE)
#define APP_ROM2_ADDR (APP_ROM1_ADDR + APP_ROM_SIZE)
#define BT_JUMP_ADDR (ROM_BASE_ADDR + BT_ROM_SIZE - 4U)
#define BT_JUMP_ADDR_PAGE (ROM_BASE_ADDR + BT_ROM_SIZE - 1024U)

typedef enum
{
		CAN_CTL_CAN_CMD,
		CAN_CTL_TYPE_SETCHANNEL,
		CAN_CTL_TYPE_SETFILTER,
		CAN_CTL_TYPE_CLEARFILTER,
		CAN_CTL_TYPE_CLEARBUFFER,
		CAN_CTL_TYPE_REINIT,
		CAN_CTL_TYPE_STOP,
		CAN_CTL_TYPE_SETTING,
		VCI_CTL_TYPE_REINIT,
		VCI_CTL_TYPE_TIMESYNC,
		VCI_CTL_TYPE_HEARTBEAT,
		VCI_CTL_TYPE_DFU,
}VCI_CTL_TYPE;

typedef struct
{
		VCI_CTL_TYPE type;
}VCI_CTL_STRUCT;

typedef struct
{
		VCI_STATUS vci_stat;
		WIFI_STATUS wifi_stat;
		MESSAGING_STATUS msg_stat;
		u8 reserved;
}VCI_CTL_HEARTBEAT_STRUCT;


typedef struct
{
		u8 filter_no;
		u8 can_no;
		u8 enable;
		u8 ext_flag;
		u32 id1;
		u32 id2;
		u32 id3;
		u32 id4;
}CAN_CTL_SETFILTER_STRUCT;

typedef struct
{
		u8 can_no;
		u8 fd_flag;
		u8 auto_retrans;
		u8 fd_iso_bosch;
		u32 buadrate;
		u32 buadrate_fd;
}CAN_CTL_SETTING_STRUCT;

typedef struct
{
		u8 serial;
		u8 total;
		u8 payload[1024];
}VCI_CTL_DFU_STRUCT;

void vci_ctl_recv(VCI_CTL_TYPE type);

#endif
