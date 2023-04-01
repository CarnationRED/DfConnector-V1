#include "VCI.h"
#include "main.h"
#include "buzzer.h"
#include "delay.h"
#include "Messages.h"
#include "M8266WIFIDrv.h"
#include "gd32c10x_fmc.h"
#define BUFFER_LEN 1024


u8 buffer[BUFFER_LEN];
VCI_CTL_DFU_STRUCT dfu_buffer;
u8 dfu_data_serial;
u8 success_beep[]={9,8,6,8,0,8,6,8,0,8,6,8,6,5,0,0,9,8,6,8,0,8,6,8,0,8,10,9,8,8,0,0,5,6,10,10,10,9,10,10,9,10,12,10,0,0,10,10,10,9,9,9,8,9,8,6,10,9,0,0,9,8,6,8,0,8,6,8,0,8,6,8,6,5,0,0,5,6,10,12,0,12,10,12,0,12,10,9,8,8,0,0,9,8,9,10,9,9,8,9,0,0,8,6,9,8,8,6,8,8,8,0,0,11,10,9,0,0,12,12,10,9,10,6,0,9,10,12,10,9,0,0,12,12,10,9,8,5,0,9,10,12,9,8,0,0,8,9,10,12,13,12,10,12,10,10,9,9,0,0,8,9,8,9,8,9,10,12,10,0,0,12,12,10,9,10,6,0,9,10,12,10,9,0,0,12,12,10,9,8,5,0,9,10,12,9,8,0,0,8,9,10,12,13,12,10,12,10,10,9,9,0,0,5,10,9,9,8,0};

VCI_CTL_STRUCT vci_ctl_type_header;
	
extern u8 current_channel;
extern u8 can_setting_fd_flag[CAN_NUMBERS];
extern u8 can_setting_auto_retrans[CAN_NUMBERS];
extern u8 can_setting_fd_iso_bosch[CAN_NUMBERS];
extern u32 can_setting_buadrate[CAN_NUMBERS];
extern u32 can_setting_buadrate_fd[CAN_NUMBERS];
extern WIFI_STATUS wifi_status;
extern u8 can_msg_recv_start;
extern u8 can_msg_recv_count;
extern u8 can_cmd_recv_start;
extern u8 can_cmd_recv_count;

u8 recv(void)
{
		u16 dlen, status;
		u8 chnl;
		dlen = M8266WIFI_SPI_RecvData(buffer, BUFFER_LEN, 1, &chnl, &status);
		if(dlen > 0)
		{
				if(chnl != WIFI_CAN_CTL_CHNL || (status & 0xff) != 0)
				{
						wifi_status = WIFI_STATUS_ERR_CTLDATER;
						return 0;
				}
				return dlen;
		}
		return 0;
}

void set_channel(void)
{
		u16 dlen = recv();
		u16 i;
		if(dlen == 1)
				set_can_channel(buffer[0]);
		else 
				wifi_status = WIFI_STATUS_ERR_CTLDATER;
}

void set_filter(void)
{
		u16 dlen = recv();
		u16 i;
		if(dlen % sizeof(CAN_CTL_SETFILTER_STRUCT) == 0)
			for(i = 0;i < dlen; i += sizeof(CAN_CTL_SETFILTER_STRUCT))
				{
						CAN_CTL_SETFILTER_STRUCT *s = (CAN_CTL_SETFILTER_STRUCT *)&buffer[i];
						if(s->ext_flag)
								can_set_id_filter_extended(s->filter_no, s->id1, s->id2, s->can_no, s->enable);
						else
								can_set_id_filter_standard(s->filter_no, s->id1, s->id2, s->id3, s->id4, s->can_no, s->enable);
				}
		else 
				wifi_status = WIFI_STATUS_ERR_CTLDATER;
}

void setting(void)
{
	u16 dlen = recv();
	u16 i;
	if(dlen % sizeof(CAN_CTL_SETTING_STRUCT) == 0)
			for(i = 0;i < dlen; i+=sizeof(CAN_CTL_SETTING_STRUCT))
			{
					CAN_CTL_SETTING_STRUCT *s = (CAN_CTL_SETTING_STRUCT *)&buffer[i];
					u8 can_no = s->can_no;
					if(can_no >= CAN_NUMBERS) continue;
					can_setting_fd_flag[can_no] = s->fd_flag;
					can_setting_auto_retrans[can_no] = s->auto_retrans;
					can_setting_fd_iso_bosch[can_no] = s->fd_iso_bosch;
					can_setting_buadrate[can_no] = s->buadrate;
					can_setting_buadrate_fd[can_no] = s->buadrate_fd;
			}
	else 
			wifi_status = WIFI_STATUS_ERR_CTLDATER;
}


void device_firmware_upgrade(void)
{
		u16 dlen, status, i;
		u8 chnl;
		u32 dfu_addr;
		u8 total;
		u8 rom1 = 1;
		//确定刷写位置
		if((*(u32*)BT_JUMP_ADDR) == APP_ROM1_ADDR)
		{
				dfu_addr = APP_ROM2_ADDR;
				rom1 = 0;
		}
		else dfu_addr = APP_ROM1_ADDR;
		
		wifi_disableIT();
		do
		{
				do
				{
						dlen = 0;
						dlen = M8266WIFI_SPI_RecvData((u8*) &dfu_buffer, sizeof(VCI_CTL_DFU_STRUCT), 1, &chnl, &status);
						if(chnl == WIFI_VCI_CTL_CHNL && dlen == sizeof(VCI_CTL_DFU_STRUCT))
						{
								total = dfu_buffer.total;
								break;
						}
				}while(M8266WIFI_SPI_Has_DataReceived());
				
				if(dlen == sizeof(VCI_CTL_DFU_STRUCT))
				{
						if(chnl != WIFI_VCI_CTL_CHNL || (status & 0xff) != 0)
						{
								wifi_status = WIFI_STATUS_ERR_CTLDATER;
								if(get_vci_status() == VCI_STATUS_DFU) 
								{
										set_vci_status(VCI_STATUS_RESET);
										dfu_data_serial = 0;
										fmc_lock();
										fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
										wifi_enableIT();
										return;
								}
								else return;
						}
						//开启DFU，解锁fmc
						if(get_vci_status() != VCI_STATUS_DFU)
						{
								if(dfu_buffer.serial != 0)
									return;
								set_vci_status(VCI_STATUS_DFU);
								dfu_data_serial = 0;
								fmc_unlock();
						}
						//固件分段的序号不一致，锁fmc，重启VCI
						if(dfu_data_serial != dfu_buffer.serial)
						{
								set_vci_status(VCI_STATUS_RESET);
								dfu_data_serial = 0;
								fmc_lock();
								fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
								wifi_enableIT();
								return;
						}
						
						fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
						fmc_page_erase(dfu_addr);
						fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
					
						for(i = 0;i < 1024;i += 4)
						{
								if(FMC_READY !=	fmc_word_program(dfu_addr, *(u32*)(dfu_buffer.payload + i)))
										;
								dfu_addr += 4;
						}
						fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
					
						if(++dfu_data_serial == total)
								break;
				}
				
				wifi_SendData(&dfu_data_serial, 1, WIFI_VCI_CTL_CHNL);
				
				i = 0;
				while(i++ < 600 && !M8266WIFI_SPI_Has_DataReceived())
					delay_ms(10);
				
				if(!M8266WIFI_SPI_Has_DataReceived()) break;
		}while(1);
		
		if(dfu_buffer.total - 1 == dfu_buffer.serial && dfu_buffer.total == total)
		{
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
				fmc_page_erase(BT_JUMP_ADDR_PAGE);
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
			
				if(rom1)
						dfu_addr = APP_ROM1_ADDR;
				else dfu_addr = APP_ROM2_ADDR;
				
				if(FMC_READY !=	fmc_word_program(BT_JUMP_ADDR, dfu_addr))
				{
						set_vci_status(VCI_STATUS_RESET);
						dfu_data_serial = 0;
						fmc_lock();
						fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
						wifi_enableIT();
						return;
				}
				fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
			
				wifi_enableIT();
				beep_setvolume(90);
				while(1)
				{
					beep(success_beep,sizeof(success_beep),240);
				}
		}
		dfu_data_serial = 0;
		fmc_lock();
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR | FMC_STAT_PGAERR);
		wifi_enableIT();
		return;
}

void vci_ctl_recv(VCI_CTL_TYPE type)
{
		u16 dlen;
		u16 i;
		switch(type)
		{
				case CAN_CTL_TYPE_SETCHANNEL:
						set_channel();
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case CAN_CTL_TYPE_SETFILTER:
						set_filter();
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case CAN_CTL_TYPE_CLEARFILTER:
								can_clear_filter(buffer[0] == 0 ? CAN0 : CAN1);
								wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case CAN_CTL_TYPE_CLEARBUFFER:
						can_msg_buffer_clear();
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case CAN_CTL_TYPE_REINIT:
						vci_can_init();
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case CAN_CTL_TYPE_STOP:
						vci_can_stop();
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						set_vci_status(VCI_STATUS_RESET);
						break;
				case CAN_CTL_TYPE_SETTING:
						setting();
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case VCI_CTL_TYPE_REINIT:
						set_vci_status (VCI_STATUS_RESET);
						wifi_SendData((u8*)&type, sizeof(type), WIFI_CAN_CTL_CHNL);
						break;
				case VCI_CTL_TYPE_TIMESYNC:
						break;
				case VCI_CTL_TYPE_DFU:
						device_firmware_upgrade();
						break;
				default:
					break;
		}
}