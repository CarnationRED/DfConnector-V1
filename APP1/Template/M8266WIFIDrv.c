/********************************************************************
 * M8266WIFIDrv.c																									 *
 * 使用M8266WIFI的驱动，要求在自己的项目实现如下函数，以供驱动调用	*
 * void M8266HostIf_Set_nRESET_Pin(u8 level);											 *
 * void M8266HostIf_Set_SPI_nCS_Pin(u8 level);											*
 * void M8266HostIf_delay_us(u8 nus);															 *
 * u8	 M8266HostIf_SPI_ReadWriteByte(u8 byte); // 大多平台不需.		*
 ********************************************************************/

#include "stdio.h"
#include "string.h"	
#include "delay.h"
#include "WIFI.h"
#include "M8266WIFIDrv.h"
#include "gd32c10x.h"

//////////////////////////////////////////////////////////////////////////////////////
// BELOW FUNCTIONS ARE REQUIRED BY M8266WIFIDRV.LIB. 
// PLEASE IMPLEMENTE THEM ACCORDING TO YOUR HARDWARE
//////////////////////////////////////////////////////////////////////////////////////
/***********************************************************************************
 * M8266HostIf_Set_nRESET_Pin																											*
 * Description																																		 *
 *		向WIFI模块的nReset输出高或低电平							 				 											 *
 *		You may update the macros of GPIO PIN usages for nRESET from brd_cfg.h			 *
 *		You are not recommended to modify codes below please												 *
 * Parameter(s):																																	 *
 *		1. level: LEVEL output to nRESET pin																				 *
 *							0 = output LOW	onto nRESET																				*
 *							1 = output HIGH onto nRESET																				*
 * Return:																																				 *
 *		None																																				 *
 ***********************************************************************************/
void M8266HostIf_Set_nRESET_Pin(u8 level)
{
		if(level!=0)
			gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_RST_PIN, SET);
		else
			gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_RST_PIN, RESET);
}
/*******************************************************90****************************
 * M8266HostIf_Set_SPI_nCS_PIN																										 *
 * Description																																		 *
 *		向WIFI模块的nCS输出高或低电平							 					 											 *
 *		You may update the macros of GPIO PIN usages for SPI nCS from brd_cfg.h			*
 *		You are not recommended to modify codes below please												 *
 * Parameter(s):																																	 *
 *		1. level: LEVEL output to SPI nCS pin																				*
 *							0 = output LOW	onto SPI nCS																			 *
 *							1 = output HIGH onto SPI nCS																			 *
 * Return:																																				 *
 *		None																																				 *
 ***********************************************************************************/
void M8266HostIf_Set_SPI_nCS_Pin(u8 level)
{
		if(level!=0)
			gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_NCS_PIN, SET);
		else
			gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_NCS_PIN, RESET);
}

/***********************************************************************************
 * M8266WIFIHostIf_delay_us																												*
 * Description																																		 *
 *		延时微秒.					 																												 *
 * Parameter(s):																																	 *
 *		1. nus: the micro seconds to delay																					 *
 * Return:																																				 *
 *		none																																				 *
 ***********************************************************************************/
void M8266HostIf_delay_us(u8 nus)
{
	 delay_us(nus);
}

/***********************************************************************************
 * M8266HostIf_SPI_ReadWriteByte																									 *
 * Description																																		 *
 *		向WIFI模块写一个byte，然后读一个byte																				 *
 *		and read back a byte from the SPI bus MISO meanwhile												 *
 *		You may update the macros of SPI usage from brd_cfg.h												*
 * Parameter(s):																																	 *
 *		1. TxdByte: the byte to be sent over MOSI																		*
 * Return:																																				 *
 *		1. The byte read back from MOSI meanwhile																		*																																				 *
 ***********************************************************************************/
u8 M8266HostIf_SPI_ReadWriteByte(u8 TxdByte)
{
		uint32_t retry=0;
    // 等待发送缓冲区为空
    while(RESET == spi_i2s_flag_get(WIFI_SPI_PORT,SPI_FLAG_TBE));
    // 发送一个字节
    spi_i2s_data_transmit(WIFI_SPI_PORT,TxdByte);
	
    // 等待接收缓冲区非空
    while(RESET == spi_i2s_flag_get(WIFI_SPI_PORT,SPI_FLAG_RBNE) && retry++<100);
    // 读取一个字节
    return spi_i2s_data_receive(WIFI_SPI_PORT);
	
//		while((SPI_STAT(SPIx)&1<<1)==0)
//		{
//				retry++;
//				if(retry>=0xFFFE)
//						return 0;
//		}
//		
//		SPI_DATA(SPIx)=(uint32_t)byte;
//		retry=0;
//		while((SPI_STAT(SPIx)&1<<0)==0)
//		{
//				retry++;
//				if(retry>=0xFFFE)
//						return 0;
//		}
//		rxdata=SPI_DATA(SPIx);
//		return (uint8_t)rxdata;
}
