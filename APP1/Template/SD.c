#ifndef __SD__
#define __SD__
#include "SD.h"
#define sd_set GPIO_BOP(SD_NCS_PORT) = (u32)SD_NCS_PIN;
#define sd_reset GPIO_BC(SD_NCS_PORT) = (u32)SD_NCS_PIN;

u8 rw(u8 data)
{
		u16 retry = 0;
		u32 SPIx = SD_SPI_PORT;
    // 等待发送缓冲区为空
    while(RESET == spi_i2s_flag_get(SPIx,SPI_FLAG_TBE));
    // 发送一个字节
    spi_i2s_data_transmit(SPIx,data);
	
    // 等待接收缓冲区非空
    while((RESET == spi_i2s_flag_get(SPIx,SPI_FLAG_RBNE)) && retry++<500);
    // 读取一个字节
    return spi_i2s_data_receive(SPIx);
//		while((SPI_STAT(SPIx)&1<<1)==0)
//		{
//				retry++;
//				if(retry>=0xFFFE)
//						return 0;
//		}
//		
//		SPI_DATA(SPIx)=(uint32_t)data;
//		retry=0;
//		while((SPI_STAT(SPIx)&1<<0)==0)
//		{
//				retry++;
//				if(retry>=0xFFFE)
//						return 0;
//		}
//		return (u8)SPI_DATA(SPIx);
}

u8 sd_cmd(u8 cmd, u32 data, u8 crc)
{
	u8 result = 0, i = 0;
	sd_set
	//rw(0xff);
	delay_ms(20);
	sd_reset
	do{
		i=rw(0xFF);
	}while(i != 0xFF);
	rw(cmd);
	
  rw(data >> 24);
  rw(data >> 16);
  rw(data >> 8);
  rw(data);
	rw(crc);
	rw(0xff);
	i = 0;
  if(cmd==0x4c)rw(0xFF);
  do
	{
		i=rw(0xFF);
	}while(i&0X80);
	return i;
}

void sd_init()
{
		u8 i;
		spi_parameter_struct spi_init_struct;
		// Enable the clock for GPIOB and SPI2
		rcu_periph_clock_enable(RCU_SPI2);
		rcu_periph_clock_enable(SD_PIN_PORT);
		rcu_periph_clock_enable(SD_NCS_PORT);
		rcu_periph_clock_enable(RCU_AF);

		// Configure the GPIO pins for SPI function
		gpio_init(SD_PIN_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SD_SCK_PIN | SD_MOSI_PIN);
		gpio_init(SD_PIN_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, SD_MISO_PIN);
		gpio_init(SD_NCS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SD_NCS_PIN);
	
		sd_set
		// Reset the SPI2 peripheral
		spi_i2s_deinit(SD_SPI_PORT);
		// Configure the parameters for SPI2
		spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX; // Full duplex mode
		spi_init_struct.device_mode = SPI_MASTER; // Master or slave mode
		spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT; // 8-bit data frame size
		spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE; // Clock polarity and phase mode 0
		spi_init_struct.nss = SPI_NSS_SOFT; // Soft NSS signal
		spi_init_struct.prescale = SPI_PSC_256; // PCLK1:60MHz, 8:7.5M, 16:7.5M, 256:234375Hz
		spi_init_struct.endian = SPI_ENDIAN_MSB; // MSB first
	 
		// Initialize the SPI2 peripheral with the parameters 
		spi_init(SD_SPI_PORT, &spi_init_struct);
		//Enable SPI2
		spi_enable(SD_SPI_PORT);
		delay_us(100);
		sd_set
		delay_ms(1);
		
		for(i = 0;i < 0x2f;i++)
				rw(0xff);
				
		i = 0;
		while(i++<200)
		{
			if(0x01 == sd_cmd(0x40, 0, 0x95))
				break;
		}
		if(i>=200)
				return;
		
		i = 0;
		while(i++<200)
		{
			if(0x00 == sd_cmd(0x41, 0, 0xff))
				break;
		}
		if(i>=200)
				return;
		
		sd_set
		SPI_CTL0(SD_SPI_PORT) |= (u32)SPI_PSC_4;
		
}

#endif
