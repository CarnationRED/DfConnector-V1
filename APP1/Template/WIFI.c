#include "WIFI.h"
#include "delay.h"
#include "M8266WIFIDrv.h"

 WIFI_OPMODE wifi_opmode = WIFI_OPMODE_AP;
 volatile char wifi_ssid[]="Dfcr-00000000";

u8 wifi_send_buffer[WIFI_SEND_BUFFER];
u8 wifi_recv_buffer[1024];

WIFI_STATUS wifi_status;

void wifi_Module_Hardware_Reset(void);

// Initialize the SPI peripheral
void wifi_spi_init(void)
{
		spi_parameter_struct spi_init_struct;
		// Enable the clock for GPIOA and SPI0
		rcu_periph_clock_enable(RCU_SPI0);
		rcu_periph_clock_enable(RCU_GPIOA);
		rcu_periph_clock_enable(RCU_GPIOC);
		rcu_periph_clock_enable(RCU_AF);

		// Configure the GPIO pins for SPI function
		gpio_init(WIFI_PIN_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, WIFI_SCK_PIN | WIFI_MOSI_PIN);
		gpio_init(WIFI_PIN_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, WIFI_MISO_PIN);
		gpio_init(WIFI_NCS_RST_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, WIFI_NCS_PIN | WIFI_RST_PIN);
	
		gpio_bit_set(WIFI_NCS_RST_PORT,WIFI_NCS_PIN);
		gpio_bit_reset(WIFI_NCS_RST_PORT,WIFI_RST_PIN);
		
		// Reset the SPI0 peripheral
		spi_i2s_deinit(WIFI_SPI_PORT);

		// Configure the parameters for SPI0
		spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX; // Full duplex mode
		spi_init_struct.device_mode = SPI_MASTER; // Master or slave mode
		spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT; // 8-bit data frame size
		spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE; // Clock polarity and phase mode 0
		spi_init_struct.nss = SPI_NSS_SOFT; // Soft NSS signal
		spi_init_struct.prescale = SPI_PSC_8; // Prescale factor to get desired speed 
		spi_init_struct.endian = SPI_ENDIAN_MSB; // MSB first
	 
	 // Initialize the SPI0 peripheral with the parameters 
	 spi_init(WIFI_SPI_PORT, &spi_init_struct);

	 // Enable the CRC calculation if needed 
	 /*spi_crc_polynomial_set(WIFI_SPI_PORT,7);
	 spi_crc_on(WIFI_SPI_PORT);*/

	 // Enable the DMA if needed 
	 /*spi_dma_enable(WIFI_SPI_PORT,SPI_DMA_TRANSMIT);
	 spi_dma_enable(WIFI_SPI_PORT,SPI_DMA_RECEIVE);*/

	 // Enable the interrupt if needed 
	 /*eclic_irq_enable(SPI0_IRQn,ECLIC_PRIGROUP_LEVEL3_PRIO1,ECLIC_SUBGROUP_LEVEL3);
	 eclic_irq_enable(DMA_Channel1_IRQn,ECLIC_PRIGROUP_LEVEL3_PRIO2,ECLIC_SUBGROUP_LEVEL3);
	 eclic_irq_enable(DMA_Channel2_IRQn,ECLIC_PRIGROUP_LEVEL3_PRIO2,ECLIC_SUBGROUP_LEVEL3);*/

	 // Enable the SPI0 peripheral 
	 spi_enable(WIFI_SPI_PORT);
}

void wifi_init_it()
{
		nvic_irq_disable(EXTI4_IRQn);
		rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
		gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_4);
		nvic_irq_enable(EXTI4_IRQn, 1, 1);
		exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_RISING);
		exti_interrupt_flag_clear(EXTI_4);
}
void wifi_disableIT(void)
{
		nvic_irq_disable(EXTI4_IRQn);
}
void wifi_enableIT(void)
{
	nvic_irq_enable(EXTI4_IRQn, 1, 1);
}

void wifi_reset(void)
{
		gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_RST_PIN, RESET);
		delay_ms(500);
}
static char IntToHex(unsigned x) {
  x &= 15;
  if (x <= 9) return x + '0';
  return x - 10 + 'A';
  }
void wifi_Init(void)
{
	  u16 retry=3;
		wifi_status = WIFI_STATUS_INIT_PENDING;
		// Initialize the SPI peripheral
		wifi_spi_init();
		while(retry-->0){
			u16 status;
			wifi_Module_Hardware_Reset();
			M8266HostIf_Set_SPI_nCS_Pin(RESET); 
			delay_ms(5);	
			
			wifi_init_it();
			
			M8266HostIf_SPI_ReadWriteByte(0x04);     //总线上 同时会返回0xFF，这个不管
			if(0x41!= M8266HostIf_SPI_ReadWriteByte(0x00))
			{
					// 此时总线上，返回的字节必须是0x41，说明spi初始化和接线都对了
					wifi_status = WIFI_STATUS_ERR_SPI_INIT;
					return;
			}
			
			if (M8266HostIf_SPI_Select(WIFI_SPI_PORT, 15000000, &status) == 0) //设置模块的SPI连接信息
			{
					wifi_status = WIFI_STATUS_ERR_SPI_SELC;
					continue;
			}
			else
			{
				wifi_status = WIFI_STATUS_SPI_COMPLETE;	
				break;
			}
		}
		if(wifi_status == WIFI_STATUS_SPI_COMPLETE)
		{
//				volatile u32 i, j;
//        u8 byte;
//        if (M8266WIFI_SPI_Interface_Communication_OK(&byte) == 0) //	if SPI logical Communication failed
//        {
//					wifi_status = WIFI_STATUS_ERR_STRESERR;
//        }

//        i = 100000;
//        j = M8266WIFI_SPI_Interface_Communication_Stress_Test(i);
//        if ((j < i) && (i - j > 5)) //	if SPI Communication stress test failed (Chinese: SPI底层通信压力测试失败，表明你的主机板或接线支持不了当前这么高的SPI频率设置)
//        {
//					wifi_status = WIFI_STATUS_ERR_STRESERR;
//        }
				u16 status;
				u8 get_opMode;
				if (M8266WIFI_SPI_Set_Opmode(wifi_opmode, 0, &status) == 0) // 模块工作模式 // 1=STA WIFI终端, 2=AP WIFI路由, 3=STA+AP 混合模式
				{
						wifi_status = WIFI_STATUS_ERR_SETOPMOD;
				}
				
				if (M8266WIFI_SPI_Get_Opmode(&get_opMode, &status) == 0 || get_opMode != wifi_opmode) {
						wifi_status = WIFI_STATUS_ERR_SETOPMOD;
				}
				else wifi_status = WIFI_STATUS_OPMODE_COMPL;
		}
		if(wifi_status == WIFI_STATUS_OPMODE_COMPL)
		{
				u16 status;
				u32 cpuid2 = *(u32*)(0x1FFFF7E8) + *(u32*)(0x1FFFF7EC) + *(u32*)(0x1FFFF7F0);
				u8 i;
				
				for(i =5;i<13;i++)
				{
					wifi_ssid[i]=IntToHex(cpuid2>>(28-(i-5)*4));
				}
			
				if (M8266WIFI_SPI_Config_AP((u8*)wifi_ssid, (u8*)"11111111", 4, 6, 0, &status) == 0) {
				if (M8266WIFI_SPI_Config_AP("Anylinkin", "1234567890", 4, 1, 0, &status) == 0)
						wifi_status = WIFI_STATUS_ERR_CONFIGAP;
				}
//				else if (M8266WIFI_SPI_OptSel_Local_Ap_Channel(NULL, 0, &status) == 0) {
//						wifi_status = WIFI_STATUS_ERR_CONFIGAP;
//				}
				else wifi_status= WIFI_STATUS_INIT_COMPLET;
		}
}

//硬件重设
void wifi_Module_Hardware_Reset()
{
		gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_NCS_PIN, RESET); 								 // Module nCS==ESP8266 GPIO15 as well, Low during reset in order for a normal reset (Chinese: 为了实现正常复位，模块的片鿉信号nCS在复位期间需要保持拉使)
		delay_ms(1);																												 // delay 1ms, adequate for nCS stable (Chinese: 延迟1毫秒，确保片选nCS设置后有足够的时间来稳定)

		gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_RST_PIN, RESET); // Pull low the nReset Pin to bring the module into reset state (Chinese: 拉低nReset管脚让模组进入复位状怿)
		delay_ms(5);																															 // delay 5ms, adequate for nRESET stable(Chinese: 延迟5毫秒，确保片选nRESER设置后有足够的时间来稳定，也确保nCS和nRESET有足够的时间同时处于低电平状怿)
																																							 // give more time especially for some board not good enough
																																							 //(Chinese: 如果主板不是很好，导致上升下降过渡时间较长，或迅因为失配存在较长的振荡时间，所以信号到轨稳定的时间较长，那么在这里可以多给丿些延旿)

		gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_RST_PIN, SET); // Pull high again the nReset Pin to bring the module exiting reset state (Chinese: 拉高nReset管脚让模组鿿出复位状怿)
		delay_ms(300);																													 // at least 18ms required for reset-out-boot sampling boottrap pin (Chinese: 至少霿覿18ms的延时来确保逿出复位时足够的boottrap管脚采样时间)
																																						 // Here, we use 300ms for adequate abundance, since some board GPIO, (Chinese: 在这里我们使用了300ms的延时来确保足够的富裕量，这是因为在某些主板上，)
																																						 // needs more time for stable(especially for nRESET) (Chinese: 他们的GPIO可能霿要较多的时间来输出稳定，特别是对于nRESET承对应的GPIO输出)
																																						 // You may shorten the time or give more time here according your board v.s. effiency
																																						 // (Chinese: 如果你的主机板在这里足够好，你可以缩短这里的延时来缩短复位周期；反之则需要加长这里的延时〿
																																						 //					 总之，你可以调整这里的时间在你们的主机板上充分测试，找到丿个合适的延时，确保每次复位都能成功㿂并适当保持丿些富裕量，来兼容批量化时主板的个体濧差弿)
		gpio_bit_write(WIFI_NCS_RST_PORT, WIFI_NCS_PIN, SET);			 // release/pull-high(defualt) nCS upon reset completed (Chinese: 释放/拉高(缺省)片鿉信叿
		// delay_ms(1); 					// delay 1ms, adequate for nCS stable (Chinese: 延迟1毫秒，确保片选nCS设置后有足够的时间来稳定)

		delay_ms(400); // Delay more than around 500ms for M8266WIFI module bootup and initialization，including bootup information print。No influence to host interface communication. Could be shorten upon necessary. But test for verification required if adjusted.
									 // (Chinese: 延迟大约500毫秒，来等待模组成功复位后完成自己的启动过程和自身初始化，包括串口信息打印㿂但是此时不影响模组和单片主机之间的通信，这里的时间可以根据霿要鿂当调整.如果调整缩短了这里的时间，建议充分测试，以确保系绿(时序关系上的)可靠怿)
}


void wifi_SetupTCPServer(u16 port,u8 link_no)
{
    u16 status;
    if (M8266WIFI_SPI_Config_Tcp_Window_num(link_no, 4, &status) == 0)
        wifi_status= WIFI_STATUS_ERR_SETCPSVR;
    else if (M8266WIFI_SPI_Setup_Connection(WIFI_WRKMODE_TCPServer, port, "1.1.1.1", 0, link_no, 13, &status) == 0) {
        wifi_status= WIFI_STATUS_ERR_SETCPSVR;
    }
}

u8 wifi_GetClients(u8 link_no)
{
    u8 count = 1;
		static u8 type = 1;
//		M8266WIFI_SPI_List_Clients_On_A_TCP_Server(link_no, &count, NULL, NULL);//这个方法特别不稳定，明明连接了，返回0
//		return count;
		M8266WIFI_SPI_Query_Connection(link_no, &type, &count, NULL,NULL,NULL,NULL);//这个方法也不稳定，明明连接了，返回0。但是好一些
		return count >= 3 && count <= 5;
}

typedef struct {
    u8 bssid[6];
    u8 ipaddr[4];
} CONNECTED_STATION_INFO;
u8 wifi_GetSTACount()
{
    u8 mode;
    u8 count;
    M8266WIFI_SPI_Get_Opmode(&mode, NULL);
    if (mode != WIFI_OPMODE_AP) return 0;
    M8266WIFI_SPI_AP_List_STAs_Info(NULL, &count, 4, NULL);
    return count;
}


/// @brief 将数据传输到WIFI模块，然后返回。模块将自行在空闲时发送
/// @param data
/// @param dataLen
u16 wifi_SendData(uint8_t data[], uint16_t dataLen,u8 link_no)
{
    u16 status;
		u16 size;
    while (1)
        if (dataLen > 1460 || (size = M8266WIFI_SPI_Send_Data(data, dataLen, link_no, &status)) == 0) {
            if((status & 0x12) == 0x12) continue;
            if((status & 0x18) == 0x18) 
								wifi_status = WIFI_STATUS_ERR_CONTLOST;
            wifi_status = WIFI_STATUS_ERR_SENDDATA;
						break;
        } else
            break;
		return size;
}
