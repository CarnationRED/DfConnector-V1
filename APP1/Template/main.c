#ifndef __MAIN__
#define __MAIN__
#include "delay.h"
#include "main.h"
#include "buzzer.h"
#include "gd32c10x_it.h"
#include "Messages.h"
#include "VCI.h"
#include "gd32c10x_gpio.h"

#define CH444G_PORT				GPIOB
#define CH444G_SW0				GPIO_PIN_4
#define CH444G_SW1				GPIO_PIN_3

#define SIGNAL1_RCU				RCU_GPIOC
#define SIGNAL1_PORT			GPIOC
#define SIGNAL1_PIN				GPIO_PIN_13
#define SIGNAL2_PORT			GPIOC
#define SIGNAL2_PIN				GPIO_PIN_14

volatile u8 can0_id_filter_count;
volatile u8 can1_id_filter_count;
volatile u8 current_channel;
u8 can_setting_fd_flag[CAN_NUMBERS];
u8 can_setting_auto_retrans[CAN_NUMBERS];
u8 can_setting_fd_iso_bosch[CAN_NUMBERS];
u32 can_setting_buadrate[CAN_NUMBERS];
u32 can_setting_buadrate_fd[CAN_NUMBERS];

volatile u8 led2_lumin = 20;

volatile static VCI_STATUS vci_status;

static u8 beep_startup1[]={22,0,22};
static u8 beep_startup2[]={15,0,22,0,26};
static u8 beep_connected[]={8,17,21};
static u8 beep_disconnected[]={21,17,8};

volatile static VCI_CTL_HEARTBEAT_STRUCT heartbeat;

extern WIFI_STATUS wifi_status;
extern MESSAGING_STATUS messaging_status;
extern u8 can_msg_recv_start;
extern u8 can_msg_recv_count;
extern u8 can_cmd_recv_start;
extern u8 can_cmd_recv_count;

void set_vci_status(VCI_STATUS s)
{
	vci_status = s;
}
VCI_STATUS get_vci_status(void)
{
	return vci_status;
}
void err_led()
{
		u8 i = 0;
		while(1)
		{
				delay_us(1000000);
				gpio_bit_write(GPIOC, GPIO_PIN_13, i);
				if(i==0)i=1;
				else i=0;
		}
}

void params_init(void)
{
		can0_id_filter_count = 0;
		can1_id_filter_count = 14;
		current_channel = 0;//2/10
		can_setting_fd_flag[0] = can_setting_fd_flag[1] = 0;
		can_setting_auto_retrans[0] = can_setting_auto_retrans[1] = 1;
		can_setting_fd_iso_bosch[0] = can_setting_fd_iso_bosch[1] = 0;
		can_setting_buadrate[0] = can_setting_buadrate[1] = 500000;
		can_setting_buadrate_fd[0] = can_setting_buadrate_fd[1] = 500000;
	
		can_msg_buffer_clear();
	
		vci_status = VCI_STATUS_RESET;
				
		wifi_status = WIFI_STATUS_INIT_PENDING;
}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
	    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);
//    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
		
    rcu_periph_clock_enable(RCU_AF);
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOB,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    
    /* configure CAN1 GPIO */
//    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
//    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    
    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP /*GPIO_CAN0_FULL_REMAP*/, ENABLE);
    gpio_pin_remap_config(GPIO_CAN1_REMAP, DISABLE);
		//CH444G
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_3);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_4);
		gpio_bit_set(CH444G_PORT, CH444G_SW0);
		gpio_bit_set(CH444G_PORT, CH444G_SW1);
}

/*!
    \brief      configure BSP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_board_config(void)
{
		rcu_periph_clock_enable(RCU_AF);
		gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP,ENABLE);
	
    rcu_periph_clock_enable(SIGNAL1_RCU);
    gpio_init(SIGNAL1_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,SIGNAL1_PIN);//SIGNAL1 LED
    gpio_init(SIGNAL2_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,SIGNAL2_PIN);//SIGNAL2 LED
}

void can_set_id_filter_standard(u8 filter_no, u16 id1, u16 id2, u16 id3, u16 id4, u32 can_number, ControlStatus enable)
{
    can_filter_parameter_struct  canf;
		if(can_number == 0 || can_number == CAN0)
		{
				if(filter_no >= 14)
						return;
		}
		else if(filter_no < 14)
				return;
		
		can_struct_para_init(CAN_FILTER_STRUCT, &canf);
		canf.filter_number = filter_no;
		
		canf.filter_mode = CAN_FILTERMODE_LIST;
		canf.filter_bits = CAN_FILTERBITS_32BIT;
		
		canf.filter_enable = enable;
		canf.filter_fifo_number = CAN_FIFO0;
		canf.filter_list_high = (u16)(id1<<5);
		canf.filter_list_low = (u16)(id2<<5);
		canf.filter_mask_high = (u16)(id3<<5);
		canf.filter_mask_low = (u16)(id4<<5);
		can_filter_init(&canf);
}

void can_set_id_filter_extended(u8 filter_no, u32 id1, u32 id2, u32 can_number, ControlStatus enable)
{
    can_filter_parameter_struct  canf;
		if(can_number == 0 || can_number == CAN0)
		{
				if(filter_no >= 14)
						return;
		}
		else if(filter_no < 14)
				return;
		
		can_struct_para_init(CAN_FILTER_STRUCT, &canf);
		canf.filter_number = filter_no;
		
		canf.filter_mode = CAN_FILTERMODE_LIST;
		canf.filter_bits = CAN_FILTERBITS_32BIT;
		
		canf.filter_enable = enable;
		canf.filter_fifo_number = CAN_FIFO0;
		canf.filter_list_high = (u16)(id1>>13);
		canf.filter_list_low = (u16)((id1<<3)|4);
		canf.filter_mask_high = (u16)(id2>>13);
		canf.filter_mask_low = (u16)((id2<<3)|4);
		can_filter_init(&canf);
}

void can_clear_filter(u32 can_number)
{
		u8 n = can_number == CAN0 ? 0 : 14;
		u8 i;
		if(can_number == 0 || can_number == CAN0)
				can0_id_filter_count = 0;
		else
				can1_id_filter_count = 14;
		
		can_set_id_filter_extended(n,0,0,can_number,DISABLE);
		can_set_id_filter_standard(++n,0,0,0,0,can_number,DISABLE);
		for(i = 0;i < 13;i++)
		{
				can_set_id_filter_extended(n + i,0,0,can_number,DISABLE);
		}
		if(can_number == 0 || can_number == CAN0)
				can0_id_filter_count = 0;
		else
				can1_id_filter_count = 14;
}

void vci_can_stop(void)
{
    can_deinit(CAN0);
    can_deinit(CAN1);
    nvic_irq_disable(CAN0_RX0_IRQn);
    nvic_irq_disable(CAN1_RX0_IRQn);
}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct cp;
    can_fdframe_struct fp; 
    can_fd_tdc_struct tdc;
		//定义CAN滤波器结构体
		can_filter_parameter_struct can_filter;
    can_struct_para_init(CAN_INIT_STRUCT, &cp);
    /* initialize CAN register */
    can_deinit(CAN0);
    can_deinit(CAN1);
    
    /* initialize CAN parameters */
    cp.time_triggered = DISABLE;
    cp.auto_bus_off_recovery = ENABLE;
    cp.auto_wake_up = DISABLE;
    cp.auto_retrans = can_setting_auto_retrans[0];
    cp.rec_fifo_overwrite = ENABLE;
    cp.trans_fifo_order = ENABLE;
    cp.working_mode = CAN_NORMAL_MODE;  
    /* initialize CAN */
    can_init(CAN0, &cp);
    cp.auto_retrans = can_setting_auto_retrans[1];
    can_init(CAN1, &cp);
   
    /* config CAN0 baud rate */
		if(ERROR == can_frequency_set(CAN0, can_setting_buadrate[0]))
			 err_led();
    /* config CAN1 baud rate */
    if(ERROR == can_frequency_set(CAN1, can_setting_buadrate[1]))
			 err_led();
    
    can_struct_para_init(CAN_FD_FRAME_STRUCT, &fp);
    fp.fd_frame = can_setting_fd_flag[0];
    fp.excp_event_detect = ENABLE;//protocol exception event detection function
    fp.delay_compensation = ENABLE;//transmitter delay compensation
    tdc.tdc_filter = 0x04; 
    tdc.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;//measurement and offset
    tdc.tdc_offset = 0x04;
    fp.p_delay_compensation = &tdc;
    fp.iso_bosch = can_setting_fd_iso_bosch[0] == 0 ? CAN_FDMOD_ISO : CAN_FDMOD_BOSCH;//ISO mode
    fp.esi_mode = CAN_ESIMOD_HARDWARE;//error state indicator mode = displays the node error state by hardware
    if(can_setting_fd_flag[0])
				can_fd_init(CAN0, &fp);
    fp.fd_frame = can_setting_fd_flag[1];
    fp.iso_bosch = can_setting_fd_iso_bosch[1] == 0 ? CAN_FDMOD_ISO : CAN_FDMOD_BOSCH;//ISO mode
    if(can_setting_fd_flag[1])
				can_fd_init(CAN1, &fp);
    
    if(can_setting_fd_flag[0])
				can_fd_frequency_set(CAN0, can_setting_buadrate_fd[0]);	//canfd 1M 2M 3M 4M 5M 6M
    if(can_setting_fd_flag[1])
				can_fd_frequency_set(CAN1, can_setting_buadrate_fd[1]);	//canfd 1M 2M 3M 4M 5M 6M
    
    /* initialize filter */ 
    can1_filter_start_bank(14);
		
		can_clear_filter(CAN0);
		can_clear_filter(CAN1);
    
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 1);
    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX0_IRQn, 0, 2);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
    can_interrupt_enable(CAN1, CAN_INTEN_RFNEIE0);
}

/*!
    \brief      switch can channel
    \param[in]  none
    \param[out] none
    \retval     CAN_PERIH
*/
u32 set_can_channel(u8 channel)
{
		//SW0		SW1		CAN			CHANNEL	CAN_PERIH
		//X			X			6/14		0				CAN0
		//0			0			1/9			1				CAN1
		//1			0			11/12		2				CAN1
		//0			1			3/8			3				CAN1
		//1			1			2/10		4				CAN1
		u8 b = channel != current_channel;
		current_channel = channel;
		switch(channel)
		{
			case 1:
				if(b)
				{
						gpio_bit_reset(CH444G_PORT, CH444G_SW0);
						gpio_bit_reset(CH444G_PORT, CH444G_SW1);
				}
				return CAN1;
			case 2:
				if(b)
				{
						gpio_bit_set(CH444G_PORT, CH444G_SW0);
						gpio_bit_reset(CH444G_PORT, CH444G_SW1);
				}
				return CAN1;
			case 3:
				if(b)
				{
						gpio_bit_reset(CH444G_PORT, CH444G_SW0);
						gpio_bit_set(CH444G_PORT, CH444G_SW1);
				}
				return CAN1;
			case 4:
				if(b)
				{
						gpio_bit_set(CH444G_PORT, CH444G_SW0);
						gpio_bit_set(CH444G_PORT, CH444G_SW1);
				}
				return CAN1;
			default:
				return CAN0;
		}
}

void vci_can_init(void)
{
	/* configure CAN GPIO, including CH444G multiplexer */
		can_gpio_config();
    /* initialize CAN and filter */
		can_config();
		set_can_channel(1);
}

void buzz_config(void)
{
		timer_parameter_struct timer_initpara;
		timer_oc_parameter_struct timer_ocinitpara;
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);//BUZZ PWM
		
		/* ----------------------------------------------------------------------------
		TIMER0 Configuration: 
		TIMER0 is from APB1, 120MHz
		TIMER0CLK = SystemCoreClock/1200 = 100 000Hz, the pwm is 100k
		Set period:
			TIMER_CAR(TIMER0) = (u32)(period-1);
		Set pulse:
			TIMER_CH0CV(TIMER0) = (u32)pulse;
		---------------------------------------------------------------------------- */

		rcu_periph_clock_enable(_BUZZER_TIMER_RCU);

		timer_deinit(_BUZZER_TIMER);
		/* initialize TIMER init parameter struct */
		timer_struct_para_init(&timer_initpara);
		/* TIMER1 configuration */
		timer_initpara.prescaler				 = _BUZZER_PSC - 1;
		timer_initpara.alignedmode			 = TIMER_COUNTER_EDGE;
		timer_initpara.counterdirection	= TIMER_COUNTER_UP;
		timer_initpara.period						= 10 - 1;
		timer_initpara.clockdivision		 = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
		timer_init(TIMER0, &timer_initpara);


    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//channel enable
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//output: high
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//idle: low
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(_BUZZER_TIMER, _BUZZER_TIMER_CH, &timer_ocinitpara);

    /* CH0 configuration in PWM mode0, duty cycle 25% */
    timer_channel_output_pulse_value_config(_BUZZER_TIMER, _BUZZER_TIMER_CH, 5);
    timer_channel_output_mode_config(_BUZZER_TIMER, _BUZZER_TIMER_CH, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(_BUZZER_TIMER, _BUZZER_TIMER_CH, TIMER_OC_SHADOW_DISABLE);
		
    timer_primary_output_config(_BUZZER_TIMER, DISABLE);
    timer_auto_reload_shadow_disable(_BUZZER_TIMER);
		timer_enable(TIMER0);
}


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{ 
		u8 rom1 = ((*(u32*)BT_JUMP_ADDR) == APP_ROM1_ADDR);
	
		__enable_irq();
		while(1)
		{
				can_trasnmit_message_struct m;
				params_init();
			
				/* configure board, led */
				bsp_board_config();
			
				nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
			
				/* start timer */
				tim4_config();
				tim3_config();
				
				buzz_config();
			
				beep_setvolume(85);
				if(rom1)
					beep(beep_startup1, 1, 520);
				else 
					beep(beep_startup2, 1, 520);
			
				gpio_bit_set(GPIOC,GPIO_PIN_13);
				gpio_bit_set(GPIOC,GPIO_PIN_14);
				
				vci_can_init();
	
				can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &m);
				m.tx_sfid = 0x001;
				m.tx_efid = 0x002;
				m.tx_ft = CAN_FT_DATA;
				m.tx_ff = CAN_FF_STANDARD;
				m.tx_dlen = 8;
				m.fd_flag = 0;
				m.fd_brs = 0;
				m.fd_esi = 0;
			  m.tx_efid = 0x111;
				m.tx_data[0] = 0xaa;
				m.tx_data[1] = 0xbb;
				m.tx_data[2] = 0xcc;
				m.tx_data[3] = 0xdd;
				m.tx_data[4] = 0xee;
				m.tx_data[5] = 0xff;
				m.tx_data[6] = 0x00;
				m.tx_data[7] = 0x11;
				can_message_transmit(CAN0, &m);
			
			
				can1_filter_start_bank(14);
				can_set_id_filter_standard(0,  0, 0,0, 0, CAN0, ENABLE);
				can_set_id_filter_standard(1,  0, 0,0, 0, CAN0, ENABLE);
				can_set_id_filter_standard(0, 0, 0,0, 0, CAN1, ENABLE);
				can_set_id_filter_standard(15, 0, 0,0, 0, CAN1, ENABLE);
//			while(1){
//					if(get_vci_status()==VCI_STATUS_DFU)
//					{
//						can_message_transmit(CAN0, &m);
//						set_vci_status(VCI_STATUS_RUN);
//					}
//			}
				/* start wifi, time consuming */
				wifi_Init();
				delay_ms(100);
				/* start server port 1112 for can message uploading and can command downloading */
				wifi_SetupTCPServer(WIFI_CAN_MSG_PORT,WIFI_CAN_MSG_CHNL);
				/* start server port 1113 for can device control, reinit, filter setting, and so on */
				wifi_SetupTCPServer(WIFI_CAN_CTL_PORT,WIFI_CAN_CTL_CHNL);
				/* start server port 1114 for vci device control, including status report, time sync */
				wifi_SetupTCPServer(WIFI_VCI_CTL_PORT,WIFI_VCI_CTL_CHNL);
				
				if(wifi_status == WIFI_STATUS_INIT_COMPLET)
				{
						if(rom1)
							beep(beep_startup1, sizeof(beep_startup1), 520);
						else 
							beep(beep_startup2, sizeof(beep_startup2), 520);
				}
				
				set_vci_status(VCI_STATUS_RUN);
				
				while(get_vci_status() == VCI_STATUS_RUN)
				{
						u8 counter = 0;
						wifi_status = WIFI_STATUS_WAIT_FOR_CNT;
						while(wifi_GetClients(WIFI_CAN_CTL_CHNL) <= 0 && wifi_GetClients(WIFI_VCI_CTL_CHNL) <= 0 && get_vci_status() == VCI_STATUS_RUN) delay_ms(100);
						
						buzzer_wait_until_idle();
						beep(beep_connected, sizeof(beep_connected), 600);
					
						messaging_status = MSG_STATUS_NORMAL;
						wifi_status = WIFI_STATUS_CONCTED_IDLE;
						set_vci_status(VCI_STATUS_RUN);
						while(get_vci_status() == VCI_STATUS_RUN)
						{
								if(counter++ == 0xfe) {
									EXTI4_IRQHandler();
									if(heartbeat_timeout())
									{
											clear_heartbeat_timeout();
											break;
									}
									if(heartbeat_pending())
									{
											clear_heartbeat_pending();
											heartbeat.msg_stat = messaging_status;
											heartbeat.vci_stat = vci_status;
											heartbeat.wifi_stat = wifi_status;
											wifi_SendData((u8*)&heartbeat, sizeof(VCI_CTL_HEARTBEAT_STRUCT), WIFI_VCI_CTL_CHNL);
									}
									counter=0;
								}
								
							// WIFI SEND
								can_msg_send2wifi();
								
								if(wifi_status != WIFI_STATUS_CONCTED_IDLE)
									break;
							//CAN SEND
								can_cmd_send2can();
						}
						while(get_vci_status() == VCI_STATUS_DFU);
						buzzer_wait_until_idle();
						beep(beep_disconnected, 3, 600);
				}
				wifi_reset();
		}
}

#endif
