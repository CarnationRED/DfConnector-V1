#ifndef __DELAY__
#define __DELAY__
#include "delay.h"
#include "gd32c10x_it.h"
#include "gd32c10x.h"
#include "Messages.h"
#include "buzzer.h"
#include "main.h"
#include <stdio.h>
#include "gd32c10x_spi.h"

#define TICKS 65536U
#define current_totaltick ((u32)tim4cycles * (u32)TICKS + (u32)TIMER_CNT(TIMER4))

void nvic_config(void);

static volatile uint32_t tim4cycles=0;
static volatile u8 tim3cycles=0;
extern u8 can_traffic_indicator;
extern bit_status led2_state;
extern bit_status systick_state;
extern u16 systick_cycles;
extern u8 connected;
static volatile u8 heartbeat_send_i=0;
static volatile u8 heartbeat_send_c=2;
static volatile u8 heartbeat_recv_i=0;
static volatile u8 heartbeat_recv_c=8;

void delay_ms(uint16_t time)
{
		u16 cnt_100ms = time / 100;
		u16 remain = time - cnt_100ms * 100;
		
		if(cnt_100ms > 0)
		{
				SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
				SysTick->LOAD  = 12000000 - 1;                                  /* set reload register */
				NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
				SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */                
				systick_cycles = 0;
				SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
										 SysTick_CTRL_TICKINT_Msk   |
										 SysTick_CTRL_ENABLE_Msk; 
				while(systick_cycles < cnt_100ms);
				SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk; 
		}
		if(remain > 0)
		{
				SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;          
				SysTick->LOAD  = 120000 * remain - 1;                                  /* set reload register */
				NVIC_SetPriority (SysTick_IRQn, 0);  /* set Priority for Systick Interrupt */
				SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */                
				systick_cycles = 0;
				SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
										 SysTick_CTRL_TICKINT_Msk   |
										 SysTick_CTRL_ENABLE_Msk; 
				while(systick_cycles == 0);
		}
		SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;          
}

void delay_us(u16 time)
{
    SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
		SysTick->LOAD  = 120 * time - 1;                                  /* set reload register */
		NVIC_SetPriority (SysTick_IRQn, 0);  /* set Priority for Systick Interrupt */
		SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */                
		systick_state = RESET;
		SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
										 SysTick_CTRL_TICKINT_Msk   |
										 SysTick_CTRL_ENABLE_Msk;    
		while(systick_state == RESET);
		SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;    
}

/*!
    \brief      Timing, period = 6.5536s
    \retval     none
*/
void timing_config(void)
{
		/* ----------------------------------------------------------------------------
		TIMER4 Configuration: 
		TIMER4 is from APB1 but 120MHz ((APB1 prescale != 1 )
		TIMER4CLK = SystemCoreClock/12000 = 10 000Hz, single period is 100us, the cycle period is 6.5536s(TICKS/10 000 = 6.5536s).
		---------------------------------------------------------------------------- */
		timer_parameter_struct timer_initpara;
		u16 prescale = 12000;
		
		rcu_periph_clock_enable(RCU_TIMER4);

		timer_deinit(TIMER4);
		/* initialize TIMER init parameter struct */
		timer_struct_para_init(&timer_initpara);
		/* TIMER1 configuration */
		timer_initpara.prescaler				 = prescale - 1;
		timer_initpara.alignedmode			 = TIMER_COUNTER_EDGE;
		timer_initpara.counterdirection	= TIMER_COUNTER_UP;
		timer_initpara.period						= TICKS - 1;
		timer_initpara.clockdivision		 = TIMER_CKDIV_DIV1;
		timer_init(TIMER4, &timer_initpara);

		/* enable the TIMER interrupt */
		timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_UP);
		timer_interrupt_enable(TIMER4, TIMER_INT_UP);
		
		nvic_irq_enable(TIMER4_IRQn, IT_PRI_PRE_TIME, IT_PRI_SUB_TIME);
		timer_enable(TIMER4);
}
/*!
    \brief      LED Flash, 2Hz, period = 0.5s
    \retval     none
*/
void led_flash_config(void)
{
		/* ----------------------------------------------------------------------------
		TIMER3 Configuration: 
		TIMER3 is from APB1 but 120MHz ((APB1 prescale != 1 )
		TIMER3CLK = SystemCoreClock/48000 = 2500Hz, 2500 / period = 500Hz, PWM = 500 / 2 = 250Hz
		---------------------------------------------------------------------------- */
		timer_parameter_struct timer_initpara;
		u16 prescale = 48000;
		
		rcu_periph_clock_enable(RCU_TIMER3);

		timer_deinit(TIMER3);
		/* initialize TIMER init parameter struct */
		timer_struct_para_init(&timer_initpara);
		/* TIMER1 configuration */
		timer_initpara.prescaler				 = prescale - 1;
		timer_initpara.alignedmode			 = TIMER_COUNTER_EDGE;
		timer_initpara.counterdirection	= TIMER_COUNTER_UP;
		timer_initpara.period						= 1250 - 1;
		timer_initpara.clockdivision		 = TIMER_CKDIV_DIV1;
		timer_init(TIMER3, &timer_initpara);

		/* enable the TIMER interrupt */
		timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
		timer_interrupt_enable(TIMER3, TIMER_INT_UP);
		
		nvic_irq_enable(TIMER3_IRQn, IT_PRI_PRE_LED, IT_PRI_SUB_LED);
		timer_enable(TIMER3);
}



uint32_t current_time01ms(void)
{
		u32 t = TIMER_CNT(TIMER4) + tim4cycles * TICKS;
		return t;
}

u8 heartbeat_pending(void)
{
		return heartbeat_send_i >= heartbeat_send_c;
}
void clear_heartbeat_pending(void)
{
		heartbeat_send_i = 0;
}
//heartbeat:3000ms
u8 heartbeat_timeout(void)
{
		return heartbeat_recv_i >= heartbeat_recv_c;
}
//heartbeat:3000ms
void clear_heartbeat_timeout(void)
{
		heartbeat_recv_i = 0;
}
/*!
    \brief      delay
    \retval     none
*/
void tim4_it(void)
{
		timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_UP);
		tim4cycles++;
}

/*!
    \brief      LED Flash, 2Hz, period = 0.5s
    \retval     none
*/
void tim3_it(void)
{
		timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
		tim3cycles++;
	
		if(can_traffic_indicator > 0)
		{
				if(--can_traffic_indicator == 0)
						gpio_bit_write(GPIOC, GPIO_PIN_14, led2_state = RESET);
				else
						gpio_bit_write(GPIOC, GPIO_PIN_14, led2_state = SET - led2_state);
				can_traffic_indicator=1;
		}
		if(heartbeat_send_i < heartbeat_send_c)
		{
				heartbeat_send_i++;
		}
		if(heartbeat_recv_i < heartbeat_recv_c)
		{
				heartbeat_recv_i++;
		}
		if(!connected && gpio_output_bit_get(GPIOC,GPIO_PIN_14))
		{
				gpio_bit_write(GPIOC, GPIO_PIN_14, 0);
		}
}

#endif
