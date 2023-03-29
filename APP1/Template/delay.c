#ifndef __DELAY__
#define __DELAY__
#include "delay.h"
#include "gd32c10x.h"
#include "Messages.h"
#include "buzzer.h"
#include "main.h"
#include <stdio.h>
#include "gd32c10x_eval.h"
#include "gd32c10x_spi.h"

#define TICKS 65536U

void nvic_config(void);
uint32_t current_totaltick();

static uint32_t tim4cycles=0;
static u8 tim3cycles=0;
u8 delay_period_ms;
extern u8 buzzer_running;
extern u8 can_traffic_indicator;
extern u8 led2_lumin;
static u8 heartbeat_send_i=0;
static u8 heartbeat_send_c=60;
static u8 heartbeat_recv_i=0;
static u8 heartbeat_recv_c=200;

void delay_ms(uint32_t time)
{
		uint32_t cur;
		uint32_t tick = current_totaltick();
		tick += time * 1000;
		while(1)
		{
				cur=current_totaltick();
				if(cur >= tick)
					break;
		}
}

void delay_us(uint32_t time)
{
		uint32_t cur;
		uint32_t tick = current_totaltick();
		tick += time < 1 ? 1 : (time / 1);
		while(current_totaltick() < tick);
}

void nvic_config(void)
{
		nvic_irq_enable(TIMER4_IRQn, 1, 0);
}

void nvic_config_tim3(void)
{
		nvic_irq_enable(TIMER3_IRQn, 3, 3);
}

void tim4_config(void)
{
		/* ----------------------------------------------------------------------------
		TIMER4 Configuration: 
		TIMER4 is from APB1 but 120MHz ((APB1 prescale != 1 )
		TIMER4CLK = SystemCoreClock/120 = 1000 000Hz, single period is 1us, the cycle period is 0.065536s(TICKS/100 000 = 0.065536s).
		---------------------------------------------------------------------------- */
		timer_parameter_struct timer_initpara;
		u16 prescale = 120;
		delay_period_ms = TICKS / (120000 / prescale);
		
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
		
		nvic_config();
		timer_enable(TIMER4);
}
void tim3_config(void)
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
		timer_initpara.period						= 5 - 1;
		timer_initpara.clockdivision		 = TIMER_CKDIV_DIV1;
		timer_init(TIMER3, &timer_initpara);

		/* enable the TIMER interrupt */
		timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
		timer_interrupt_enable(TIMER3, TIMER_INT_UP);
		
		nvic_config_tim3();
		timer_enable(TIMER3);
}

uint32_t current_totaltick(void)
{
		return tim4cycles * TICKS + TIMER_CNT(TIMER4);
}
uint32_t current_time01ms(void)
{
		return (tim4cycles * TICKS/100) + TIMER_CNT(TIMER4)/100;
}

u8 heartbeat_pending(void)
{
		return heartbeat_send_i >= heartbeat_send_c;
}
void clear_heartbeat_pending(void)
{
		heartbeat_send_i = 0;
}
u8 heartbeat_timeout(void)
{
		return heartbeat_recv_i >= heartbeat_recv_c;
}
void clear_heartbeat_timeout(void)
{
		heartbeat_recv_i = 0;
}
void tim4_it(void)
{
		tim4cycles++;
		if(buzzer_running)
				buzzer_tick();
		if(heartbeat_send_i < heartbeat_send_c)
		{
				heartbeat_send_i++;
		}
		if(heartbeat_recv_i < heartbeat_recv_c)
		{
				heartbeat_recv_i++;
		}
}

void tim3_slow(void)
{
	TIMER_CAR(TIMER3) = 100 - 1;
}

void tim3_normal(void)
{
	TIMER_CAR(TIMER3) = 5 - 1;
}

void tim3_it(void)
{
		tim3cycles++;
		if(led2_lumin <= 0) return;
		if(tim3cycles >= 100 / led2_lumin)
		{
				bit_status bit;
				if(can_traffic_indicator == 0)
						bit = RESET;
				else
				{
						bit = SET - gpio_output_bit_get(GPIOC,GPIO_PIN_14);
						can_traffic_indicator--;
				}
				gpio_bit_write(GPIOC, GPIO_PIN_14, bit);
				tim3cycles = 0;
		}
}

#endif
