#include "delay.h"
#include "gd32c10x.h"
#include <stdio.h>
#include "gd32c10x_eval.h"
#include "gd32c10x_spi.h"

#define TICKS 65536

void nvic_config(void);
uint32_t current_totaltick();

uint32_t tim4cycles=0;


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
		uint32_t tick = current_totaltick() + time;
		while(current_totaltick() < tick);
}

void nvic_config(void)
{
		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
		nvic_irq_enable(TIMER4_IRQn, 1, 1);
}

void tim4_config(void)
{
		/* ----------------------------------------------------------------------------
		TIMER4 Configuration: 
		TIMER4 is from APB1 but 120MHz ((APB1 prescale != 1 )
		TIMER4CLK = SystemCoreClock/120 = 1000 000Hz, the period is 0.065536s(TICKS/1000 000 = 0.065536s).
		---------------------------------------------------------------------------- */
		timer_parameter_struct timer_initpara;

		rcu_periph_clock_enable(RCU_TIMER4);

		timer_deinit(TIMER4);
		/* initialize TIMER init parameter struct */
		timer_struct_para_init(&timer_initpara);
		/* TIMER1 configuration */
		timer_initpara.prescaler				 = 119;
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
uint32_t current_totaltick()
{
		return tim4cycles * TICKS + TIMER_CNT(TIMER4);
}
uint32_t current_time01ms()
{
		return (tim4cycles * TICKS/100) + TIMER_CNT(TIMER4)/100;
}
void tim4_it()
{
		uint16_t tick = TIMER_CNT(TIMER4);
		tim4cycles++;
}