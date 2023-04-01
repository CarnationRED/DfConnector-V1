#include "delay.h"
#include "buzzer.h"

volatile u8 buzzer_running = 0;
volatile u8 buzzer_duty = 100;
volatile u8 music[256];
volatile u8 total_note, curr_note;
volatile u16 bpm, beatgap;
static u16 _tone_period[29]
	= {
			0,
			_BUZZER_PWM/(float)_FREQ_1_DO_L,//1
			_BUZZER_PWM/(float)_FREQ_2_RE_L,//2
			_BUZZER_PWM/(float)_FREQ_3_MI_L,//3
			_BUZZER_PWM/(float)_FREQ_4_FA_L,//4
			_BUZZER_PWM/(float)_FREQ_5_SO_L,//5
			_BUZZER_PWM/(float)_FREQ_6_LA_L,//6
			_BUZZER_PWM/(float)_FREQ_7_SI_L,//7
			_BUZZER_PWM/(float)_FREQ_1_DO_M,//8
			_BUZZER_PWM/(float)_FREQ_2_RE_M,//9
			_BUZZER_PWM/(float)_FREQ_3_MI_M,//10
			_BUZZER_PWM/(float)_FREQ_4_FA_M,//11
			_BUZZER_PWM/(float)_FREQ_5_SO_M,//12
			_BUZZER_PWM/(float)_FREQ_6_LA_M,//13
			_BUZZER_PWM/(float)_FREQ_7_SI_M,//14
			_BUZZER_PWM/(float)_FREQ_1_DO_H,//15
			_BUZZER_PWM/(float)_FREQ_2_RE_H,//16
			_BUZZER_PWM/(float)_FREQ_3_MI_H,//17
			_BUZZER_PWM/(float)_FREQ_4_FA_H,//18
			_BUZZER_PWM/(float)_FREQ_5_SO_H,//19
			_BUZZER_PWM/(float)_FREQ_6_LA_H,//20
			_BUZZER_PWM/(float)_FREQ_7_SI_H,//21
			_BUZZER_PWM/(float)_FREQ_1_DO_U,//22
			_BUZZER_PWM/(float)_FREQ_2_RE_U,//23
			_BUZZER_PWM/(float)_FREQ_3_MI_U,//24
			_BUZZER_PWM/(float)_FREQ_4_FA_U,//25
			_BUZZER_PWM/(float)_FREQ_5_SO_U,//26
			_BUZZER_PWM/(float)_FREQ_6_LA_U,//27
			_BUZZER_PWM/(float)_FREQ_7_SI_U //28
		};
		
void beep_setvolume(u8 percent)
{
		buzzer_duty = percent * ((float)percent + 10) / 86 + 1;
}
void beep(u8* arr, u8 len, u16 bpm)
{
		u8 i;
		timer_parameter_struct timer_initpara;
		for(i = 0;i < len; i++)
				music[i] = *(arr++);
		curr_note = 0;
		total_note = len;
		buzzer_running = 1;
		beatgap = 60000 / bpm;
	
		rcu_periph_clock_enable(RCU_TIMER2);

		timer_deinit(TIMER2);
		timer_struct_para_init(&timer_initpara);
		//single period is 0.1ms
		timer_initpara.prescaler				 = 12000 - 1;
		timer_initpara.alignedmode			 = TIMER_COUNTER_EDGE;
		timer_initpara.counterdirection	= TIMER_COUNTER_UP;
		timer_initpara.period						= beatgap * 10 - 1;
		timer_initpara.clockdivision		 = TIMER_CKDIV_DIV1;
		timer_init(TIMER2, &timer_initpara);

		/* enable the TIMER interrupt */
		timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
		timer_interrupt_enable(TIMER2, TIMER_INT_UP);
		
		timer_enable(TIMER2);
		nvic_irq_enable(TIMER2_IRQn, 2, 2);
}

void buzzer_wait_until_idle(void)
{
		while(buzzer_running);
}
void TIMER2_IRQHandler(void)
{
		//must clear the flag, otherwise the interrupt was triggered repeatly
		timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
		//next note
		if(curr_note < total_note)
		{
				volatile u8 note = music[curr_note];
			
				TIMER_CAR(_BUZZER_TIMER) = (u32) _tone_period[note];//period (reload)
//				if(note > 18 && buzzer_duty < 130) duty_corrected += (float)(note - 18) * .1f * (130 - buzzer_duty);
				TIMER_CH0CV(_BUZZER_TIMER) = (u32)TIMER_CAR(_BUZZER_TIMER) * buzzer_duty / 256;//pulse (volume)
				TIMER_CCHP(_BUZZER_TIMER) |= (u32) TIMER_CCHP_POEN;//enable pwm
				TIMER_CTL0(_BUZZER_TIMER) |= (u32)TIMER_CTL0_ARSE;//enable auto reload
				curr_note++;
		}
		else{
				buzzer_running = 0;
				TIMER_CCHP(_BUZZER_TIMER) &= (~(u32)TIMER_CCHP_POEN);//disable pwm
				TIMER_CTL0(_BUZZER_TIMER) &= ~(u32)TIMER_CTL0_ARSE;//disable auto reload
			
				timer_disable(TIMER2);
        rcu_periph_reset_enable(RCU_TIMER2RST);
        rcu_periph_reset_disable(RCU_TIMER2RST);
				NVIC_DisableIRQ(TIMER2_IRQn);
		}
}
//void buzzer_tick(void)
//{	
//		//end beep
//		if(curr_note >= total_note)
//		{
//				buzzer_running = 0;
//				TIMER_CCHP(_BUZZER_TIMER) &= (~(u32)TIMER_CCHP_POEN);//disable pwm
//				TIMER_CTL0(_BUZZER_TIMER) &= ~(u32)TIMER_CTL0_ARSE;//disable auto reload
//				return;
//		}
//		if(!buzzer_running) return;
//		//next note
//		if(curr_tick >= curr_tick_total||(curr_tick == 0 && curr_note == 0))
//		{
//				u8 note = music[curr_note++];
//				TIMER_CAR(_BUZZER_TIMER) = (u32) _tone_period[note];//period (reload)
////				if(note > 18 && buzzer_duty < 130) duty_corrected += (float)(note - 18) * .1f * (130 - buzzer_duty);
//				TIMER_CH0CV(_BUZZER_TIMER) = (u32)TIMER_CAR(_BUZZER_TIMER) * buzzer_duty / 256;//pulse (volume)
//				TIMER_CCHP(_BUZZER_TIMER) |= (u32) TIMER_CCHP_POEN;//enable pwm
//				TIMER_CTL0(_BUZZER_TIMER) |= (u32)TIMER_CTL0_ARSE;//enable auto reload
//				curr_tick = 1;
//				return;
//		}
//		//gap between notes
//		if(curr_tick == curr_tick_total - 1)
//		{
//				TIMER_CCHP(_BUZZER_TIMER) &= (~(u32)TIMER_CCHP_POEN);//disable pwm
//				TIMER_CTL0(_BUZZER_TIMER) &= ~(u32)TIMER_CTL0_ARSE;//disable auto reload
//		}
//		curr_tick++;
//		
//}