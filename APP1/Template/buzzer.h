#ifndef __buzzer__
#define __buzzer__
#include "WIFI.h"
#include "gd32c10x.h"
#define _BUZZER_TIMER TIMER0
#define _BUZZER_TIMER_RCU RCU_TIMER0
#define _BUZZER_TIMER_CH TIMER_CH_0
#define _BUZZER_PSC 1200
#define _BUZZER_PWM 120000000u / _BUZZER_PSC //100000u

#define _FREQ_1_DO_L 262
#define _FREQ_2_RE_L 294
#define _FREQ_3_MI_L 330
#define _FREQ_4_FA_L 349
#define _FREQ_5_SO_L 392
#define _FREQ_6_LA_L 440
#define _FREQ_7_SI_L 494
#define _FREQ_1_DO_M 523
#define _FREQ_2_RE_M 587
#define _FREQ_3_MI_M 659
#define _FREQ_4_FA_M 698
#define _FREQ_5_SO_M 784
#define _FREQ_6_LA_M 880
#define _FREQ_7_SI_M 988
#define _FREQ_1_DO_H 1047
#define _FREQ_2_RE_H 1175
#define _FREQ_3_MI_H 1319
#define _FREQ_4_FA_H 1397
#define _FREQ_5_SO_H 1568
#define _FREQ_6_LA_H 1760
#define _FREQ_7_SI_H 1967
#define _FREQ_1_DO_U 2093
#define _FREQ_2_RE_U 2350
#define _FREQ_3_MI_U 2638
#define _FREQ_4_FA_U 2794
#define _FREQ_5_SO_U 3137
#define _FREQ_6_LA_U 3521
#define _FREQ_7_SI_U 3952

void beep(u8* arr, u8 len, u16 bpm);
void beep_setvolume(u8 percent);

void TIMER2_IRQHandler(void);
void buzzer_wait_until_idle(void);

#endif
