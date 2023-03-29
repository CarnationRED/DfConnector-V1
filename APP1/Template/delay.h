
#include "gd32c10x.h"
#include <stdio.h>
#include "gd32c10x_eval.h"
void tim4_config(void);
void tim4_it(void);


void tim3_slow(void);
void tim3_normal(void);

void tim3_config(void);
void tim3_it(void);

void delay_ms(uint32_t time);
void delay_us(uint32_t time);
uint32_t current_time01ms(void);
uint32_t current_totaltick(void);

uint8_t heartbeat_pending(void);
void clear_heartbeat_pending(void);

uint8_t heartbeat_timeout(void);
void clear_heartbeat_timeout(void);
