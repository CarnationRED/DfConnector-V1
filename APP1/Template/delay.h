
#include "gd32c10x.h"
#include <stdio.h>
void timing_config(void);
void tim4_it(void);


void led_flash_config(void);
void tim3_it(void);

void delay_ms(uint16_t time);
void delay_us(uint16_t time);
uint32_t current_time01ms(void);
uint32_t current_totaltick(void);

uint8_t heartbeat_pending(void);
void clear_heartbeat_pending(void);

uint8_t heartbeat_timeout(void);
void clear_heartbeat_timeout(void);
