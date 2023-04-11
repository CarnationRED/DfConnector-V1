#ifndef __SD_H__
#define __SD_H__
#include "main.h"
#include "delay.h"
#include "gd32c10x.h"
#include <stdio.h>

#define SD_SPI_PORT			SPI2
#define SD_PIN_PORT			GPIOB
#define SD_SCK_PIN 			GPIO_PIN_3
#define SD_MISO_PIN			GPIO_PIN_4
#define SD_MOSI_PIN			GPIO_PIN_5

#define SD_NCS_PORT			GPIOD
#define SD_NCS_PIN			GPIO_PIN_2

void sd_init(void);

#endif
