#ifndef __WIFI__
#define __WIFI__
#include "gd32c10x.h"
#include <stdio.h>
#define WIFI_SEND_BUFFER 1024

// Define the SPI port and pins
#define WIFI_SPI_PORT			SPI0
#define WIFI_PIN_PORT			GPIOA
#define WIFI_IT_PIN				GPIO_PIN_4
#define WIFI_SCK_PIN 			GPIO_PIN_5
#define WIFI_MISO_PIN			GPIO_PIN_6
#define WIFI_MOSI_PIN			GPIO_PIN_7

#define WIFI_NCS_RST_PORT	GPIOC
#define WIFI_NCS_PIN			GPIO_PIN_4
#define WIFI_RST_PIN			GPIO_PIN_5

typedef enum
{	
	WIFI_STATUS_INIT_PENDING,
	WIFI_STATUS_SPI_COMPLETE,
	WIFI_STATUS_OPMODE_COMPL,
	WIFI_STATUS_INIT_COMPLET,
	WIFI_STATUS_WAIT_FOR_CNT,
	WIFI_STATUS_CONCTED_IDLE,
	WIFI_STATUS_ERR_SPI_INIT,
	WIFI_STATUS_ERR_SPI_SELC,
	WIFI_STATUS_ERR_STRESERR,
	WIFI_STATUS_ERR_SETOPMOD,
	WIFI_STATUS_ERR_CONFIGAP,
	WIFI_STATUS_ERR_SETCPSVR,
	WIFI_STATUS_ERR_CONTLOST,
	WIFI_STATUS_ERR_SENDDATA,
	WIFI_STATUS_ERR_DATAEROR,
	WIFI_STATUS_ERR_CTLDATER,
}WIFI_STATUS;
/**
 * @brief  WIFI模块工作模式
 */
typedef enum {
    WIFI_OPMODE_STA = 1,   // STA
    WIFI_OPMODE_AP = 2,    // AP
    WIFI_OPMODE_STAAP = 3, // STA+AP
} WIFI_OPMODE;

/**
 * @brief  WIFI模块工作模式
 */
typedef enum {
    WIFI_WRKMODE_UDP = 0,       // UDP
    WIFI_WRKMODE_TCPClient = 1, // TCP客户端
    WIFI_WRKMODE_TCPServer = 2, // TCP服务器
} WIFI_WRKMODE;

typedef uint32_t		u32;
typedef uint16_t		u16;
typedef uint8_t			u8;

// Define the SPI mode and speed
#define SPI_SPEED   15000000 // 15 MHz

void wifi_reset(void);
void wifi_Init(void); 
void wifi_SetupTCPServer(u16 port,u8 link_no);
u8 wifi_GetClients(u8 link_no);
u8 wifi_GetSTACount(void);
void wifi_disableIT(void);
void wifi_enableIT(void);

/// @brief 将数据传输到WIFI模块，然后返回。模块将自行在空闲时发送
/// @param data
/// @param dataLen
u16 wifi_SendData(uint8_t data[], uint16_t dataLen,u8 link_no);

#endif
