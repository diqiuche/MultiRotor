#ifndef _Led_H_
#define _Led_H_

#include "stm32f1xx_hal.h"

#define LED_NUM 2
#define LA	    0x01
#define LB      0x02

#define E_READY       0
#define E_CALI        1
#define E_BAT_LOW     2
#define E_CALI_FAIL   3
#define E_LOST_RC     4
#define E_AUTO_LANDED 5
#define E_BatChg      6

typedef union {
	uint8_t byte;
	struct
	{
		uint8_t A        	: 1;
		uint8_t B        	: 1;
		uint8_t reserved 	: 2;
	} bits;
} LEDBuf_t;

typedef struct Led_tt
{
	uint8_t  event;
	uint8_t  state;
	uint16_t cnt;
} LED_t;

#define LedA_on      HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_12, GPIO_PIN_RESET)
#define LedA_off     HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_12, GPIO_PIN_SET)

#define LedB_on      HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_13, GPIO_PIN_RESET)
#define LedB_off     HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_13, GPIO_PIN_SET)

#define LEDA_troggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12)
#define LEDB_troggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13)

extern LED_t LEDCtrl;

void LedInit(void);   //Led初始化函数外部声明
void LEDFSM(void);
void LedFlash(void);

#endif
