#ifndef __SYS_Init_H
#define __SYS_Init_H
#include "config.h"

void SystemClock_Config(void);

void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);

void SYS_Init(void);//初始化所有硬件

#endif
