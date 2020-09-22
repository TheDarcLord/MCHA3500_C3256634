#ifndef UART_SERVO_H
#define UART_SERVO_H

#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_rcc.h"
#include "cmsis_os2.h"
#include "stm32f446xx.h"
#include "stdlib.h"
#include "string.h"
void servo_comms_init(void);
void servo_command(void);
// Note: No other public functions needed;
//       all access via stdin and stdout

#define GPIO_AF7_USART3 ((uint8_t)0x07) 

#endif