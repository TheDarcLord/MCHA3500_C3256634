#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>   
#include <stdint.h>
#include <math.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_it.h"
#include "ammeter.h"
//#include "stm32f4xx_hal_adc.h"

//#define     MAXDUTY (0x000A)        // 10 -> 10 Mhz
//#define     MAXDUTY (0x0064)        // 100  -> 1 MHz
//#define     MAXDUTY (0x30D4)        // 12 500 -> 8 kHz
#define     MAXDUTY (0x2710)        // 10 000 -> 10 kHz
#define     MAXVOLT (12.1f)
#define     FWD   1     // Forward
#define     BCK   2     // Backward
#define     BRK   3     // Brake

void        motor_init(void);
uint16_t    motor_set_voltage(float voltage);
void        _motor_set_direction(uint8_t dir);

#endif