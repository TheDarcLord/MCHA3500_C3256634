#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>   
#include <stdint.h>
#include <math.h>
#include "cmsis_os2.h"
//#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

//#define     MAXDUTY (0x000A)        // 10 -> 10 Mhz
//#define     MAXDUTY (0x0064)        // 100  -> 1 MHz
//#define     MAXDUTY (0x30D4)        // 12 500 -> 8 kHz
#define     MAXDUTY (0x2710)        // 10 000 -> 10 kHz
#define     MAXVOLT (12.1f)
#define     MOTOR_DIR_FWD   1
#define     MOTOR_DIR_BCK   2
#define     MOTOR_DIR_BRK   3

void        motor_set(float voltage);
void        motor_init(void);
void        _motor_set_direction(uint8_t dir);
uint16_t    _motor_set_dutycyle(float percent);

#endif