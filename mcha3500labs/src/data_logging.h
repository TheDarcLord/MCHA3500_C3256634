#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include "IMU.h"
#include "potentiometer.h"
#include "stm32f4xx_hal.h" 
#include <stdint.h>
#include <limits.h>
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "kalman.h"


typedef enum {
    POT     = 0,
    IMU_POT = 1,
    IMU     = 2
} SENSOR;

void logging_init(void);
void data_logging_start(SENSOR X);
void data_logging_stop(void);
// Sensors to Log

#endif