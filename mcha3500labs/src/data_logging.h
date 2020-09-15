#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include "potentiometer.h"
#include "stm32f4xx_hal.h" 
#include <stdint.h>
#include <limits.h>

void logging_init(void);
void data_logging_start(void);
void data_logging_stop(void);
// Sensors to Log

#endif