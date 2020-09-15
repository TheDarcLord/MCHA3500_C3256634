#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include "potentiometer.h"
#include "stm32f4xx_hal.h" 
#include <stdint.h>
#include <limits.h>

void log_potentiometer(void *argument);
void logging_init(void);
void potentiometer_logging_start(void);
void potentiometer_logging_stop(void);

#endif