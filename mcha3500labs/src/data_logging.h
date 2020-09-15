#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include "potentiometer.h"
#include "stm32f4xx_hal.h" 
#include <stdint.h>
#include <limits.h>

static void log_pointer(void *argument);
void logging_init(void);
void data_logging_start(void);
void data_logging_stop(void);
// Sensors to Log
static void log_potentiometer(void *argument);

#endif