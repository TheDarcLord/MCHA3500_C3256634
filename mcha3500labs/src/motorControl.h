#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "stm32f4xx_hal.h" 
#include <stdint.h>
#include <limits.h>
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include "encoder.h"
#include "motor.h"
#include "ammeter.h"

void ctrlMotor_init(void);
void ctrlMotor_start(void);
void ctrlMotor_stop(void);

void motor_set_torque(float t);

#endif