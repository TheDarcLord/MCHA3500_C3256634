#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <string.h>
#include "math.h"


void ctrl_init(void);

void ctrl_set_x1h(float x1h);
void ctrl_set_x2h(float x2h);
void ctrl_set_x3h(float x3h);

float getControl(void);
void ctrl_update(void);


#endif