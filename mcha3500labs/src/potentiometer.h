#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <stdint.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"

#define MAX12BIT (4095e0)
#define PIN_VMAX (3.3e0)
#define V_PER_C (140e-3)

void        pot_init(void);
void        pot_deinit(void);
uint16_t    pot_get_value(void);
float       get_pot_voltage(void);

#endif