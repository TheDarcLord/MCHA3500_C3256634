#ifndef AMMETER_H
#define AMMETER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "math.h"

#define MAX12BIT (4095e0)
#define PIN_VMAX (3.3e0)
#define V_PER_C (140e-3)


float   ammeter_get_value(void);
void    ammeter_deinit(void);
void    ammeter_init(void);
void    set_direction(float);
/* 140mV per Amp 
 * -> PC4 - CHANNEL 14 ! -> ADC12_IN14
 * -> ADC 1 or 2 ... IN channel 14
 * 3.3V <-> 4095.0
 * 
 *     3.3V            
 * ------------- * x  =  Amps
 * 4095 * 140e-3
 */

#endif