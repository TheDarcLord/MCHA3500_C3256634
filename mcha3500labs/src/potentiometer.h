#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <stdint.h>
#include <limits.h>

enum {
    POT_FULL_SCALE = UINT16_MAX - UINT16_MAX/(1<<12), // Full scale for 12 bit left aligned ADC
};

void     pot_init(void);
void     pot_deinit(void);
uint16_t pot_get_value(void);
float get_pot_voltage(void);

#endif