#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "math.h"


/* 30 ->
#define GEARRATIO (30.0f)
#define CPR (1920.0f)
*/
/* 19 -> */
#define GEARRATIO (18.75f)
#define CPR (1200.0f)           
#define M_PI    3.14159265358979323846

void    encoder_init(void);
void    encoder_set_count(int32_t);
float   encoder_get_count(void);
float   encoder_pop_count(void);
void    encoder_edge_A_isr(void);
void    encoder_edge_B_isr(void);
float   countToRadians(int32_t c);

#endif