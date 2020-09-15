#ifndef IMU_H
#define IMU_H

// https://stm32f4-discovery.net/hal_api/group___t_m___m_p_u6050.html
/* SPECIFICALLY!
 *  -> https://stm32f4-discovery.net/hal_api/group___t_m___m_p_u6050___functions.html#gaa762763ccd791e995b61120fab1dfa4e
 */

#include "tm_stm32_mpu6050.h"
#include <stdlib.h>
#include <stdint.h>

void IMU_init(void);
void IMU_read(void);
float get_accY(void);

#endif
