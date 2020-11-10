#ifndef KALMAN_H
#define KALMAN_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "IMU.h"
#include "ammeter.h"
#include "encoder.h"
#include "motor.h"

#define  T  (float) 0.01
// Motor Params
#define  Ra (float) 6.6002
#define  La (float) 0.4000
#define  Kw (float) 0.0055

void initKF(void);
float runKF(float Ia, float Vin, float Wa);
float getFilterAngle(void);
float getFilterOmega(void);
float getFilterCurrent(void);

/* Sensor Fusion!>?
 * MODEL:
 * 
 * (k + 1)          A               xk              B              uk
 * 
 * |  ω' |   | 1  0  0     0   |   | ω |   |  0        0    |   |  Vin |
 * |  θ' | = | T  1  0     0   | . | θ | + |  0        0    | . |  Wa  |
 * |  b' |   | 0  0  1     0   |   | b |   |  0        0    |  
 * |  I' |   | 0  0  0  -Ra/La |   | I |   | 1/La  (-Kw/La) |
 * 
 *    Y         C          xk     D uk        v(k)
 * 
 * | θ |   | 0 1 0 0 |   | ω |                 get_angle(RADIANS)
 * | ω | = | 1 0 1 0 | . | θ | +    0    +     get_gyroY()
 * | I |   | 0 0 0 1 |   | b |                 ammeter_get_value()
 *                       | I |
 * 
 */

#endif