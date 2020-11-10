#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "qpas_sub_noblas.h"
#include "arm_math.h"
#include <string.h>
#include "math.h"
#include "cmsis_os2.h"
#include "kalman.h"
#include "motor.h"
#include "motorControl.h"

// Dummy variables for QP solver
int numits,numadd,numdrop;

/* Define actuator limits */
#define u_min           -0.19
#define u_max           0.19
#define delta_u_min     -0.01
#define delta_u_max     0.01
#define N               30.0

void ctrl_init(void);
void ctrl_update(void *argument);
void ctrl_set_x1h(float x1h);
void ctrl_set_x2h(float x2h);
void ctrl_set_x3h(float x3h);

float getControl(void);
void ctrl_start(void);
void ctrl_stop(void);

enum {
    CTRL_N_INPUT  = 1,      // number of controller inputs
    CTRL_N_STATE  = 4,      // number of controller states
    CTRL_N_HORIZON = 5,     // number of controller outputs / plant inputs
    CTRL_N_EQ_CONST = 0,    // number of equality constraints
    CTRL_N_INEQ_CONST = 10,  // number of inequality constraints
    CTRL_N_LB_CONST = 5,    // number of lower bound constraints
    CTRL_N_UB_CONST = 5,    // number of upper bound constraints
};


#endif