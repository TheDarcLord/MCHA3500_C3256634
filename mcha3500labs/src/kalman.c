#include "kalman.h"

#define KALMAN_STATES 4
#define KALMAN_INPUTS 2
#define SENSOR_OUTPUT 3


static float xmd[KALMAN_STATES] = {
    0.0,
    0.0,
    0.0,
    0.0
};

static float ukd[KALMAN_INPUTS] = {
    0.0,
    0.0
};

static float vkd[SENSOR_OUTPUT] = {
    0.0,
    0.0
};

static float ypd[SENSOR_OUTPUT] = {
    0.0,
    0.0,
    0.0
};

static float yid[SENSOR_OUTPUT] = {
    0.0,
    0.0,
    0.0
};

static float Ad[KALMAN_STATES*KALMAN_STATES] = {
    1.0,    0.0,   0.0,         0.0,
    T,      1.0,   0.0,         0.0,
    0.0,    0.0,   1.0,         0.0,
    0.0,    0.0,   0.0,   -1*(Ra/La)
};

static float Bd[KALMAN_STATES*KALMAN_INPUTS] = {
    0.0,        0.0,
    0.0,        0.0,
    0.0,        0.0,
    (1/La),     (-Kw/La)
};

static float Cd[SENSOR_OUTPUT*KALMAN_STATES] = {
    0.0, 1.0, 0.0, 0.0,
    1.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0
};

static float Rd[SENSOR_OUTPUT*SENSOR_OUTPUT] = {
    0.3213e-4,  0.0,        0.0,
    0.0,        0.3213e-4,  0.0,
    0.0,        0.0,        0.0022
};

static float Qd[KALMAN_STATES*KALMAN_STATES] = {
    1e-7,   0.0,    0.0,    0.0,
    0.0,    1e-7,   0.0,    0.0,
    0.0,    0.0,    1e-7,   0.0,
    0.0,    0.0,    0.0,    1e-15
};

static float Pd[KALMAN_STATES*KALMAN_STATES] = {
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0
};
                               // ROWS        // Columns
arm_matrix_instance_f32 A   = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Ad};   // 4x4
arm_matrix_instance_f32 B   = {KALMAN_STATES, KALMAN_INPUTS, (float32_t*)Bd};   // 4x2
arm_matrix_instance_f32 C   = {SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)Cd};   // 3x4
arm_matrix_instance_f32 R   = {SENSOR_OUTPUT, SENSOR_OUTPUT, (float32_t*)Rd};   // 3x3
arm_matrix_instance_f32 Q   = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Qd};   // 4x4
arm_matrix_instance_f32 Pm  = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Pd};   // 4x4
arm_matrix_instance_f32 vk  = {SENSOR_OUTPUT, 1, (float32_t*)vkd};  // 3x1
arm_matrix_instance_f32 uk  = {KALMAN_INPUTS, 1, (float32_t*)ukd};  // 2x1
arm_matrix_instance_f32 xm  = {KALMAN_STATES, 1, (float32_t*)xmd};  // 4x1
arm_matrix_instance_f32 yp  = {SENSOR_OUTPUT, 1, (float32_t*)ypd};  // 3x1
arm_matrix_instance_f32 yi  = {SENSOR_OUTPUT, 1, (float32_t*)yid};  // 3x1

void runKF(void) {
    // 1st, 2nd, Out
    arm_mat_mult_f32(C,xm,yi);
}