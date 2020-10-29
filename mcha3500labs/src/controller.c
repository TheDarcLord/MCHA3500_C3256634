#include "controller.h"

#define CTRL_N_INPUT 1
#define CTRL_N_STATE 4
#define T 0.01

/* Defining sutiable matrix variables for: 
 * https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
 * | θ | - Pendulum Angle            -> MPU Acc         0
 * | x'| - Cart Velocity                                1
 * | θ'| - Pendulum Angular Velocity -> MPU Gyro        2
 * | z | - Error State of the Integrator                3
 * 
 * Define control matrix values */
static float ctrl_mK_f32[CTRL_N_INPUT * CTRL_N_STATE] = {
    /* negative K, 1x4 */
    43.9434,  9.1494,  6.8254,  0.9429
};
static float ctrl_xHat_f32[CTRL_N_STATE] = {
    /* estimate of state, 4x1 */
    0.0, 0.0, 0.0, 0.0
};
static float ctrl_u_f32[CTRL_N_INPUT] = {
    /* control action, 1x1 */
    0.0
};

/* Define control matrix variables */
// rows, columns, data array
arm_matrix_instance_f32 ctrl_mK = {CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32};
arm_matrix_instance_f32 ctrl_xHat = {CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};

void ctrl_init(void) {
    arm_mat_init_f32(&ctrl_mK, CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32);
    arm_mat_init_f32(&ctrl_xHat, CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
}

/* Update state vector elements */
void ctrl_set_x1h(float x1h) {
    // Update state x1h
    ctrl_xHat_f32[0] = x1h;
}
void ctrl_set_x2h(float x2h) {
    // Update state x2h
    ctrl_xHat_f32[1] = x2h;
}
void ctrl_set_x3h(float x3h) {
    // Update state x3h
    ctrl_xHat_f32[2] = x3h;
}

/* Get the current control output */
float getControl(void) {
    return ctrl_u_f32[0];
}

/* Update control output */
void ctrl_update(void) {
    /* Increment integrator state:
     * Z(k+1) = Zk + T*Cr*X(k) + T*Dr*U(k)
     * -> Simplified! -> Zk + T*X(k)[x'] + 0
     */ 
    ctrl_xHat_f32[3] = ctrl_xHat_f32[3] + (T * ctrl_xHat_f32[1]);

    /* Compute control action:
     *             |Xk|
     * U(k) = -K * |Zk|
     * ie: xHat
     * 
     * arm_mat_mult_f32	(
     *      arm_matrix_instance_f32 *pSrcA, ->  [in]	pSrcA	points to the first input matrix structure
     *      arm_matrix_instance_f32 *pSrcB, ->  [in]	pSrcB	points to the second input matrix structure
     *      arm_matrix_instance_f32 *pDst   ->  [out]	pDst	points to output matrix structure
     * )
     */

    arm_mat_mult_f32(&ctrl_mK, &ctrl_xHat, &ctrl_u);
}


