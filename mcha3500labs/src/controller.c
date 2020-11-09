#include "controller.h"

#define T 0.01

/* Defining sutiable matrix variables for: 
 * https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
 * | θ | - Pendulum Angle            -> MPU Acc         0
 * | x'| - Cart Velocity                                1
 * | θ'| - Pendulum Angular Velocity -> MPU Gyro        2
 * | z | - Error State of the Integrator                3
 * 
 * Define control matrix values */

/* Define control matrix values */
/* COLUMNS = LENGTH CTRL HORIZON */
static float ctrl_H_f32[CTRL_N_HORIZON*CTRL_N_HORIZON] = {
    2.340564407470560,	0.320892625666911,	0.301511939073556,	0.282415056489929,	0.263594835913616,
    0.320892625666911,	2.309380570170620,	0.290794961939758,	0.272474840547024,	0.254413485269127,
    0.301511939073556,	0.290794961939758,	2.280331247179360,	0.262769263285105,	0.245448600436500,
    0.282415056489929,	0.272474840547024,	0.262769263285105,	2.253293102940420,	0.236695359210516,
    0.263594835913616,	0.254413485269127,	0.245448600436500,	0.236695359210516,	2.228149052088300,
};

/* ROWS = LENGTH CTRL HORIZON */
static float ctrl_Gamma_f32[CTRL_N_HORIZON*CTRL_N_STATE] = {
    -236.922573072506,	-11.6851420589415,	-104.407428303733,	-6.83914711184453,
    -221.138361910047,	-10.8435555348667,	-97.3795503514464,	-6.28072220821059,
    -205.728628214385,	-10.0256194038905,	-90.5226557834554,	-5.74174796957586,
    -190.684585828527,	-9.23078697821476,	-83.8327363868148,	-5.22176581373894,
    -175.997633610805,	-8.45852324988399,	-77.3058688925836,	-4.72032737432637
};

static float ctrl_f_f32[CTRL_N_HORIZON] = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
};
static float ctrl_xHat_f32[CTRL_N_STATE] = {
    /* estimate of state, 4x1 */
    0.0, 
    0.0, 
    0.0, 
    0.0
};
static float ctrl_u_f32[CTRL_N_INPUT] = {
    /* control action, 1x1 */
    0.0
};
static float ctrl_Ustar_f32[CTRL_N_HORIZON*CTRL_N_INPUT] = {
    0.0, 
    0.0,
    0.0, 
    0.0,
    0.0
};

/* TRANSPOSE OF A FROM MATLAB -> A' */
static float ctrl_A_f32[CTRL_N_INEQ_CONST*CTRL_N_HORIZON] = {
    1.0,    -1.0,     0.0,     0.0,     0.0,    -1.0,     1.0,     0.0,     0.0,     0.0,
    0.0,     1.0,    -1.0,     0.0,     0.0,     0.0,    -1.0,     1.0,     0.0,     0.0,
    0.0,     0.0,     1.0,    -1.0,     0.0,     0.0,     0.0,    -1.0,     1.0,     0.0,
    0.0,     0.0,     0.0,     1.0,    -1.0,     0.0,     0.0,     0.0,    -1.0,     1.0,
    0.0,     0.0,     0.0,     0.0,     1.0,     0.0,     0.0,     0.0,     0.0,    -1.0
};
static float ctrl_b_f32[CTRL_N_INEQ_CONST] ={
    delta_u_max,
    delta_u_max,
    delta_u_max,
    delta_u_max,
    delta_u_max,
    -delta_u_min,
    -delta_u_min,
    -delta_u_min,
    -delta_u_min,
    -delta_u_min
};
static float ctrl_xl_f32[CTRL_N_LB_CONST] = { 
    u_min, 
    u_min,
    u_min,
    u_min,
    u_min
};
static float ctrl_xu_f32[CTRL_N_UB_CONST] = { 
    u_max, 
    u_max,
    u_max,
    u_max,
    u_max
};
static float ctrl_lm_f32[CTRL_N_EQ_CONST+CTRL_N_INEQ_CONST+CTRL_N_LB_CONST+CTRL_N_UB_CONST] = { };

/* Define control matrix variables */
// rows, columns, data array
/* d) Define and add code to initialise matrix structures for H, Γ, f, xˆ and u...
 * The remaining matrices do not need to me initialised as matrix structures.. 
 * as they will not be used in matrix manipulations using the CMSIS-DSP libraries.
 */ 
arm_matrix_instance_f32 ctrl_H =        {CTRL_N_HORIZON, CTRL_N_HORIZON, (float32_t *)ctrl_H_f32};
arm_matrix_instance_f32 ctrl_Gamma =    {CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_Gamma_f32};
arm_matrix_instance_f32 ctrl_f =        {CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32};
arm_matrix_instance_f32 ctrl_xHat =     {CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32};
arm_matrix_instance_f32 ctrl_u =        {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};

/* Control functions */
void ctrl_init(void) {
arm_mat_init_f32(&ctrl_H, CTRL_N_HORIZON, CTRL_N_HORIZON, (float32_t *)ctrl_H_f32);
arm_mat_init_f32(&ctrl_Gamma, CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_Gamma_f32);
arm_mat_init_f32(&ctrl_f, CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32);
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
     * Z(k+1) = Z(k) + T*Cr*X(k) + T*Dr*U(k);  -LQR
     * Z(k+1) = Z(k) + T*Cr*(X(k)' - xStar);   -MPC
     * -> Simplified! -> Zk + T*X(k)[x'] + 0
     */ 
    
    ctrl_xHat_f32[3] = ctrl_xHat_f32[3] + (T * ctrl_xHat_f32[1]);
    /* Compute f vector */
    arm_mat_mult_f32(&ctrl_Gamma,&ctrl_xHat,&ctrl_f);
    /* Solve for optimal inputs over control horizon ->
        (
            varint n,           -   Dimension of the vector x
            varint me,          -   Number of equality constraints
            varint mc,          -   Number of inequality constraints
            varint nl,          -   Number of lower bound constraints
            varint nu,          -   Number of upper bound constraints
            float * H,          -   H matrix from quadratic cost. Should be flattened into a array of length n×n.
            float * f,          -   f vector from quadratic cost.
            float * A,          -   A matrix from inequality constraints. Should be flattened into a array of length mc×n
            float * b,          -   b vector from inequality constraints.
            float * l,          -   l_b vector from bound constraints.
            float * u,          -   ub vector from bound constraints.
            float * x,          -   Array where solution to QP problem is stored.
            float * lm,         -   Array where Lagrange multipliers are stored.
            varint display,     -   1 or 0
            varint * numits,    -   
            varint * numadd,    -
            varint * numdrop    -
        )
    */



    qpas_sub_noblas(
        CTRL_N_HORIZON, 
        CTRL_N_EQ_CONST,    // Number of equality constraints
        CTRL_N_INEQ_CONST,  // Number of inequality constraints
        CTRL_N_LB_CONST,    // Number of lower bound constraints
        CTRL_N_UB_CONST,    // Number of upper bound constraints
        ctrl_H_f32,         // H matrix from quadratic cost
        ctrl_f_f32,         // f vector from quadratic cost
        ctrl_A_f32,         // A matrix from inequality constraints
        ctrl_b_f32,         // b vector from inequality constraints
        ctrl_xl_f32,        // l_b vector from bound constraints
        ctrl_xu_f32,        // u_b vector from bound constraints
        ctrl_Ustar_f32,     // Array where solution to QP problem is stored. *USTAR TEMP!
        ctrl_lm_f32,        // Array where Lagrange multipliers are stored.
        0,                  // Display 1 or 0
        &numits, &numadd, &numdrop);
    /* Extract first control term */
    // LEC 9 -> 48
    ctrl_u_f32[0] = ctrl_Ustar_f32[0];
    ctrl_b_f32[0] = delta_u_max + ctrl_u_f32[0];
    ctrl_b_f32[CTRL_N_HORIZON] = -delta_u_min - ctrl_u_f32[0];
    /* Print functions for debugging. Uncomment to use */
    // printmatrix (CTRL_N_HORIZON,CTRL_N_HORIZON,ctrl_H_f32,CTRL_N_HORIZON,"H");
    // printvector (CTRL_N_HORIZON, ctrl_f_f32, "f");
}


