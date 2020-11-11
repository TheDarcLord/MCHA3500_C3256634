#include "controller.h"

/* Defining sutiable matrix variables for: 
 * https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
 * | θ | - Pendulum Angle            -> MPU Acc         0
 * | ϕ'| - Wheel Velocity                               1
 * | θ'| - Pendulum Angular Velocity -> MPU Gyro        2
 * | z | - Error State of the Integrator                3
 * 
 * Define control matrix values */

/* Define control matrix values */

osTimerId_t _CtrlID;


void ctrl_start(void) {
    //printf("Controller Started");
    osTimerStart(_CtrlID, 5U);
}

void ctrl_stop(void) {
    osTimerStop(_CtrlID);
}


/* COLUMNS = LENGTH CTRL HORIZON */
static float ctrl_H_f32[CTRL_N_HORIZON*CTRL_N_HORIZON] = {
131050206.745791,	28739824.3524483,	26600911.2368757,	24620741.5553675,	22787533.9277213,
28739824.3524483,	126601595.407971,	24622056.8973691,	22789433.4896140,	21092821.6613021,
26600911.2368757,	24622056.8973691,	122790066.927171,	21094039.3135440,	19523885.5195303,
24620741.5553675,	22789433.4896140,	21094039.3135440,	119524471.999971,	18071390.8623214,
22787533.9277213,	21092821.6613021,	19523885.5195303,	18071390.8623214,	116726695.832579
}; // Current Best Guess at model

/* ROWS = LENGTH CTRL HORIZON */
static float ctrl_Gamma_f32[CTRL_N_HORIZON*CTRL_N_STATE] = {
-348272777.225285,	-2731330.81450069,	-47097298.5176270,	-58012.7769453235,
-322320089.847024,	-2434551.27269786,	-43493039.3579583,	-51342.9858407824,
-298293537.012653,	-2159818.18743220,	-40156298.7046334,	-45169.2280865449,
-276050167.884346,	-1905495.35157590,	-37067222.2831482,	-39454.6896736883,
-255457640.884982,	-1670067.99122911,	-34207429.3275793,	-34165.2887932550,
}; // Current Best Guess at model

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
/* Define and add code to initialise matrix structures for H, Γ, f, xˆ and u... */ 
arm_matrix_instance_f32 ctrl_H =        {CTRL_N_HORIZON, CTRL_N_HORIZON, (float32_t *)ctrl_H_f32};
arm_matrix_instance_f32 ctrl_Gamma =    {CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_Gamma_f32};
arm_matrix_instance_f32 ctrl_f =        {CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32};
arm_matrix_instance_f32 ctrl_xHat =     {CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32};
arm_matrix_instance_f32 ctrl_u =        {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};

/* Control functions */
void ctrl_init(void) {
    /* Initialise timer */
    _CtrlID = osTimerNew(ctrl_update, osTimerPeriodic, (void *)5, NULL);
    arm_mat_init_f32(&ctrl_H, CTRL_N_HORIZON, CTRL_N_HORIZON, (float32_t *)ctrl_H_f32);
    arm_mat_init_f32(&ctrl_Gamma, CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_Gamma_f32);
    arm_mat_init_f32(&ctrl_f, CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32);
    arm_mat_init_f32(&ctrl_xHat, CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
}

/* Update control output */
void ctrl_update(void *argument) {
    UNUSED(argument);
    //printf("STUCK");
    
    ctrl_xHat_f32[0] = getFilterAngle();            // -> MPU Acc
    ctrl_xHat_f32[1] = (motor_get_velocity() / N);  // -> Wheel Velocity 
    ctrl_xHat_f32[2] = getFilterOmega();            // -> MPU Gyro

    /* Increment integrator state:
     * Z(k+1) = Z(k) + T*Cr*X(k) + T*Dr*U(k);  -LQR
     * Z(k+1) = Z(k) + T*Cr*(X(k)' - xStar);   -MPC
     * -> Simplified! -> Zk + T*X(k)[x'] + 0
     */ 
    
    ctrl_xHat_f32[3] = ctrl_xHat_f32[3] + (T * ctrl_xHat_f32[1]);
    /* Compute f vector */
    arm_mat_mult_f32(&ctrl_Gamma,&ctrl_xHat,&ctrl_f);

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
    //printvector (CTRL_N_STATE, ctrl_xHat_f32, "x");
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
