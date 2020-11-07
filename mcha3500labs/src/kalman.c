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

static float xpd[KALMAN_STATES] = {
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

static float Pmd[KALMAN_STATES*KALMAN_STATES] = {
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0
};

static const float eyed[KALMAN_STATES*KALMAN_STATES] = {
    1.0,   0.0,   0.0,   0.0,
    0.0,   1.0,   0.0,   0.0,
    0.0,   0.0,   1.0,   0.0,
    0.0,   0.0,   0.0,   1.0
};

static float Ppd[KALMAN_STATES*KALMAN_STATES] = {
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0,
    0.0,   0.0,   0.0,   0.0
};

static float Kkd[KALMAN_STATES*KALMAN_STATES] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0
};

static float temp41d[KALMAN_STATES*1] = {
    0.0, 
    0.0, 
    0.0,
    0.0
};

static float temp41e[KALMAN_STATES*1] = {
    0.0, 
    0.0, 
    0.0,
    0.0
};

static float temp31d[SENSOR_OUTPUT*1] = {
    0.0, 
    0.0, 
    0.0
};

static float temp31e[SENSOR_OUTPUT*1] = {
    0.0, 
    0.0, 
    0.0
};

static float temp33d[SENSOR_OUTPUT*SENSOR_OUTPUT] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
};

static float temp33e[SENSOR_OUTPUT*SENSOR_OUTPUT] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
};

static float temp34d[SENSOR_OUTPUT*KALMAN_STATES] = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
};

static float temp43d[SENSOR_OUTPUT*KALMAN_STATES] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0
};

static float temp43e[SENSOR_OUTPUT*KALMAN_STATES] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0
};

static float temp44d[KALMAN_STATES*KALMAN_STATES] = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
};

static float temp44e[KALMAN_STATES*KALMAN_STATES] = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
};

static float temp44f[KALMAN_STATES*KALMAN_STATES] = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
};


                               // ROWS        // Columns
arm_matrix_instance_f32 A   = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Ad};   // 4x4
arm_matrix_instance_f32 B   = {KALMAN_STATES, KALMAN_INPUTS, (float32_t*)Bd};   // 4x2
arm_matrix_instance_f32 C   = {SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)Cd};   // 3x4
arm_matrix_instance_f32 R   = {SENSOR_OUTPUT, SENSOR_OUTPUT, (float32_t*)Rd};   // 3x3
arm_matrix_instance_f32 Q   = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Qd};   // 4x4
arm_matrix_instance_f32 Pm  = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Pmd};   // 4x4
arm_matrix_instance_f32 Pp  = {KALMAN_STATES, KALMAN_STATES, (float32_t*)Ppd};   // 4x4
arm_matrix_instance_f32 Kk  = {KALMAN_STATES, SENSOR_OUTPUT, (float32_t*)Kkd};  // 4x4
arm_matrix_instance_f32 vk  = {SENSOR_OUTPUT, 1, (float32_t*)vkd};  // 3x1
arm_matrix_instance_f32 uk  = {KALMAN_INPUTS, 1, (float32_t*)ukd};  // 2x1
arm_matrix_instance_f32 xm  = {KALMAN_STATES, 1, (float32_t*)xmd};  // 4x1
arm_matrix_instance_f32 xp  = {KALMAN_STATES, 1, (float32_t*)xpd};  // 4x1
arm_matrix_instance_f32 yp  = {SENSOR_OUTPUT, 1, (float32_t*)ypd};  // 3x1
arm_matrix_instance_f32 yi  = {SENSOR_OUTPUT, 1, (float32_t*)yid};  // 3x1

arm_matrix_instance_f32 EYE  = {KALMAN_STATES, KALMAN_STATES, (float32_t*)eyed};  // 4x4

arm_matrix_instance_f32 temp41a = {KALMAN_STATES, 1, (float32_t*)temp41d};
arm_matrix_instance_f32 temp41b = {KALMAN_STATES, 1, (float32_t*)temp41e};
arm_matrix_instance_f32 temp31a = {SENSOR_OUTPUT, 1, (float32_t*)temp31d};
arm_matrix_instance_f32 temp31b = {SENSOR_OUTPUT, 1, (float32_t*)temp31e};
arm_matrix_instance_f32 temp33a = {SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)temp33d};   // 3x4
arm_matrix_instance_f32 temp33b = {SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)temp33e};   // 3x4
arm_matrix_instance_f32 temp34 = {SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)temp34d};   // 3x4
arm_matrix_instance_f32 temp44a = {KALMAN_STATES, KALMAN_STATES, (float32_t*)temp44d};   // 4x4
arm_matrix_instance_f32 temp44b = {KALMAN_STATES, KALMAN_STATES, (float32_t*)temp44e};   // 4x4
arm_matrix_instance_f32 temp44c = {KALMAN_STATES, KALMAN_STATES, (float32_t*)temp44f};   // 4x4
arm_matrix_instance_f32 temp43a = {KALMAN_STATES, SENSOR_OUTPUT, (float32_t*)temp43d};   // 3x4
arm_matrix_instance_f32 temp43b = {KALMAN_STATES, SENSOR_OUTPUT, (float32_t*)temp43e};   // 3x4

void runKF(void) {
    // 1st, 2nd, Out
    /* PACK MEASUREMENT VECTOR */
    arm_mat_mult_f32(&C, &xm, &yi);
    yid[0] = yid[0] + get_angle(RADIANS);
    yid[1] = yid[1] + get_gyroY();
    yid[2] = yid[2] + ammeter_get_value();
    // Uk too !

    /* Compute Kalman gain */
    arm_mat_mult_f32(&C,&Pm,&temp34);               // temp34 = C * Pm
    arm_mat_trans_f32(&C,&temp43a);                  // temp43 = C'
    arm_mat_mult_f32(&temp34,&temp43a,&temp33a);    // temp33a = C * PM * C' 
    arm_mat_add_f32(&temp33a,&R,&temp33b);          // temp33b = (C*PM*C' + R)
    arm_mat_inverse_f32(&temp33b,&temp33a);         // temp33a = (C*PM*C' + R)^-1
    arm_mat_mult_f32(&Pm,&temp43a,&temp43b);        // temp43b = Pm * C'
    arm_mat_mult_f32(&temp43b,&temp33a,&Kk);        // Kk = (Pm * C') * (C*PM*C' + R)^-1

    /* Compute corrected state estimate */
    arm_mat_mult_f32(&C,&xm,&temp31a);              // temp31a = C * xm
    arm_mat_sub_f32(&yi,&temp31a,&temp31b);         // temp31b = yi - C*xm
    arm_mat_mult_f32(&Kk,&temp31b,&temp41a);        // temp41a = KK * (yi - C*xm)
    arm_mat_add_f32(&xm,&temp41a,&xp);              // xp = xm + KK * (yi - C*xm)

    /* Compute new measurement error covariance */
    arm_mat_mult_f32(&Kk,&C,&temp44a);              // temp44a = Kk*C
    arm_mat_sub_f32(&EYE,&temp44a,&temp44b);        // temp44b = EYE - Kk*C
    arm_mat_inverse_f32(&temp44b,&temp44c);         // temp44c = (EYE - Kk*C)'
    arm_mat_mult_f32(&temp44b,&Pm,&temp44a);        // temp44a = (EYE - Kk*C) * Pm
    arm_mat_mult_f32(&temp44a,&temp44c,&temp44b);   // temp44b = (EYE - Kk*C) * Pm * (EYE - Kk*C)'
    arm_mat_mult_f32(&Kk,&R,&temp43a);              // temp43a = Kk*R
    arm_mat_trans_f32(&Kk,&temp34);                  // temp34 = Kk'
    arm_mat_mult_f32(&temp43a,&temp34,&temp44a);    // temp44a = Kk*R*KK'
    arm_mat_add_f32(&temp44b,&temp44a,&Pp);         // Pp = (EYE - Kk*C) * Pm * (EYE - Kk*C)' + Kk*R*KK'

    /* Predict next state */
    arm_mat_mult_f32(&A,&xp,&temp41a);               // temp41a = A*xp
    arm_mat_mult_f32(&B,&uk,&temp41b);               // temp41b = B*uk
    arm_mat_add_f32(&temp41a,&temp41b,&xm);         // xm = A*xp + B*uk

    /* Compute prediction error covariance */
    arm_mat_mult_f32(&A,&Pp,&temp44a);               // temp44a = A*Pp
    arm_mat_trans_f32(&A,&temp44b);                  // temp44b = A'
    arm_mat_mult_f32(&temp44a,&temp44b,&temp44c);    // temp44c = A*Pp*A'
    arm_mat_add_f32(&temp44c,&Q,&Pm);               // Pm = A*Pp*A' + Q
}