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
    0.0,    //  ω
    0.0,    //  θ
    0.0,    //  b...
    0.0     //  I
};

static float ukd[KALMAN_INPUTS] = {
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
    1.0,    0.0,   0.0,   0.0,
    0.0,    1.0,   0.0,   0.0,
    0.0,    0.0,   1.0,   0.0,
    0.0,    0.0,   0.0,   1.0
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

// Core Matrices
arm_matrix_instance_f32 A   = {KALMAN_STATES, KALMAN_STATES,        (float32_t*)Ad};   // 4x4
arm_matrix_instance_f32 B   = {KALMAN_STATES, KALMAN_INPUTS,        (float32_t*)Bd};   // 4x2
arm_matrix_instance_f32 C   = {SENSOR_OUTPUT, KALMAN_STATES,        (float32_t*)Cd};   // 3x4
arm_matrix_instance_f32 R   = {SENSOR_OUTPUT, SENSOR_OUTPUT,        (float32_t*)Rd};   // 3x3
arm_matrix_instance_f32 Q   = {KALMAN_STATES, KALMAN_STATES,        (float32_t*)Qd};   // 4x4
arm_matrix_instance_f32 EYE  = {KALMAN_STATES, KALMAN_STATES,       (float32_t*)eyed};  // 4x4
// Filter Matrices
arm_matrix_instance_f32 Pm  = {KALMAN_STATES, KALMAN_STATES,        (float32_t*)Pmd};   // 4x4
arm_matrix_instance_f32 Pp  = {KALMAN_STATES, KALMAN_STATES,        (float32_t*)Ppd};   // 4x4
arm_matrix_instance_f32 Kk  = {KALMAN_STATES, SENSOR_OUTPUT,        (float32_t*)Kkd};   // 4x3
// Vectors
arm_matrix_instance_f32 uk  = {KALMAN_INPUTS, 1,                    (float32_t*)ukd};  // 2x1
arm_matrix_instance_f32 xm  = {KALMAN_STATES, 1,                    (float32_t*)xmd};  // 4x1
arm_matrix_instance_f32 xp  = {KALMAN_STATES, 1,                    (float32_t*)xpd};  // 4x1
arm_matrix_instance_f32 yp  = {SENSOR_OUTPUT, 1,                    (float32_t*)ypd};  // 3x1
arm_matrix_instance_f32 yi  = {SENSOR_OUTPUT, 1,                    (float32_t*)yid};  // 3x1
// Temp Vectors
arm_matrix_instance_f32 temp41a = {KALMAN_STATES, 1,                (float32_t*)temp41d};
arm_matrix_instance_f32 temp41b = {KALMAN_STATES, 1,                (float32_t*)temp41e};
arm_matrix_instance_f32 temp31a = {SENSOR_OUTPUT, 1,                (float32_t*)temp31d};
arm_matrix_instance_f32 temp31b = {SENSOR_OUTPUT, 1,                (float32_t*)temp31e};
// Temp Squares
arm_matrix_instance_f32 temp33a = {SENSOR_OUTPUT, KALMAN_STATES,    (float32_t*)temp33d};   // 3x3
arm_matrix_instance_f32 temp33b = {SENSOR_OUTPUT, KALMAN_STATES,    (float32_t*)temp33e};   // 3x4
arm_matrix_instance_f32 temp44a = {KALMAN_STATES, KALMAN_STATES,    (float32_t*)temp44d};   // 4x4
arm_matrix_instance_f32 temp44b = {KALMAN_STATES, KALMAN_STATES,    (float32_t*)temp44e};   // 4x4
arm_matrix_instance_f32 temp44c = {KALMAN_STATES, KALMAN_STATES,    (float32_t*)temp44f};   // 4x4
// Temp Rectangles
arm_matrix_instance_f32 temp34 =  {SENSOR_OUTPUT, KALMAN_STATES,    (float32_t*)temp34d};   // 3x4
arm_matrix_instance_f32 temp43a = {KALMAN_STATES, SENSOR_OUTPUT,    (float32_t*)temp43d};   // 4x3
arm_matrix_instance_f32 temp43b = {KALMAN_STATES, SENSOR_OUTPUT,    (float32_t*)temp43e};   // 4x3

void initKF(void) {
    // Core Matrices
    arm_mat_init_f32(&A, KALMAN_STATES, KALMAN_STATES, (float32_t*)Ad);
    arm_mat_init_f32(&B, KALMAN_STATES, KALMAN_INPUTS, (float32_t*)Bd);
    arm_mat_init_f32(&C, SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)Cd);
    arm_mat_init_f32(&R, SENSOR_OUTPUT, SENSOR_OUTPUT, (float32_t*)Rd);
    arm_mat_init_f32(&Q, KALMAN_STATES, KALMAN_STATES, (float32_t*)Qd);
    arm_mat_init_f32(&EYE, KALMAN_STATES, KALMAN_STATES, (float32_t*)eyed);
    // Filter Matrices
    arm_mat_init_f32(&Pm, KALMAN_STATES, KALMAN_STATES, (float32_t*)Pmd);
    arm_mat_init_f32(&Pp, KALMAN_STATES, KALMAN_STATES, (float32_t*)Ppd);
    arm_mat_init_f32(&Kk, KALMAN_STATES, SENSOR_OUTPUT, (float32_t*)Kkd);
    // Vectors 
    arm_mat_init_f32(&uk, KALMAN_INPUTS, 1, (float32_t*)ukd);
    arm_mat_init_f32(&xm, KALMAN_STATES, 1, (float32_t*)xmd);
    arm_mat_init_f32(&xp, KALMAN_STATES, 1, (float32_t*)xpd);
    arm_mat_init_f32(&yp, SENSOR_OUTPUT, 1, (float32_t*)ypd);
    arm_mat_init_f32(&yi, SENSOR_OUTPUT, 1, (float32_t*)yid);
    // Temp Vectors
    arm_mat_init_f32(&temp41a, KALMAN_STATES, 1, (float32_t*)temp41d);
    arm_mat_init_f32(&temp41b, KALMAN_STATES, 1, (float32_t*)temp41e);
    arm_mat_init_f32(&temp31a, SENSOR_OUTPUT, 1, (float32_t*)temp31d);
    arm_mat_init_f32(&temp31b, SENSOR_OUTPUT, 1, (float32_t*)temp31e);
    // Temp Squares
    arm_mat_init_f32(&temp33a, SENSOR_OUTPUT, SENSOR_OUTPUT, (float32_t*)temp33d);
    arm_mat_init_f32(&temp33b, SENSOR_OUTPUT, SENSOR_OUTPUT, (float32_t*)temp33e);
    arm_mat_init_f32(&temp44a, KALMAN_STATES, KALMAN_STATES, (float32_t*)temp44d);
    arm_mat_init_f32(&temp44b, KALMAN_STATES, KALMAN_STATES, (float32_t*)temp44e);
    arm_mat_init_f32(&temp44c, KALMAN_STATES, KALMAN_STATES, (float32_t*)temp44f);
    // Temp Rectangles
    arm_mat_init_f32(&temp34, SENSOR_OUTPUT, KALMAN_STATES, (float32_t*)temp34d);
    arm_mat_init_f32(&temp43a, KALMAN_STATES, SENSOR_OUTPUT, (float32_t*)temp43d);
    arm_mat_init_f32(&temp43b, KALMAN_STATES, SENSOR_OUTPUT, (float32_t*)temp43e);
}

void runKF(void) {
    // 1st, 2nd, Out
    /* PACK MEASUREMENT VECTOR */
    IMU_read();
    arm_mat_mult_f32(&C, &xm, &yi);
    // vk
    yid[0] = yid[0] + get_angle(RADIANS);
    yid[1] = yid[1] + get_gyroY();
    yid[2] = yid[2] + ammeter_get_value();
    //printf("%f omega\n%f theta\n%f I\n", yid[0],yid[1],yid[2]);
    
    ukd[0] = 0;
    ukd[1] = 0;
    // Uk too !

    /* Compute Kalman gain */
    arm_mat_mult_f32(&C,&Pm,&temp34);               // temp34 = C * Pm
    //printf("temp34:\n%f %f %f %f \n%f %f %f %f\n%f %f %f %f\n", temp34d[0], temp34d[1], temp34d[2], temp34d[3],temp34d[4], temp34d[5], temp34d[6], temp34d[7],temp34d[8], temp34d[9], temp34d[10], temp34d[11]);
    arm_mat_trans_f32(&C,&temp43a);                 // temp43 = C'
    //printf("temp43:\n%f %f %f \n%f %f %f \n%f %f %f \n%f %f %f \n", temp43d[0], temp43d[1], temp43d[2], temp43d[3],temp43d[4], temp43d[5], temp43d[6], temp43d[7],temp43d[8], temp43d[9], temp43d[10], temp43d[11]);
    arm_mat_mult_f32(&temp34,&temp43a,&temp33a);    // temp33a = C * PM * C'
    //printf("temp33:\n%f %f %f \n%f %f %f \n%f %f %f \n", temp33d[0], temp33d[1], temp33d[2], temp33d[3],temp33d[4], temp33d[5], temp33d[6], temp33d[7],temp33d[8]);
    arm_mat_add_f32(&temp33a,&R,&temp33b);          // temp33b = (C*PM*C' + R)
    arm_mat_inverse_f32(&temp33b,&temp33a);         // temp33a = (C*PM*C' + R)^-1
    arm_mat_mult_f32(&Pm,&temp43a,&temp43b);        // temp43b = Pm * C'
    arm_mat_mult_f32(&temp43b,&temp33a,&Kk);        // Kk = (Pm * C') * (C*PM*C' + R)^-1
    //printf("Kk:\n%f %f %f \n%f %f %f \n%f %f %f \n%f %f %f \n", Kkd[0], Kkd[1], Kkd[2], Kkd[3],Kkd[4], Kkd[5], Kkd[6], Kkd[7],Kkd[8], Kkd[9], Kkd[10], Kkd[11]);

    /* Compute corrected state estimate */
    arm_mat_mult_f32(&C,&xm,&temp31a);              // temp31a = C * xm
    //printf("temp31a:\n%f \n%f \n%f \n", temp31d[0], temp31d[1], temp31d[2]);
    arm_mat_sub_f32(&yi,&temp31a,&temp31b);         // temp31b = yi - C*xm
    //printf("temp31b:\n%f \n%f \n%f \n", temp31e[0], temp31e[1], temp31e[2]);
    arm_mat_mult_f32(&Kk,&temp31b,&temp41a);        // temp41a = KK * (yi - C*xm)
    //printf("Temp41a: \n%f \n%f \n%f \n%f \n", temp41d[0], temp41d[1], temp41d[2], temp41d[3]);
    arm_mat_add_f32(&xm,&temp41a,&xp);              // xp = xm + KK * (yi - C*xm)

    printf("XP: \n%f - Omega \n%f - Theta \n%f - B \n%f - Current \n", xpd[0], xpd[1], xpd[2], xpd[3]);

    /* Compute new measurement error covariance */
    arm_mat_mult_f32(&Kk,&C,&temp44a);              // temp44a = Kk*C
    //printf("temp44a: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44d[0], temp44d[1], temp44d[2], temp44d[3],temp44d[4], temp44d[5], temp44d[6], temp44d[7],temp44d[8], temp44d[9], temp44d[10], temp44d[11], temp44d[12], temp44d[13], temp44d[14], temp44d[15]);
    arm_mat_sub_f32(&EYE,&temp44a,&temp44b);        // temp44b = EYE - Kk*C
    //printf("temp44b: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44e[0], temp44e[1], temp44e[2], temp44e[3],temp44e[4], temp44e[5], temp44e[6], temp44e[7],temp44e[8], temp44e[9], temp44e[10], temp44e[11], temp44e[12], temp44e[13], temp44e[14], temp44e[15]);
    arm_mat_trans_f32(&temp44b,&temp44c);         // temp44c = (EYE - Kk*C)'
    //printf("temp44c: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44f[0], temp44f[1], temp44f[2], temp44f[3],temp44f[4], temp44f[5], temp44f[6], temp44f[7],temp44f[8], temp44f[9], temp44f[10], temp44f[11], temp44f[12], temp44f[13], temp44f[14], temp44f[15]);
    arm_mat_mult_f32(&temp44b,&Pm,&temp44a);        // temp44a = (EYE - Kk*C) * Pm
    //printf("temp44a: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44d[0], temp44d[1], temp44d[2], temp44d[3],temp44d[4], temp44d[5], temp44d[6], temp44d[7],temp44d[8], temp44d[9], temp44d[10], temp44d[11], temp44d[12], temp44d[13], temp44d[14], temp44d[15]);
    arm_mat_mult_f32(&temp44a,&temp44c,&temp44b);   // temp44b = (EYE - Kk*C) * Pm * (EYE - Kk*C)'
    //printf("temp44b: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44e[0], temp44e[1], temp44e[2], temp44e[3],temp44e[4], temp44e[5], temp44e[6], temp44e[7],temp44e[8], temp44e[9], temp44e[10], temp44e[11], temp44e[12], temp44e[13], temp44e[14], temp44e[15]);
    arm_mat_mult_f32(&Kk,&R,&temp43a);              // temp43a = Kk*R
    //printf("temp43:\n%f %f %f \n%f %f %f \n%f %f %f \n%f %f %f \n", temp43d[0], temp43d[1], temp43d[2], temp43d[3],temp43d[4], temp43d[5], temp43d[6], temp43d[7],temp43d[8], temp43d[9], temp43d[10], temp43d[11]);
    arm_mat_trans_f32(&Kk,&temp34);                 // temp34 = Kk'
    arm_mat_mult_f32(&temp43a,&temp34,&temp44a);    // temp44a = Kk*R*KK'
    //printf("temp44a: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44d[0], temp44d[1], temp44d[2], temp44d[3],temp44d[4], temp44d[5], temp44d[6], temp44d[7],temp44d[8], temp44d[9], temp44d[10], temp44d[11], temp44d[12], temp44d[13], temp44d[14], temp44d[15]);
    arm_mat_add_f32(&temp44b,&temp44a,&Pp);         // Pp = (EYE - Kk*C) * Pm * (EYE - Kk*C)' + Kk*R*KK'
    //printf("Pp: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", Ppd[0], Ppd[1], Ppd[2], Ppd[3],Ppd[4], Ppd[5], Ppd[6], Ppd[7],Ppd[8], Ppd[9], Ppd[10], Ppd[11], Ppd[12], Ppd[13], Ppd[14], Ppd[15]);
    
    /* Predict next state */
    arm_mat_mult_f32(&A,&xp,&temp41a);              // temp41a = A*xp
    //printf("A*XP: \n%f - Omega \n%f - Theta \n%f - B \n%f - Current \n", temp41d[0], temp41d[1], temp41d[2], temp41d[3]);
    arm_mat_mult_f32(&B,&uk,&temp41b);              // temp41b = B*uk
    //printf("B*uk: \n%f - Omega \n%f - Theta \n%f - B \n%f - Current \n", temp41e[0], temp41e[1], temp41e[2], temp41e[3]);
    arm_mat_add_f32(&temp41a,&temp41b,&xm);         // xm = A*xp + B*uk

    //printf("XM: \n%f - Omega \n%f - Theta \n%f - B \n%f - Current \n", xmd[0], xmd[1], xmd[2], xmd[3]);

    /* Compute prediction error covariance */
    arm_mat_mult_f32(&A,&Pp,&temp44a);              // temp44a = A*Pp
    //printf("temp44a: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44d[0], temp44d[1], temp44d[2], temp44d[3],temp44d[4], temp44d[5], temp44d[6], temp44d[7],temp44d[8], temp44d[9], temp44d[10], temp44d[11], temp44d[12], temp44d[13], temp44d[14], temp44d[15]);
    arm_mat_trans_f32(&A,&temp44b);                 // temp44b = A'
    arm_mat_mult_f32(&temp44a,&temp44b,&temp44c);   // temp44c = A*Pp*A'
    //printf("temp44c: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", temp44f[0], temp44f[1], temp44f[2], temp44f[3],temp44f[4], temp44f[5], temp44f[6], temp44f[7],temp44f[8], temp44f[9], temp44f[10], temp44f[11], temp44f[12], temp44f[13], temp44f[14], temp44f[15]);
    arm_mat_add_f32(&temp44c,&Q,&Pm);               // Pm = A*Pp*A' + Q
    //printf("Pm: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", Pmd[0], Pmd[1], Pmd[2], Pmd[3],Pmd[4], Pmd[5], Pmd[6], Pmd[7],Pmd[8], Pmd[9], Pmd[10], Pmd[11], Pmd[12], Pmd[13], Pmd[14], Pmd[15]);
}

float getFilteredOmega(void) {
    return xpd[0];
}

float getFilteredAngle(void) {
    return xpd[1];
}

float getFilteredCurrent(void) {
    return xpd[3];
}

