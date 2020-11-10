#include "motorControl.h"

uint16_t motorCount;
osTimerId_t _motorCtrlID;
static void ctrlMotor(void *argument);

static float VOLTAGE = 0.0;
static float CURRENT = 0.0;
static float _error = 0;
static float OMEGAA = 0.0;

#define KI  0.00
#define KP  0.001

void ctrlMotor(void *argument) {
    
    UNUSED(argument);
    OMEGAA = encoder_pop_count();
    float x = ammeter_get_value();
    //printf("Va: %f \n", VOLTAGE);
    //printf("Ia RAW: %f \n", x);
    x = runKF(x, VOLTAGE, OMEGAA);
    //printf("Ia FIL: %f \n", x);

    // Vin = Ra*Ia + KwWa + U
    float Ihat = x - CURRENT;
    float U = (KP*(Ihat) + KI*_error);
    // Ra & Kw defined in Kalman.h
    VOLTAGE = (Ra*CURRENT) + (Kw*OMEGAA) + U;
    // (Z+1) = 0.99*_error + 100*U;
    if(fabs(VOLTAGE) < 11) {
        motor_set_voltage(VOLTAGE);
    } else {
        //printf("VOLTAGE LIMIT EXCEEDED!!! \n");
        motor_set_voltage(0.0);
    }
    _error = 0.99*_error + 0.995*U;
}

void ctrlMotor_init(void) {
    /* Initialise timer */
    _motorCtrlID = osTimerNew(ctrlMotor, osTimerPeriodic, (void *)5, NULL);
}

void ctrlMotor_start(void) {
    /* Start data logging timer at 100Hz */
    osTimerStart(_motorCtrlID, 10U);
    encoder_enable_interrupts();
    encoder_set_count(0);
}

void ctrlMotor_stop(void) {
    /* TODO: Stop data logging timer */
    osTimerStop(_motorCtrlID);
    encoder_disable_interrupts();
    encoder_set_count(0);
}


void motor_set_current(float c) {
    CURRENT = c;
}

float motor_get_velocity(void) {
    return OMEGAA;
}
