#include "motorControl.h"

uint16_t motorCount;
osTimerId_t _motorCtrlID;
static void ctrlMotor(void *argument);

static float VOLTAGE =  0.0;
static float CURRENT =  0.0;
static float TORQUE =   0.0;
static float OMEGAA =   0.0;
static float Tf(float);
static uint16_t SAFE =  0;
static float _error =   0;

#define KI  0.0
#define KP  0.01
#define mKi 0.0055
#define BR  4.4e-06
#define BF  4.3e-06

void ctrlMotor(void *argument) {
    UNUSED(argument);
    if(SAFE < 50) {
        TORQUE = getControl();
        OMEGAA  = encoder_pop_count();
        //printf("Va: %f \n", VOLTAGE);
        //printf("Ia RAW: %f \n", x);
        float x = getFilterCurrent();
        printf("Theta: %f\nTorque: %f\n",getFilterAngle(),TORQUE);
        // MOTOR SAFETY CHECK - LONGEVITY:
        // Should remain 25% below stall current: 5.5A
        // Should consume < 12W
        CURRENT = ((TORQUE/N) + Tf(OMEGAA)) / mKi;

        if(fabs(x) > 5 || (fabs(x)*fabs(VOLTAGE) > 11.8)) {
            SAFE++;
            //printf("Unsafe Ia: %f, Va: %f, Power: %f \n", fabs(x), fabs(VOLTAGE), fabs(x)*fabs(VOLTAGE));
        } else if((fabs(x)*fabs(VOLTAGE) > 12)) {
            ctrlMotor_stop();
            //printf("Unsafe POWER! \n");
        } else {
            SAFE = 0;
        }
        //printf("Ia FIL: %f \n", x);

        // Vin = Ra*Ia + KwWa + U
        float Ihat = x - CURRENT;
        float U = (KP*(Ihat) + KI*_error);
        // Ra & Kw defined in Kalman.h
        VOLTAGE = (Ra*CURRENT) + (Kw*OMEGAA) + U;
        
        if(fabs(VOLTAGE) < 12) {
            motor_set_voltage(VOLTAGE);
        } else {
            //printf("VOLTAGE LIMIT EXCEEDED!!! \n");
            if(VOLTAGE > 0) {
                motor_set_voltage(11.9);
            } else {
                motor_set_voltage(-11.9);
            }
        }
        // (Z+1) = 0.99*_error + 100*U;
        _error = 0.99*_error + 0.995*U;
        //printf("Err: %f \n", _error);
    } else {
        ctrlMotor_stop();
    }
}

float Tf(float omega) {
    if(omega > 0) {
        return BF*omega;
    } else if(omega < 0) {
        return BR*omega;
    } else {
        return 0;
    }
}

void ctrlMotor_init(void) {
    /* Initialise timer */
    _motorCtrlID = osTimerNew(ctrlMotor, osTimerPeriodic, (void *)5, NULL);
}

void ctrlMotor_start(void) {
    /* Start data logging timer at 100Hz */
    SAFE = 0;
    //printf("Motor Started");
    osTimerStart(_motorCtrlID, 10U);
    encoder_enable_interrupts();
    encoder_set_count(0);
}

void ctrlMotor_stop(void) {
    motor_set_voltage(0.0);
    osTimerStop(_motorCtrlID);
    encoder_disable_interrupts();
    encoder_set_count(0);
}

void motor_set_torque(float t) {
    TORQUE = t;
}

float motor_get_current(void) {
    return CURRENT;
}

float motor_get_voltage(void) {
    return VOLTAGE;
}

float motor_get_velocity(void) {
    return OMEGAA;
}
