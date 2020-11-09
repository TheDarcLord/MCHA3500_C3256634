#include "motorControl.h"

uint16_t motorCount;
osTimerId_t _motorCtrlID;
static void ctrlMotor(void *argument);

static float VOLTAGE = 0.0;
static float CURRENT = 0.0;
static float TORQUE = 0.0;
static float _error = 0;
static float Tf(float);
#define KI  0.00
#define KP  0.001
#define N   18.75
#define mKi 0.0055
#define BR  0.5947e-9
#define BF  0.9033e-9

void ctrlMotor(void *argument) {
    printf("Va: %f \n", VOLTAGE);
    UNUSED(argument);
    float Wa = encoder_pop_count();

    CURRENT = ((TORQUE/N) + Tf(Wa)) / mKi;
    
    float x = ammeter_get_value();
    printf("Ia: %f \n", x);
    float Ra = 6.6002;
    float Kw = 0.0055;
    // Vin = Ra*Ia + KwWa + U
    float Ihat = x - CURRENT;
    float U = (KP*(Ihat) + KI*_error);
    VOLTAGE = (Ra*CURRENT) + (Kw*Wa) + U;
    // (Z+1) = 0.99*_error + 100*U;
    if(fabs(VOLTAGE) < 11) {
        motor_set_voltage(VOLTAGE);
    } else {
        printf("VOLTAGE LIMIT EXCEEDED!!! \n");
        motor_set_voltage(0.0);
    }
    _error = 0.99*_error + 0.995*U;
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


void motor_set_torque(float t) {
    TORQUE = t;
}
