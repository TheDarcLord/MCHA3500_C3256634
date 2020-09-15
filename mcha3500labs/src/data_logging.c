#include "data_logging.h"

uint16_t logCount;
osTimerId_t _dataLogID;
static osTimerAttr_t _dataLogATTR = {
    .name = "datalog"
};
static void (*log_function)(void);              // Variable to point to function we intend to log
static void log_pointer(void *argument);
static void log_potentiometer(void *argument);
static void log_imu_pot(void *argument);
void logging_init(void);
void data_logging_start(SENSOR X);
void data_logging_stop(void);

void logging_init(void) {
    /* Initialise timer for use with generic data logging */
    _dataLogID = osTimerNew(log_pointer, osTimerPeriodic, NULL, &_dataLogATTR);
}

static void log_pointer(void *argument) {
    UNUSED(argument);
    // Calls function pointed to by *log_function()
    (*log_function)();
}

void data_logging_start(SENSOR X) {
    /* Pass a function to log_function to LOG */
    switch(X) {
        case(POT):
            log_function = &log_potentiometer;
        break;
        case(IMU_POT):
            log_function = &log_imu_pot;
        break;
        default:
            log_function = &log_potentiometer;
        break;
    }
    /* Reset the log counter */
    logCount = 0;
    /* Start data logging timer at 100Hz */
    osTimerStart(_dataLogID, 10U);
}


void data_logging_stop(void) {
    /* Stop data logging timer */
    osTimerStop(_dataLogID);
}

/* LOGGING EACH SENSOR ELEMENT */
static void log_potentiometer(void *argument) {
    UNUSED(argument);   // TICK = 1ms
    /*
        Print the sample time and potentiometer voltage to,
        the serial terminal in the format -> 
        [time],[voltage]
    */
    printf("%f, %f\n", (float) logCount/100.0, get_pot_voltage()); // Actually want 'Sample Time'
    logCount ++;
    /* Stop logging once 2 seconds is reached (Complete this once you have created the stop function
    in the next step) */
    if(logCount > 200) {
        data_logging_stop();
    }
}

static void log_imu_pot(void *argument) {
    UNUSED(argument);

    IMU_read();

    /* TODO: Print the time, accelerometer angle, gyro angular velocity and pot voltage values to the
    serial terminal in the format %f,%f,%f,%f\n */

    printf("%f, %f, %f, %f\n", (float) logCount/100.0, get_angle(1), get_gyroX() ,get_pot_voltage()); // Actually want 'Sample Time'
    /* Increment log count */
    logCount ++;
    
    /* Stop logging once 5 seconds is reached (Complete this once you have created the stop function
    in the next step) */
    if(logCount > 500) {
        data_logging_stop();
    }
}
