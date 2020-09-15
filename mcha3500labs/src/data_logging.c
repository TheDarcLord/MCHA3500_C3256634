#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include "data_logging.h"

uint16_t logCount;
osTimerId_t _dataLogID;
static osTimerAttr_t _dataLogATTR = {
    .name = "datalog"
};
static void (*log_function)(void);              // Variable to point to function we intend to log
static void log_pointer(void *argument);
static void log_potentiometer(void *argument);
void logging_init(void);
void data_logging_start(void);
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

void data_logging_start(void) {
    /* Pass a function to log_function to LOG */
    log_function = &log_potentiometer;
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
    UNUSED(argument);
    const float POT_VMAX = 3.3;
    // TICK = 1ms
    float readPotVoltage = ((pot_get_value() * POT_VMAX) / 4095.0);
    
    /*
        Print the sample time and potentiometer voltage to,
        the serial terminal in the format -> 
        [time],[voltage]
    */
    printf("%f, %f\n", (float) logCount/100.0, readPotVoltage); // Actually want 'Sample Time'
    /* Increment log count */
    logCount ++;
    

    /* Stop logging once 2 seconds is reached (Complete this once you have created the stop function
    in the next step) */
    if(logCount > 200) {
        data_logging_stop();
    }
}

