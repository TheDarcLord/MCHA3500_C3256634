#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include "data_logging.h"

uint16_t logCount;
osTimerId_t _dataLogID;
void log_potentiometer(void *argument);
void logging_init(void);
void potentiometer_logging_start(void);
void potentiometer_logging_stop(void);

void log_potentiometer(void *argument) {
    UNUSED(argument);
    const float POT_VMAX = 3.3;
    // TICK = 1m
    float readPotVoltage = ((pot_get_value() * POT_VMAX) / 4095.0);
    
    /* TODO: 
        Print the sample time and potentiometer voltage to,
        the serial terminal in the format -> 
        [time],[voltage]
    */
    printf("%f, %f\n", (float) logCount/100.0, readPotVoltage); // Actually want 'Sample Time'
    logCount ++;
    /* TODO: Increment log count */

    /* TODO: Stop logging once 2 seconds is reached (Complete this once you have created the stop function
    in the next step) */
    if(logCount > 200) {
        potentiometer_logging_stop();
    }
}

void logging_init(void) {
    /* TODO: Initialise timer for use with pendulum data logging */
    _dataLogID = osTimerNew(log_potentiometer, osTimerPeriodic, (void *)5, NULL);
}

void potentiometer_logging_start(void) {
    /* TODO: Reset the log counter */
    logCount = 0;
    /* TODO: Start data logging timer at 100Hz */
    osTimerStart(_dataLogID, 10U);
}

void potentiometer_logging_stop(void) {
    /* TODO: Stop data logging timer */
    osTimerStop(_dataLogID);
}