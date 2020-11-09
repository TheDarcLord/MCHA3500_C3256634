#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <inttypes.h>           // For PRIxx and SCNxx macros
#include "stm32f4xx_hal.h"      // to import UNUSED() macro
#include "cmd_line_buffer.h"
#include "potentiometer.h"
#include "data_logging.h"
#include "motor.h"
#include "encoder.h"
#include "ammeter.h"
#include "heartbeat_cmd.h"
#include "motorControl.h"
void cmd_parse(char *);

// Type for each command table entry
typedef struct
{
    void (*func)(int argc, char *argv[]);   // Command function pointer
    const char * cmd;                       // Command name
    const char * args;                      // Command arguments syntax
    const char * help;                      // Command description
} CMD_T;

#endif
