#include "IMU.h"

const float EIGHTG = 78.48;
const float MIDPNT = 19.00;
const float RANGE  = 65553.00;
TM_MPU6050_t MPU_6050;          // Data Struct -> Holds X, Y & Z etc.
/* TM_MPU6050_ReadAll fills this STRUCT
    uint8_t 	Address
    float 	    Gyro_Mult
    float 	    Acce_Mult
    int16_t 	Accelerometer_X
    int16_t 	Accelerometer_Y
    int16_t 	Accelerometer_Z
    int16_t 	Gyroscope_X
    int16_t 	Gyroscope_Y
    int16_t 	Gyroscope_Z
    float 	    Temperature
*/
static uint8_t _is_init = 0;
void IMU_init(void);
void IMU_read(void);
float get_accY(void);

void IMU_init(void) {
    /* Initialise IMU with AD0 LOW, acceleration sensitivity +-4g, gyroscope +-250 deg/s */
    if (!_is_init)
    {
        TM_MPU6050_Result_t result = TM_MPU6050_Result_Error; //Initialise to error state. Result is overwritten in following line.
        if ((result = TM_MPU6050_Init(&MPU_6050, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_250s)) == TM_MPU6050_Result_Ok) {
             _is_init = 1;
        } else {
            printf("Error initialising MPU6050! Error code = %d\n", result);
        }
    }
}

void IMU_read(void)
{
    TM_MPU6050_ReadAll(&MPU_6050);
}

float get_accY(void) {
    float data = (float) MPU_6050.Accelerometer_Y;
    /*  
        int16 t can store the values −32786 to 32767 and 1g = 9.81ms−2. 
        Consider how you can convert the return value at a sensitivity of ±4g.
        RANGE: 65,553 MID: 32,776.5
        -4g = -32786
        0g = -9.5
        +4g = 32767
        4g = 4*9.81 = 39.24

              X 

        4g = Xmax 
        0g = 
        -4g = Xmin
    */
    if(data > -9.5) {
        return (float) data * (EIGHTG/(RANGE - MIDPNT));
    } else if(data < -9.5) {
        return (float) data * (EIGHTG/(RANGE + MIDPNT));
    } else {
        // Not Possible but clears up compiler errors
        return (float) 0.0;
    }
}