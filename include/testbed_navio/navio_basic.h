/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef NAVIO_BASIC
#define NAVIO_BASIC

/*****************************************************************************************
Header files
******************************************************************************************/
#include <signal.h>       // signal ctrl+c
#include <stdio.h>        // printf
#include <cstdlib>
#include <unistd.h>      // usleep
#include <pthread.h>   // create thread

#include "../Navio/MPU9250.h"      // Navio mpu sensor
#include "../Navio/LSM9DS1.h"     // Navio lsm sensor
#include "../lib/AHRS.hpp"              // Navio Mahony AHRS
#include "../Navio/Util.h"                 // Navio Utility
#include "../Navio/PWM.h"             // Navio PWM output
#include "../lib/SamplingTime.h"    // samplig time

/*****************************************************************************************
Global variables
******************************************************************************************/
#define _SENSOR_FREQ 500        // Sensor thread frequency in Hz
#define _CONTROL_FREQ 200     // Control thread frequency in Hz
#define G_SI 9.80665
#define _MOTOR1    0                // CH0 front motor
#define _MOTOR2    1                // CH1 right motor
#define _MOTOR3    2                // CH2 back motor
#define _MOTOR4    3                // CH3 left motor
#define _SERVO_MIN  1.0        // in mS
#define _SERVO_MAX  2.0       // in mS
#define _FREQ       50                 // in Hz
#define _MAX_ROLL   0.3
#define _MAX_PITCH  0.3
#define _MAX_YAW    0.6
#define _MAX_Thrust  0.7

using namespace std;
bool _CloseRequested = false;
pthread_t _Thread_Sensors;
pthread_t _Thread_Control;

/*****************************************************************************************
Define structures
******************************************************************************************/
struct imuStruct{
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float r, p, w;
};
struct dataStruct {
        float PWMval[4];
        float du[4];
        float encoderes[3];
        PWM pwm;
        imuStruct imu;
        InertialSensor *ins;
        AHRS ahrs;
        int argc;
        char** argv;
};

/*****************************************************************************************
Functions prototype
******************************************************************************************/
void *sensorsThread(void *data);
void *controlThread(void *data);
void ctrlCHandler(int signal);
InertialSensor* imuSetup(AHRS *ahrs, char *sensor_name);
void getIMU(InertialSensor *ins, AHRS *ahrs, imuStruct* imu, float dt);
void setPWMDuty(PWM* pwm, float uPWM[4]);
void du2motor(PWM* pwm, float du0,float du1,float du2,float du3);
float sat(float x, float upper, float lower);


#endif // NAVIO_BASIC

