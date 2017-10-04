/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef NAVIO_INTERFACE
#define NAVIO_INTERFACE

/*****************************************************************************************
Header files
******************************************************************************************/
#include "Navio/MPU9250.h"     // Navio mpu sensor
#include "Navio/LSM9DS1.h"     // Navio lsm sensor
#include "Navio/Util.h"                 // Navio Utility
#include "Navio/PWM.h"             // Navio PWM output
#include "lib/AHRS.hpp"             // Navio Mahony AHRS
#include <stdio.h>                       // printf
#include <cmath>                        // math function and variables
#include "lib/complementary_filter.h"

/*****************************************************************************************
Global variables
******************************************************************************************/
#define _G_SI 9.80665
#define _MOTOR1    0                // CH0 front motor
#define _MOTOR2    1                // CH1 right motor
#define _MOTOR3    2                // CH2 back motor
#define _MOTOR4    3                // CH3 left motor
#define _SERVO_MIN  1.0        // in mS
#define _SERVO_MAX  2.0       // in mS
#define _FREQ       50                 // in Hz

/*****************************************************************************************
Define structures
******************************************************************************************/
struct imuStruct{
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float r, p, w;
    float qx, qy, qz, qw;
    float mag_offset[3];
    float mag_scale[3];
};

/*****************************************************************************************
Functions prototype
******************************************************************************************/
InertialSensor* imuSetup(AHRS *ahrs, char *sensor_name, imuStruct* imu);
void getIMU(InertialSensor *ins, imuStruct* imu);
void doAHRS(AHRS *ahrs, imuStruct* imu, float dt);
void gyroCalibrate(InertialSensor *ins, AHRS *ahrs);
void magCalibrate(InertialSensor *ins,float mag_offset[3], float mag_scale[3]);
void setPWMDuty(PWM* pwm, float uPWM[4]);
void initializePWM(PWM* pwm);
float sat(float x, float upper, float lower);
void doComplementaryFilter(imu_tools::ComplementaryFilter* comp_filter, imuStruct* imu, float dt);
void Quaternion2Euler(float& roll, float& pitch, float& yaw, float q0, float q1, float q2, float q3);
#endif // NAVIO_INTERFACE

