/*
 * File:   navio_interface.h
 * Author: Bara Emran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef NAVIO_INTERFACE
#define NAVIO_INTERFACE

/*****************************************************************************************
Header files
******************************************************************************************/
#include "lib/Sensors.h"
#include "lib/Encoder.h"
#include "lib/Navio/Common/Util.h"  // Navio Utility
#include "lib/Navio/Navio2/PWM.h"   // Navio PWM output
#include "cmath"
/*****************************************************************************************
Global variables
******************************************************************************************/
#define _MOTOR1 0       // CH0 front motor
#define _MOTOR2 1       // CH1 right motor
#define _MOTOR3 2       // CH2 back motor
#define _MOTOR4 3       // CH3 left motor

#define _SERVO_MIN 1.0  // PWM duty cycle in mS
#define _SERVO_MAX 2.0  // PWM duty cycle in mS
#define _FREQ   50      // PWM period in Hz

/*****************************************************************************************
Functions prototype
******************************************************************************************/
void initializePWM(PWM* pwm);
void du2motor(PWM* pwm, float du[4], float offset[4], float du_min[4], float du_max[4]);
void setPWMDuty(PWM* pwm, float uPWM[4]);

float sat(float x, float lower, float upper);
void Quaternion2Euler(float& roll, float& pitch, float& yaw, float q0, float q1, float q2, float q3);

#endif // NAVIO_INTERFACE

