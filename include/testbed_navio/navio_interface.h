/*
 * File:   navio_interface.h
 * Author: Bara Emran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef NAVIO_INTERFACE
#define NAVIO_INTERFACE
/**************************************************************************************************
Header files
**************************************************************************************************/
#include "../lib/Navio/Common/Util.h"               // Navio Utility
#include "../lib/Navio/Navio2/PWM.h"                // Navio PWM output
#include "iostream"
/**************************************************************************************************
Global variables
**************************************************************************************************/
namespace navio_interface {
#define _PWM_MIN 0.85                               // PWM duty cycle in mS
#define _PWM_MAX 2.00                               // PWM duty cycle in mS
#define _PWM_FREQ 50                                // PWM period in Hz
int _CH_SERVO[4] = {0, 1, 2, 3};                    // Channel# {front, right, back, left}
float _SCALE_SERVO[4] = {1.03, 1.0, 0.98, 1.0};     // Scale value for servos
float _OFFSET_SERVO[4] = {0.15, 0.15, 0.15, 0.15};  // Offset value for servos
}
/**************************************************************************************************
Functions prototype
**************************************************************************************************/
void initializePWM(PWM* pwm);
void sendToMotors(PWM* pwm, float du[4], float du_min[4], float du_max[4]);
void setMaxPWM(PWM* pwm);
void setMinPWM(PWM* pwm);

#endif // NAVIO_INTERFACE

