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
#include "../lib/Sensors.h"
#include "../lib/Encoder.h"
#include "../lib/rotor.h"
#include "../lib/Navio/Common/Util.h"  // Navio Utility
#include "../lib/Navio/Navio2/PWM.h"   // Navio PWM output
#include "cmath"
#include "iostream"
/*****************************************************************************************
Global variables
******************************************************************************************/
#define _MOTOR_FRONT 0                // Channel# for front motor
#define _MOTOR_RIGHT 1                // Channel# for right motor
#define _MOTOR_RARE 2                 // Channel# for rare motor
#define _MOTOR_LEFT 3                 // Channel# for left motor

#define _PWM_MIN 0.85                 // PWM duty cycle in mS
#define _PWM_MAX 2.0                  // PWM duty cycle in mS
#define _FREQ   50                    // PWM period in Hz

#define _PWM_OFFSET 0.15              // PWM offset value
#define _PWM_SCALE_MOTOR_FRONT 1.03   // PWM scale value for front motor
#define _PWM_SCALE_MOTOR_RIGHT 1      // PWM scale value for right motor
#define _PWM_SCALE_MOTOR_RARE 0.98    // PWM scale value for rare motor
#define _PWM_SCALE_MOTOR_LEFT 1       // PWM scale value for left motor

/*****************************************************************************************
Functions prototype
******************************************************************************************/

class Navio{
public:
  Navio(){
    _ch[0] = _MOTOR_FRONT;
    _ch[1] = _MOTOR_RIGHT;
    _ch[2] = _MOTOR_RARE;
    _ch[3] = _MOTOR_LEFT;
    _scale[0] = _PWM_SCALE_MOTOR_FRONT;
    _scale[1] = _PWM_SCALE_MOTOR_RIGHT;
    _scale[2] = _PWM_SCALE_MOTOR_RARE;
    _scale[3] = _PWM_SCALE_MOTOR_LEFT;
  }
  ~Navio(){}
  /*****************************************************************************************
   initializePWM: Initialize PWM object used in Navio2 board
  *****************************************************************************************/
  void initializePWM(float init_val = _PWM_MIN) {
    for (int i=0; i<4; i++){
      // initialize pwm channels
      _pwm->init(_ch[i]);
      // set period to _FREQ = 50;
      _pwm->set_period(_ch[i], _FREQ);
      // set initiale duty cycle to minimum == motor off
      _pwm->set_duty_cycle(_ch[i], init_val);
      // enable pwm channels
      _pwm->enable(_ch[i]);
    }
  }
  /**************************************************************************************************
   du2motor: map du to PWM and send signal to motors
   **************************************************************************************************/
  void map(float du[4], float du_min[4], float du_max[4], float uPWM[4]) {
    // apply saturation for du
    for (int i=0; i<4; i++)
      du[i] = sat(du[i], du_min[i], du_max[i]);

    float dz = du[0] / 4.0;
    float dr = du[1] / 2.0;
    float dp = du[2] / 2.0;
    float dw = du[3] / 4.0;

    // du to PWM
    uPWM[0] = dz - dp - dw;
    uPWM[1] = dz - dr + dw;
    uPWM[2] = dz + dp - dw;
    uPWM[3] = dz + dr + dw;
  }
  /**************************************************************************************************
   toMotor: map du to PWM and send signal to motors
   **************************************************************************************************/
  void toMotor(float du[4], float du_min[4], float du_max[4], float dt) {
    float uPWM[4];

    // apply mapping
    map(du, du_min, du_max, uPWM);

    // rotor control
    float duty[4];
    for (int i=0; i<4 ; i++)
      duty[i] = _rotors[i].update(uPWM[i], dt);

    // send PWM duty cycle
    send(duty);
  }
  /*****************************************************************************************
   setFullPWM: send maximum PWM signals to motor
  *****************************************************************************************/
  void setFullPWM() {
    // set PWM duty
    for (int i=0; i<4; i++)
      _pwm->set_duty_cycle(_ch[i], _PWM_MAX);
  }
  /*****************************************************************************************
   setOffPWM: send minimum PWM signals to motor
  *****************************************************************************************/
  void setOffPWM() {
    // set PWM duty
    for (int i=0; i<4; i++)
      _pwm->set_duty_cycle(_ch[i], _PWM_MIN);
  }
  /*****************************************************************************************
   setPWMADuty: send PWM signal to motor
  *****************************************************************************************/
  void send(float uPWM[4]) {
    // set PWM duty
    for (int i=0; i<4; i++){
      // add minmum PWM value and apply saturation for PWM
      float tmp = sat(uPWM[i]*_scale[i] + _PWM_MIN + _PWM_OFFSET, _PWM_MIN, _PWM_MAX);
      // set PWM duty
      _pwm->set_duty_cycle(_ch[i], tmp);
    }
  }

private:
  Rotor _rotors[4];
  PWM* _pwm;
  float _ch[4];
  float _scale[4];

  /*****************************************************************************************
   sat: apply saturation
  *****************************************************************************************/
  float sat(float x, float lower, float upper) {
    if (x <= lower)
      x = lower;
    else if (x >= upper)
      x = upper;
    return x;
  }
};
#endif // NAVIO_INTERFACE


