/*
 * File:   navio_interface.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "navio_interface.h"

using namespace navio_interface;
void setPWMDuty(PWM* pwm, float duty[4]);

/**************************************************************************************************
 initializePWM: Initialize PWM object used in Navio2 board
**************************************************************************************************/
void initializePWM(PWM *pwm) {
  for (int i=0; i<4; i++){
    // initialize pwm channels
    pwm->init(_CH_SERVO[i]);
    // set period to _FREQ = 50;
    pwm->set_period(_CH_SERVO[i], _PWM_FREQ);
    // set initiale duty cycle to minimum == motor off
    pwm->set_duty_cycle(_CH_SERVO[i], _PWM_MIN);
    // enable pwm channels
    pwm->enable(_CH_SERVO[i]);
  }
}
/**************************************************************************************************
 du2motor: map du to PWM and send signal to motors
 *************************************************************************************************/
void sendToMotors(PWM* pwm, float du[4], float du_min[4], float du_max[4]) {
  // apply saturation for du
  float dz = sat(du[0], du_min[0], du_max[0]) / 4.0;
  float dr = sat(du[1], du_min[1], du_max[1]) / 2.0;
  float dp = sat(du[2], du_min[2], du_max[2]) / 2.0;
  float dy = sat(du[3], du_min[3], du_max[3]) / 4.0;

  // du to PWM
  float duty[4];
  duty[0] = dz - dp + dy;
  duty[1] = dz - dr - dy;
  duty[2] = dz + dp + dy;
  duty[3] = dz + dr - dy;

  // send PWM duty cycle
  setPWMDuty(pwm, duty);
}
/**************************************************************************************************
 setPWMADuty: send PWM signal to motor
**************************************************************************************************/
void setPWMDuty(PWM* pwm, float duty[4]) {
  for (int i=0; i<4; i++){
    // add minmum PWM value and apply saturation for PWM
    duty = duty * _SCALE_SERVO[i] + _OFFSET_SERVO[i] + _PWM_MIN;
    if (duty > _PWM_MAX)
      duty = _PWM_MAX;
    if (duty < _PWM_MIN)
      duty = _PWM_MIN;
    // set PWM duty
    pwm->set_duty_cycle(_CH_SERVO[i], duty);
  }
}
/**************************************************************************************************
 setMaxPWM: send maximum PWM signals to all motors
**************************************************************************************************/
void setMaxPWM(PWM* pwm) {
  // set PWM duty cycle to maximum
  for (int i=0; i<4; i++){
    pwm->set_duty_cycle(_CH_SERVO[i], _PWM_MAX);
  }
}
/**************************************************************************************************
 setMinPWM: send minimum PWM signals to all motors
**************************************************************************************************/
void setMinPWM(PWM* pwm) {
  // set PWM duty cycle to minimum
  for (int i=0; i<4; i++){
    pwm->set_duty_cycle(_CH_SERVO[i], _PWM_MIN);
  }
}
