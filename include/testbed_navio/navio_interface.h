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
#include "../lib/rotor.h"                           //

/**************************************************************************************************
Global variables
**************************************************************************************************/
#define _PWM_MIN 0.85                               // PWM duty cycle in mS
#define _PWM_MAX 2.00                               // PWM duty cycle in mS
#define _PWM_FREQ 50                                // PWM period in Hz
namespace navio_interface {
int ch[4] = {0, 1, 2, 3};                           // Channel# {front, right, back, left}
float scale[4] = {1.03, 1.0, 0.98, 1.0};            // Scale value for servos
float offset = 0.15;                                // Offset value for servos
}
/**************************************************************************************************
Class
**************************************************************************************************/
class NavioInterface{
public:
  NavioInterface(){}
  ~NavioInterface(){}
  /************************************************************************************************
     initialize: Initialize PWM object used in Navio2 board
  ************************************************************************************************/
  void initialize() {
    for (int i=0; i<4; i++){
      // initialize pwm channels
      _pwm->init(navio_interface::ch[i]);
      // set period to _FREQ = 50;
      _pwm->set_period(navio_interface::ch[i], _PWM_FREQ);
      // set initiale duty cycle to minimum == motor off
      _pwm->set_duty_cycle(navio_interface::ch[i], _PWM_MIN);
      // enable pwm channels
      _pwm->enable(navio_interface::ch[i]);
      // create a rotor object for each channel
      _rotors[i] = Rotor();
    }
  }
  /************************************************************************************************
     send: map du to PWM and send signal to motors
  ************************************************************************************************/
  void send(float du[4], float du_min[4], float du_max[4]) {
    // map du to PWM
    float duty[4];
    map(du, duty, du_min, du_max);

    // send PWM duty cycle
    setPWMDuty(duty);
  }
  /**************************************************************************************************
     sendAndControl: map du to PWM and send signal to motors and apply pid control
  **************************************************************************************************/
  void sendAndControl(float du[4], float du_min[4], float du_max[4], float dt) {
    // map du to PWM
    float duty[4];
    map(du, duty, du_min, du_max);

    // rotor control
    float adjPWM[4];
    for (int i=0; i<4 ; i++)
      adjPWM[i] = _rotors[i].update(duty[i], dt);

    // send PWM duty cycle
    setPWMDuty(adjPWM);
  }
  /************************************************************************************************
     setMaxPWM: send maximum PWM signals to all motors
  ************************************************************************************************/
  void setMaxPWM() {
    float max[4] = {_PWM_MAX, _PWM_MAX, _PWM_MAX, _PWM_MAX};
    float min[4] = {_PWM_MIN, _PWM_MIN, _PWM_MIN, _PWM_MIN};
    // set PWM duty cycle to maximum
    send(max, min, max);
  }
  /************************************************************************************************
     setMinPWM: send minimum PWM signals to all motors
  ************************************************************************************************/
  void setMinPWM() {
    float max[4] = {_PWM_MAX, _PWM_MAX, _PWM_MAX, _PWM_MAX};
    float min[4] = {_PWM_MIN, _PWM_MIN, _PWM_MIN, _PWM_MIN};
    // set PWM duty cycle to maximum
    send(min, min, max);
  }

private:
  PWM *_pwm;
  Rotor _rotors[4];
  /************************************************************************************************
   setPWMADuty: send PWM signal to motor
  ************************************************************************************************/
  void setPWMDuty(float duty[4]) {
    for (int i=0; i<4; i++){
      // add minmum PWM value and apply saturation for PWM
      float tmp = sat(duty[i] * navio_interface::scale[i] + navio_interface::offset + _PWM_MIN,
                      _PWM_MIN, _PWM_MAX);
      // set PWM duty
      _pwm->set_duty_cycle(navio_interface::ch[i], tmp);
    }
  }
  /************************************************************************************************
   sat: apply saturation
  ************************************************************************************************/
  float sat(float u, float min, float max){
    float y;
    if (u >= max)
      y = max;
    else if (u <= min)
      y = min;
    else
      y = u;
    return y;
  }

  void map(float du[4], float duty[4], float du_min[4], float du_max[4]){
    // apply saturation for du
    float dz = sat(du[0], du_min[0], du_max[0]) / 4.0;
    float dr = sat(du[1], du_min[1], du_max[1]) / 2.0;
    float dp = sat(du[2], du_min[2], du_max[2]) / 2.0;
    float dy = sat(du[3], du_min[3], du_max[3]) / 4.0;

    // du to PWM
    duty[0] = dz - dp + dy;
    duty[1] = dz - dr - dy;
    duty[2] = dz + dp + dy;
    duty[3] = dz + dr - dy;
  }
};
#endif // NAVIO_INTERFACE

