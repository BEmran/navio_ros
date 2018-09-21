/*
 * File:   navio_interface.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "navio_interface.h"

/*****************************************************************************************
 initializePWM: Initialize PWM object used in Navio2 board
*****************************************************************************************/
void initializePWM(PWM *pwm, bool tune) {
    // initialize pwm channels
    pwm->init(_MOTOR1);
    pwm->init(_MOTOR2);
    pwm->init(_MOTOR3);
    pwm->init(_MOTOR4);
    // set period to _FREQ = 50;
    pwm->set_period(_MOTOR1, _FREQ);
    pwm->set_period(_MOTOR2, _FREQ);
    pwm->set_period(_MOTOR3, _FREQ);
    pwm->set_period(_MOTOR4, _FREQ);
    // set initiale duty cycle to minimum == motor off
    pwm->set_duty_cycle(_MOTOR1, _SERVO_MIN);
    pwm->set_duty_cycle(_MOTOR2, _SERVO_MIN);
    pwm->set_duty_cycle(_MOTOR3, _SERVO_MIN);
    pwm->set_duty_cycle(_MOTOR4, _SERVO_MIN);
    // enable pwm channels
    pwm->enable(_MOTOR1);
    pwm->enable(_MOTOR2);
    pwm->enable(_MOTOR3);
    pwm->enable(_MOTOR4);
    if (tune){
      float high[4] = {1.0, 1.0, 1.0, 1.0};
      float low[4] = {0.0, 0.0, 0.0, 0.0};
      setPWMDuty(pwm, high);
      int x = 0;
      while (x == 0) {
          printf("Enter 1 to finish calibrate\n");
          std::cin >> x;
          sleep(1);
      }
      setPWMDuty(pwm, low);
    }
}

/**************************************************************************************************
 du2motor: map du to PWM and send signal to motors
 **************************************************************************************************/
void du2motor(PWM* pwm, float du[4], float offset[4], float du_min[4], float du_max[4]) {
    // apply saturation for du

    float dz = sat(du[0], du_min[0], du_max[0]) / 4.0;
    float dr = sat(du[1], du_min[1], du_max[1]) / 2.0;
    float dp = sat(du[2], du_min[2], du_max[2]) / 2.0;
    float dw = sat(du[3], du_min[3], du_max[3]) / 4.0;

    // du to PWM
    float uPWM[4];
    uPWM[0] = dz + dp - dw + offset[0];
    uPWM[1] = dz - dr + dw + offset[1];
    uPWM[2] = dz - dp - dw + offset[2];
    uPWM[3] = dz + dr + dw + offset[3];

    // send PWM duty cycle
    setPWMDuty(pwm, uPWM);
}

/*****************************************************************************************
 setPWMADuty: send PWM signal to motor
*****************************************************************************************/
void setPWMDuty(PWM* pwm, float uPWM[4]) {
    // add minmum PWM value and apply saturation for PWM
    uPWM[0] = sat(uPWM[0] + _SERVO_MIN, _SERVO_MIN, _SERVO_MAX);
    uPWM[1] = sat(uPWM[1] + _SERVO_MIN, _SERVO_MIN, _SERVO_MAX);
    uPWM[2] = sat(uPWM[2] + _SERVO_MIN, _SERVO_MIN, _SERVO_MAX);
    uPWM[3] = sat(uPWM[3] + _SERVO_MIN, _SERVO_MIN, _SERVO_MAX);

    // set PWM duty
    pwm->set_duty_cycle(_MOTOR1, uPWM[0]);
    pwm->set_duty_cycle(_MOTOR2, uPWM[1]);
    pwm->set_duty_cycle(_MOTOR3, uPWM[2]);
    pwm->set_duty_cycle(_MOTOR4, uPWM[3]);
}

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

/*****************************************************************************************
 Quaternion2Euler: convert Quaternion to Euler angles in rad
*****************************************************************************************/
void Quaternion2Euler(float& roll, float& pitch, float& yaw, float q0, float q1, float q2, float q3)
{
    roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
    pitch = asin(2*(q0*q2-q3*q1));
    yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}
