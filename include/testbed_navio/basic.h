/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef BASIC
#define BASIC

/*****************************************************************************************
Header files
******************************************************************************************/
#include <signal.h>       // signal ctrl+c
#include <stdio.h>        // printf
#include <cstdlib>
#include <unistd.h>      // usleep
#include <pthread.h>   // create thread

#include "navio_interface.h"         // Navio io interfaces
#include "lib/SamplingTime.h"      // Sampling time Class

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"   // for encodres msg
#include "geometry_msgs/TwistStamped.h"       // for du msg
#include "sensor_msgs/Imu.h"                              // for IMU sensor msg
#include "sensor_msgs/MagneticField.h"            // for Magnetic sensor msg
/*****************************************************************************************
Global variables
******************************************************************************************/
#define _SENSOR_FREQ 500        // Sensor thread frequency in Hz
#define _CONTROL_FREQ 200     // Control thread frequency in Hz
#define _ROS_FREQ 100                // ROS thread frequency in Hz
#define _MAX_ROLL   0.3
#define _MAX_PITCH  0.3
#define _MAX_YAW    0.6
#define _MAX_Thrust  0.7

using namespace std;
bool _CloseRequested = false;
pthread_t _Thread_Sensors;
pthread_t _Thread_Control;
pthread_t _Thread_RosNode;
geometry_msgs::Vector3Stamped encoders;

/*****************************************************************************************
Define structures
******************************************************************************************/
struct controlStruct {
        std::vector<double> kp;
        std::vector<double> ki;
        std::vector<double> kd;
};
struct dataStruct {
        float pwmVal[4];
        float du[4];
        float encoders[3];
        PWM pwm;
        imuStruct imu;
        InertialSensor *ins;
        AHRS ahrs;
        controlStruct angCon;
        int argc;
        char** argv;
        imu_tools::ComplementaryFilter comp_filter_;
};

/*****************************************************************************************
Functions prototype
******************************************************************************************/
void *sensorsThread(void *data);
void *controlThread(void *data);
void *rosNodeThread(void *data);
void ctrlCHandler(int signal);
void du2motor(PWM* pwm, float du0,float du1,float du2,float du3);
float sat(float x, float upper, float lower);
void encodersCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void initializeParams(ros::NodeHandle& n, imu_tools::ComplementaryFilter& comp_filter_);

/*****************************************************************************************
 ctrlCHandler: Detect ctrl+c to quit program
 ****************************************************************************************/
void ctrlCHandler(int signal) {
    _CloseRequested = true;
    printf("Ctrl+c have been detected\n");
}

/*****************************************************************************************
 du2motor: map du to PWM and send signal to motors
 *****************************************************************************************/
void du2motor(PWM* pwm, float du0, float du1, float du2, float du3) {

    //---------------------------------- apply saturation for du -----------------------------------
    float dr = sat(du0, _MAX_ROLL   , -_MAX_ROLL    ) / 2.0;
    float dp = sat(du1, _MAX_PITCH  , -_MAX_PITCH   ) / 2.0;
    float dw = sat(du2, _MAX_YAW    , -_MAX_YAW     ) / 4.0;
    float dz = sat(du3, _MAX_Thrust , 0             ) / 1.0;

    //----------------------------------------- du to PWM ------------------------------------------
    float uPWM[4];
    uPWM[0] = dz - dp - dw;
    uPWM[1] = dz - dr + dw;
    uPWM[2] = dz + dp - dw;
    uPWM[3] = dz + dr + dw;

    uPWM[0] = uPWM[0] + _SERVO_MIN;
    uPWM[1] = uPWM[1] + _SERVO_MIN;
    uPWM[2] = uPWM[2] + _SERVO_MIN;
    uPWM[3] = uPWM[3] + _SERVO_MIN;
    //---------------------------------- send PWM duty cycle ------------------------------------
    setPWMDuty(pwm, uPWM);
}

/*****************************************************************************************
 sat: apply saturation
 *****************************************************************************************/
float sat(float x, float upper, float lower) {
    if (x <= lower)
        x = lower;
    else if (x >= upper)
        x = upper;
    return x;
}

/*****************************************************************************************
encodersCallback: Read encoders and map it to gloabal variable
******************************************************************************************/
void encodersCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    encoders.header = msg->header;
    encoders.vector = msg->vector;
}

/*****************************************************************************************
initializeParams:
******************************************************************************************/
void initializeParams(ros::NodeHandle& n, imu_tools::ComplementaryFilter& comp_filter_){
    double gain_acc;
    double gain_mag;
    bool do_bias_estimation;
    double bias_alpha;
    bool do_adaptive_gain;

    if (!n.getParam ("gain_acc", gain_acc))
        gain_acc = 0.01;
    if (!n.getParam ("gain_mag", gain_mag))
        gain_mag = 0.01;
    if (!n.getParam ("do_bias_estimation", do_bias_estimation))
        do_bias_estimation = true;
    if (!n.getParam ("bias_alpha", bias_alpha))
        bias_alpha = 0.01;
    if (!n.getParam ("do_adaptive_gain", do_adaptive_gain))
        do_adaptive_gain = true;

    comp_filter_.setDoBiasEstimation(do_bias_estimation);
    comp_filter_.setDoAdaptiveGain(do_adaptive_gain);

    if(!comp_filter_.setGainAcc(gain_acc))
        ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
    if(!comp_filter_.setGainMag(gain_mag))
        ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
    if (do_bias_estimation)
    {
        if(!comp_filter_.setBiasAlpha(bias_alpha))
            ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
    }
}

#endif // BASIC



