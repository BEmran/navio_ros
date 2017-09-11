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
#include "sensor_msgs/Imu.h"                              // for sensor msg

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
geometry_msgs::Vector3Stamped encoderes;

/*****************************************************************************************
Define structures
******************************************************************************************/
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
void *rosNodeThread(void *data);
void ctrlCHandler(int signal);
void du2motor(PWM* pwm, float du0,float du1,float du2,float du3);
float sat(float x, float upper, float lower);
void encoderesCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

#endif // BASIC

