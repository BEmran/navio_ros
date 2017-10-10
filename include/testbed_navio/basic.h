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
float ang_cmd[3]={0.0,0.0,0.0};

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
	bool is_sensor_ready;
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
void angCmdCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void initializeParams(ros::NodeHandle& n, imu_tools::ComplementaryFilter& comp_filter_);
void control(dataStruct* data, float dt);

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
void initializeParams(ros::NodeHandle& n, dataStruct* data){
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

   data->comp_filter_.setDoBiasEstimation(do_bias_estimation);
   data-> comp_filter_.setDoAdaptiveGain(do_adaptive_gain);

    if(!data->comp_filter_.setGainAcc(gain_acc))
        ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
    if(!data->comp_filter_.setGainMag(gain_mag))
        ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
    if (do_bias_estimation)
    {
        if(!data->comp_filter_.setBiasAlpha(bias_alpha))
            ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
    }

    //------------------------------------  Get Control Parameter -----------------------------------

    if (n.getParam("testbed/control/angle/gains/kp", data->angCon.kp))
        ROS_INFO("Found angle control kp gains: kp[0] %f, kp[1] %f, kp[2] %f\n",my_data->angCon.kp[0],my_data->angCon.kp[1],my_data->angCon.kp[2]);
    else {
        ROS_INFO("Can't find angle control kp gains");
        data->angCon.kp.assign(0,0.4);
        data->angCon.kp.assign(1,0.4);
        data->angCon.kp.assign(2,0.8);
    }

    if (n.getParam("testbed/control/angle/gains/ki", data->angCon.ki))
        ROS_INFO("Found angle control ki gains: ki[0] %f, ki[1] %f, ki[2] %f\n",my_data->angCon.ki[0],my_data->angCon.ki[1],my_data->angCon.ki[2]);
    else {
        ROS_INFO("Can't find angle control ki gains");
        data->angCon.ki.assign(0,1.0);
        data->angCon.ki.assign(1,1.0);
        data->angCon.ki.assign(2,2.0);
    }

    if (n.getParam("testbed/control/angle/gains/kd", data->angCon.kd))
        ROS_INFO("Found angle control kd gains: kd[0] %f, kd[1] %f, kd[2] %f\n",my_data->angCon.kd[0],my_data->angCon.kd[1],my_data->angCon.kd[2]);
    else {
        ROS_INFO("Can't find angle control kd gains");
        data->angCon.kd.assign(0,1.0);
        data->angCon.kd.assign(1,1.0);
        data->angCon.kd.assign(2,2.0);
    }

}

/*****************************************************************************************
 sensorsThread: read navio sensors (IMU +...) and perfourm AHRS
 *****************************************************************************************/
void *sensorsThread(void *data) {
    printf("Start Sensors thread\n");
    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;

    //----------------------------------------  Initialize IMU ----------------------------------------
    char imu_name[] = "mpu";
    //char imu_name[] = "lsm";
    my_data->ins = imuSetup(&my_data->ahrs, imu_name, &my_data->imu);
    if (my_data->ins == NULL) {
        printf("Cannot initialize imu sensor\n");
        pthread_exit(NULL);
    }
    printf("Initialized imu sensor\n");
    my_data->is_sensor_ready = true;

    //------------------------------------------  Main loop ------------------------------------------
    SamplingTime st(_SENSOR_FREQ);
    float dt, dtsumm = 0;
    while (!_CloseRequested) {
        dt = st.tsCalculat();

        //-------------------------------------- Read Sensor ---------------------------------------
        getIMU(my_data->ins, &my_data->imu);

        //------------------------------------- Perfourm filter ---------------------------------------
        //doAHRS(&my_data->ahrs, &my_data->imu, dt);
        doComplementaryFilter(&my_data->comp_filter_, &my_data->imu, dt);

        //-------------------------------------- Display data ---------------------------------------
        dtsumm += dt;
        if (dtsumm > 1) {
            dtsumm = 0;
            printf("Sensors thread with ROLL: %+03.2f PITCH: %+03.2f YAW: %+03.2f %d Hz\n"
                   , my_data->imu.r, my_data->imu.p, my_data->imu.w * -1, int(1 / dt));
        }
    }

    //---------------------------------------- Exit procedure -------------------------------------
    printf("Exit sensor thread\n");
    pthread_exit(NULL);
}

/*****************************************************************************************
 controlThread: Perfourmcontrol loop and send PWM output
*****************************************************************************************/
void *controlThread(void *data) {
    printf("Start Control thread\n");

    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;
    SamplingTime st(400);
    float dt, dtsumm = 0;
    //--------------------------------------- Initialize PWM ------------------------------------------
    initializePWM(&my_data->pwm);
    my_data->du[0] = 0.0;
    my_data->du[1] = 0.0;
    my_data->du[2] = 0.0;
    my_data->du[3] = 0.0;
    //------------------------------------- Wait for user input --------------------------------------
    int x = 0;
    while (x == 0) {
        printf("Enter 1 to start control\n");
        cin >> x;
        sleep(1);
    }

    //------------------------------------------  Main loop -------------------------------------------
    while (!_CloseRequested) {
        dt = st.tsCalculat();
        du2motor(&my_data->pwm,my_data->du[0],my_data->du[1],my_data->du[2],my_data->du[3]);
        dtsumm += dt;
        if (dtsumm > 1) {
            dtsumm = 0;
            printf("Control thread with %d Hz\n", int(1 / dt));
        }
    }
    //---------------------------------------- Exit procedure ---------------------------------------
    printf("Exit control thread\n");
    pthread_exit(NULL);
}

/*****************************************************************************************
 rosNodeThread: ROS Node thread
 *****************************************************************************************/
void *rosNodeThread(void *data) {
    printf("Start ROS Node thread\n");
    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;

    //--------------------------------------- Initialize ROS ------------------------------------------
    ros::init(my_data->argc,my_data->argv,"navio_basic");
    int queue_size = 10;
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise <sensor_msgs::Imu>("testbed/sensors/imu", queue_size);
    ros::Publisher mag_pub = n.advertise <sensor_msgs::MagneticField>("testbed/sensors/mag", queue_size);
    ros::Publisher rpy_pub = n.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/rpy/filtered", queue_size);
    ros::Publisher du_pub = n.advertise <geometry_msgs::TwistStamped>("testbed/motors/du", queue_size);
    ros::Subscriber ang_cmd_sub = n.subscribe("testbed/cmd/angle", queue_size, angCmdCallback);
    ros::Subscriber encoder_sub = n.subscribe("testbed/sensors/encoders", queue_size, encodersCallback);
    ros::Rate loop_rate(_ROS_FREQ);

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    geometry_msgs::Vector3Stamped rpy_msg;
    geometry_msgs::TwistStamped du_msg;
    initializeParams(n,my_data->comp_filter_);
    while(!my_data->is_sensor_ready);
    //------------------------------------------  Main loop -------------------------------------------
    while (ros::ok() && !_CloseRequested)
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.seq++;

        imu_msg.header = header;
        imu_msg.angular_velocity.x = my_data->imu.gx;
        imu_msg.angular_velocity.y = my_data->imu.gy;
        imu_msg.angular_velocity.z = my_data->imu.gz;
        imu_msg.linear_acceleration.x = my_data->imu.ax;
        imu_msg.linear_acceleration.y = my_data->imu.ay;
        imu_msg.linear_acceleration.z = my_data->imu.az;
        imu_msg.orientation.x = my_data->imu.qx;
        imu_msg.orientation.y = my_data->imu.qy;
        imu_msg.orientation.z = my_data->imu.qz;
        imu_msg.orientation.w = my_data->imu.qw;
        imu_pub.publish(imu_msg);

        mag_msg.header = header;
        mag_msg.magnetic_field.x = my_data->imu.mx;
        mag_msg.magnetic_field.y = my_data->imu.my;
        mag_msg.magnetic_field.z = my_data->imu.mz;
        mag_pub.publish(mag_msg);

        rpy_msg.header = header;
        rpy_msg.vector.x = my_data->imu.r;
        rpy_msg.vector.y = my_data->imu.p;
        rpy_msg.vector.z = my_data->imu.w;
        rpy_pub.publish(rpy_msg);

        du_msg.header = header;
        du_msg.twist.angular.x = my_data->du[0];
        du_msg.twist.angular.y = my_data->du[1];
        du_msg.twist.angular.z = my_data->du[2];
        du_msg.twist.linear.x = my_data->du[3];
        du_pub.publish(du_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    //---------------------------------------- Exit procedure ---------------------------------------
    ctrlCHandler(0);
    printf("Exit ROS Node thread\n");
    pthread_exit(NULL);
}

/*****************************************************************************************
angCmdCallback: Read encoders and map it to gloabal variable
******************************************************************************************/
void angCmdCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    ang_cmd[0] = msg->vector.x;
    ang_cmd[1] = msg->vector.y;
    ang_cmd[2] = msg->vector.z;
}

#endif // BASIC



