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
#include "testbed_navio/basic_ros_node.h"

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <iostream>     // localtime
#include <string>     // std::string, std::stof

#include <lib/DynSys.h>     //
/*****************************************************************************************
Global variables
******************************************************************************************/
#define _SENSOR_FREQ 500        // Sensor thread frequency in Hz
#define _CONTROL_FREQ 200     // Control thread frequency in Hz
#define _ROS_FREQ 100                // ROS thread frequency in Hz
#define _MAX_ROLL   1.0
#define _MAX_PITCH  1.0
#define _MAX_YAW    1.0
#define _MAX_THRUST  4.0

using namespace std;
bool _CloseRequested = false;
pthread_t _Thread_Sensors;
pthread_t _Thread_Control;
pthread_t _Thread_RosNode;

/*****************************************************************************************
Define structures
******************************************************************************************/
struct controlStruct {
        std::vector<double> kp;
        std::vector<double> ki;
        std::vector<double> kd;
};

struct dataStruct {
        float du[4];
        float enc[3];
        float enc_dot[3];
        float pwmVal[4];
        float motors_offset[4];
        int argc;
        char** argv;
        bool is_tcp_ready;
        bool is_sensor_ready;
        PWM pwm;
        AHRS ahrs;
        imuStruct imu;
        InertialSensor *ins;
        controlStruct angCon;
        BasicRosNode* rosnode;
        std::vector<float> vec;
        imu_tools::ComplementaryFilter comp_filter_;
        const float* w;
        const float* ref;
        DynSys wSys,refSys;
};

struct tcpStruct{
        int portNum, bufsize;
        int client, server;
        char buffer[1024];
        struct timeval tval;
        struct sockaddr_in server_addr;
        socklen_t size;
};

/*****************************************************************************************
Functions prototype
******************************************************************************************/
void *sensorsThread(void *data);
void *controlThread(void *data);
void *rosNodeThread(void *data);
void ctrlCHandler(int signal);
void du2motor(PWM* pwm, float du[4], float offset[4]);
float sat(float x, float upper, float lower);
void control(dataStruct* data, float dt);
bool initTcp(tcpStruct* tcp);
void getTcpData(tcpStruct* tcp, dataStruct* data, bool print);
void wdotDyn(float* y, float* x, float* xdot, float* u, float t);
void refdotDyn(float* y, float* x, float* xdot, float* u, float t);
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
void du2motor(PWM* pwm, float du[4], float offset[4]) {

    //---------------------------------- apply saturation for du -----------------------------------
    float dr = sat(du[0], _MAX_ROLL   , -_MAX_ROLL    ) / 2.0;
    float dp = sat(du[1], _MAX_PITCH  , -_MAX_PITCH  ) / 2.0;
    float dw = sat(du[2], _MAX_YAW    , -_MAX_YAW     ) / 4.0;
    float dz = sat(du[3],  _MAX_THRUST , 0                      ) / 4.0;

    //----------------------------------------- du to PWM ------------------------------------------
    float uPWM[4];
    uPWM[0] = dz - dp - dw;
    uPWM[1] = dz - dr + dw;
    uPWM[2] = dz + dp - dw;
    uPWM[3] = dz + dr + dw;

    uPWM[0] = uPWM[0] + _SERVO_MIN + offset[0];
    uPWM[1] = uPWM[1] + _SERVO_MIN + offset[1];
    uPWM[2] = uPWM[2] + _SERVO_MIN + offset[2];
    uPWM[3] = uPWM[3] + _SERVO_MIN + offset[3];
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
        ROS_INFO("Found angle control kp gains: kp[0] %f, kp[1] %f, kp[2] %f\n",data->angCon.kp[0],data->angCon.kp[1],data->angCon.kp[2]);
    else {
        ROS_INFO("Can't find angle control kp gains");
        data->angCon.kp.assign(0,0.4);
        data->angCon.kp.assign(1,0.4);
        data->angCon.kp.assign(2,0.8);
    }

    if (n.getParam("testbed/control/angle/gains/ki", data->angCon.ki))
        ROS_INFO("Found angle control ki gains: ki[0] %f, ki[1] %f, ki[2] %f\n",data->angCon.ki[0],data->angCon.ki[1],data->angCon.ki[2]);
    else {
        ROS_INFO("Can't find angle control ki gains");
        data->angCon.ki.assign(0,1.0);
        data->angCon.ki.assign(1,1.0);
        data->angCon.ki.assign(2,2.0);
    }

    if (n.getParam("testbed/control/angle/gains/kd", data->angCon.kd))
        ROS_INFO("Found angle control kd gains: kd[0] %f, kd[1] %f, kd[2] %f\n",data->angCon.kd[0],data->angCon.kd[1],data->angCon.kd[2]);
    else {
        ROS_INFO("Can't find angle control kd gains");
        data->angCon.kd.assign(0,1.0);
        data->angCon.kd.assign(1,1.0);
        data->angCon.kd.assign(2,2.0);
    }
    std::vector<double> offset;
    if (n.getParam("testbed/motors/offset", offset)){
        data->motors_offset[0] = offset[0];
        data->motors_offset[1] = offset[1];
        data->motors_offset[2] = offset[2];
        data->motors_offset[3] = offset[3];
        ROS_INFO("Found motor offset: m0[0] %f, m1[1] %f, m2[2] %f, m3[3] %f\n",data->motors_offset[0],data->motors_offset[1],data->motors_offset[2],data->motors_offset[3]);
    }
    else {
        ROS_INFO("Can't find offset of the motors");
        data->motors_offset[0] = 0.0;
        data->motors_offset[1] = 0.0;
        data->motors_offset[2] = 0.0;
        data->motors_offset[3] = 0.0;
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
        if (dtsumm > 2) {
            dtsumm = 0;
            printf("Sensors thread with ROLL: %+03.2f PITCH: %+03.2f YAW: %+03.2f %d Hz\n"
                   , my_data->imu.rpy[0], my_data->imu.rpy[1], my_data->imu.rpy[2] * -1, int(1 / dt));
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
    while(!my_data->is_sensor_ready);
    int x = 0;
    while (x == 0) {
        printf("Enter 1 to start control\n");
        cin >> x;
        sleep(1);
    }

    //------------------------------------------  Main loop -------------------------------------------
    while (!_CloseRequested) {
        dt = st.tsCalculat();
        if (dt < 0.01)
            control(my_data,dt);
        du2motor(&my_data->pwm,my_data->du, my_data->motors_offset);
        dtsumm += dt;
        if (dtsumm > 2) {
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
    std::string name = "navio_basic";
    ros::init(my_data->argc,my_data->argv,name);
    ros::NodeHandle nh;
    my_data->rosnode = new BasicRosNode (nh,name);
    ros::Rate loop_rate(_ROS_FREQ);
    initializeParams(nh,my_data);

    while(!my_data->is_sensor_ready);
    //------------------------------------------  Main loop -------------------------------------------
    while (ros::ok() && !_CloseRequested)
    {
        my_data->rosnode->publishAllMsgs(my_data->imu.gyro,my_data->imu.acc,
                                         my_data->imu.quat,my_data->imu.mag,my_data->imu.rpy,
                                         my_data->enc,my_data->du, my_data->vec);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //---------------------------------------- Exit procedure ---------------------------------------
    ctrlCHandler(0);
    printf("Exit ROS Node thread\n");
    pthread_exit(NULL);
}

/*****************************************************************************************
 initTcp: initialize tcp socket
 *****************************************************************************************/
bool initTcp(tcpStruct* tcp){
    tcp->bufsize = 1024;

    //------------------------------- Establish socket connection --------------------------------
    tcp->client = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp->client < 0)
    {
        printf("\nError establishing socket...");
        return false;
    }

    printf("\n=> Socket server has been created...");
    tcp->server_addr.sin_family = AF_INET;
    tcp->server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    tcp->server_addr.sin_port = htons(tcp->portNum);

    //-------------------------------------  Binding socket ------------------------------------------
    if ((bind(tcp->client, (struct sockaddr*)&tcp->server_addr,sizeof(tcp->server_addr))) < 0)
    {
        printf("=> Error binding connection, the socket has already been established...\n");
        return false;
    }
    tcp->size = sizeof(tcp->server_addr);

    //-------------------------------------  Listening for clients ------------------------------------
    printf("=> Looking for clients...\n");
    listen(tcp->client, 1);

    //--------------------------------------  Accepting Clients -------------------------------------
    int clientCount = 1;
    tcp->server = accept(tcp->client,(struct sockaddr *)&tcp->server_addr,&tcp->size);
    if (tcp->server < 0)  // first check if it is valid or not
        printf("=> Error on accepting...\n");

    //------------------------------------ Send acknowledgment --------------------------------
    strcpy(tcp->buffer, "=> Server connected...\n");
    send(tcp->server, tcp->buffer, tcp->bufsize, 0);
    printf("=> Connected with the client #%d, you are good to go...\n",clientCount);

    return true;
}

/*****************************************************************************************
 getTcpData: read data from tcp connection and decode it
 *****************************************************************************************/
void getTcpData(tcpStruct* tcp, dataStruct* data, bool print = false){
    char * pEnd;
    unsigned long int time, time0;
    float tmp[3] = {data->enc[0],data->enc[1],data->enc[2]};

    //---------------------------------- Wait for data from user -------------------------------
    recv(tcp->server, tcp->buffer, tcp->bufsize, 0);

    //----------------------------------------- Decode data -------------------------------------
    time = strtoul (tcp->buffer,&pEnd,10);
    time0 = strtoul (pEnd,&pEnd,10);
    data->enc[0] = (float) strtod (pEnd,&pEnd);
    data->enc[1] = (float) strtod (pEnd,&pEnd);
    data->enc[2] = (float) strtod (pEnd,NULL);

    float dt = (time-time0)/1000000.0;
    data->enc_dot[0] = (data->enc[0] - tmp[0]) / dt;
    data->enc_dot[1] = (data->enc[1] - tmp[1]) / dt;
    data->enc_dot[2] = (data->enc[2] - tmp[2]) / dt;

    data->wSys.update(data->enc, 0.0, dt);
    data->enc_dot[0] = data->w[0];
    data->enc_dot[1] = data->w[1];
    data->enc_dot[2] = data->w[2];
    //---------------------------------- Send acknowledgment ------------------------------
    send(tcp->server,"a", 2, 0);

    //---------------------------------- Print data for depuging ------------------------------
    if (print){
        printf("Connection delay (sec)= %+5.5f, %+5.5f, %+5.5f, %+5.5f\n", dt, data->enc[0], data->enc[1], data->enc[2]);
    }
}

/*****************************************************************************************
 wdotDyn: Dynamic system for a derivative + filter
 *****************************************************************************************/
void wdotDyn(float* y, float* x, float* xdot, float* u, float t)
{
    float wf[] = {50, 50, 50};

    xdot[0] = -wf[0] * x[0] - wf[0] * wf[0] * u[0];
    xdot[1] = -wf[1] * x[1] - wf[1] * wf[1] * u[1];
    xdot[2] = -wf[2] * x[2] - wf[2] * wf[2] * u[2];

    y[0] = x[0] + wf[0] * u[0];
    y[1] = x[1] + wf[1] * u[1];
    y[2] = x[2] + wf[2] * u[2];
}
/*****************************************************************************************
 refdotDyn: Dynamic system for a derivative + filter
 *****************************************************************************************/
void refdotDyn(float* y, float* x, float* xdot, float* u, float t)
{
    float a[2] = {11.3225,4.0};
    xdot[0] =                                 x[1];
    xdot[1] = - a[0] * x[0] - a[1] * x[1] +  a[0] * u[0];

    y[0] = x[0];
    y[1] = x[1];
}
#endif // BASIC



