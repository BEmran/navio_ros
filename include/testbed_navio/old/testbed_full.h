/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */
#ifndef TESTBED_FULL
#define TESTBED_FULL

/*****************************************************************************************
Header files
******************************************************************************************/
#include <signal.h>     // signal ctrl+c
#include <stdio.h>      // printf
#include <cstdlib>
#include <unistd.h>     // usleep
#include <pthread.h>    // create thread

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <iostream>     // localtime
#include <string>       // std::string, std::stof

#include "navio_interface.h"    // Navio io interfaces
#include "lib/SamplingTime.h"   // Sampling time Class
#include "lib/DynSys.h"         // Dyanmic system
#include "testbed_navio/ros_node_full.h"   // ROS node

#include <time.h>       /* time_t, struct tm, time, localtime */
#include <fstream> // file

/*****************************************************************************************
Global variables
******************************************************************************************/
#define _SENSORS_FREQ   500   // Sensor thread frequency in Hz
#define _MAINTCP_FREQ   500   // Main thread frequency in Hz
#define _CONTROL_FREQ   200   // Control thread frequency in Hz
#define _ROSNODE_FREQ   100   // ROS thread frequency in Hz

#define _MAX_DU_R       0.5
#define _MAX_DU_P       0.5
#define _MAX_DU_Y       0.4
#define _MAX_DU_T       2.5

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
        bool is_maintcp_ready;
        bool is_control_ready;
        bool is_sensors_ready;
        float du[4];
        float enc[3];
        float pwm_val[4];
        float pwm_offset[4];
        float record[20];
        const float* w;
        const float* ref;
        FILE *file;
        PWM pwm;
        DynSys wSys;
        DynSys refSys;
        FullRosNode* rosnode;
        controlStruct angCon;
        int argc;
        char** argv;
        AHRS ahrs;
        imuStruct imu;
        InertialSensor *ins;
        imu_tools::ComplementaryFilter comp_filter_;
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
void initializeParams(ros::NodeHandle& n, dataStruct* data);
void printRecord(FILE* file,float data[20]);
/*****************************************************************************************
 ctrlCHandler: Detect ctrl+c to quit program
 ****************************************************************************************/
void ctrlCHandler(int signal) {
    _CloseRequested = true;
    printf("Ctrl+c have been detected\n");
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
 du2motor: map du to PWM and send signal to motors
 *****************************************************************************************/
void du2motor(PWM* pwm, float du[4], float offset[4]) {
    //---------------------------------- apply saturation for du -----------------------------------
    float dr = sat(du[0], _MAX_DU_R, -_MAX_DU_R ) / 2.0;
    float dp = sat(du[1], _MAX_DU_P, -_MAX_DU_P ) / 2.0;
    float dw = sat(du[2], _MAX_DU_Y, -_MAX_DU_Y ) / 4.0;
    float dz = sat(du[3], _MAX_DU_T, 0          ) / 4.0;
    //----------------------------------------- du to PWM ------------------------------------------
    float uPWM[4];
    uPWM[0] = dz - dp - dw + offset[0];
    uPWM[1] = dz - dr + dw + offset[1];
    uPWM[2] = dz + dp - dw + offset[2];
    uPWM[3] = dz + dr + dw + offset[3];
    //---------------------------------- send PWM duty cycle ------------------------------------
    setPWMDuty(pwm, uPWM);
}

/*****************************************************************************************
 controlThread: Perfourmcontrol loop and send PWM output
*****************************************************************************************/
void *controlThread(void *data) {
    printf("Start Control thread\n");
    //----------------------------------- Initialize mapping data -------------------------------------
    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;
    //---------------------------------- Initialize sampling time -------------------------------------
    SamplingTime st(_CONTROL_FREQ);
    float dt, dtsumm = 0;
    //--------------------------------------- Initialize PWM ------------------------------------------
    initializePWM(&my_data->pwm);
    my_data->du[0] = 0.0;
    my_data->du[1] = 0.0;
    my_data->du[2] = 0.0;
    my_data->du[3] = 0.0;
    //------------------------------------- Wait for user input --------------------------------------
    while(!my_data->is_maintcp_ready && !my_data->is_sensors_ready);
    int x = 0;
    while (x == 0) {
        printf("Enter 1 to start control\n");
        cin >> x;
        sleep(1);
    }
    my_data->is_control_ready = true;
    //---------------------------- Print data header ------------------------------------
    fprintf(data->file, "time,roll,pitch,yaw,p,q,r,ur,up,uw,uz,d0,d1,d2,d3,d4,d5\n");
    //------------------------------------------  Main loop -------------------------------------------
    while (!_CloseRequested) {
        dt = st.tsCalculat();   // calculate sampling time
        if (dt < 0.01)          // to check the sampling is not big
            control(my_data,dt);
        du2motor(&my_data->pwm,my_data->du, my_data->pwm_offset);
        printRecord(my_data->file,my_data->record);
        dtsumm += dt;
        if (dtsumm > 2.0) {
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
    //----------------------------------- Initialize mapping data -------------------------------------
    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;
    //--------------------------------------- Initialize ROS ------------------------------------------
    while(!my_data->is_maintcp_ready && !my_data->is_sensors_ready);
    string name = "navio_basic";
    ros::init(my_data->argc, my_data->argv, name);
    ros::NodeHandle nh;
    my_data->rosnode = new FullRosNode (nh,name);
    ros::Rate loop_rate(_ROSNODE_FREQ);
    initializeParams(nh, my_data);
    //------------------------------------------  Main loop -------------------------------------------
    while (ros::ok() && !_CloseRequested)
    {
        float w[3] = {my_data->w[0],my_data->w[1],my_data->w[2]};
        my_data->rosnode->publishAllMsgs(my_data->imu.gyro,
                                         my_data->imu.acc,
                                         my_data->imu.quat,
                                         my_data->imu.mag,
                                         my_data->imu.rpy,
                                         w,
                                         my_data->du,
                                         my_data->enc);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //---------------------------------------- Exit procedure ---------------------------------------
    ctrlCHandler(0);
    printf("Exit ROS Node thread\n");
    pthread_exit(NULL);
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
    my_data->is_sensors_ready = true;

    //------------------------------------------  Main loop ------------------------------------------
    SamplingTime st(_SENSORS_FREQ);
    float dt, dtsumm = 0;
    while (!_CloseRequested) {
        dt = st.tsCalculat();
        //-------------------------------------- Read Sensor ---------------------------------------
        getIMU(my_data->ins, &my_data->imu);
        //------------------------------------- Perfourm filter ---------------------------------------
        doAHRS(&my_data->ahrs, &my_data->imu, dt);
        //doComplementaryFilter(&my_data->comp_filter_, &my_data->imu, dt);
        //-------------------------------------- Display data ---------------------------------------
        dtsumm += dt;
        if (dtsumm > 2) {
            dtsumm = 0;
            printf("Sensors thread with %d Hz\n", int(1 / dt));
        }
    }

    //---------------------------------------- Exit procedure -------------------------------------
    printf("Exit sensor thread\n");
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
void getTcpData(tcpStruct* tcp, dataStruct* data, float dt, bool print){
    char * pEnd;
    unsigned long int time, time0;
    //---------------------------------- Wait for data from user -------------------------------
    recv(tcp->server, tcp->buffer, tcp->bufsize, 0);
    //----------------------------------------- Decode data -------------------------------------
    time = strtoul (tcp->buffer,&pEnd,10);
    time0 = strtoul (pEnd,&pEnd,10);
    data->enc[0] = (float) strtod (pEnd,&pEnd);
    data->enc[1] = (float) strtod (pEnd,&pEnd);
    data->enc[2] = (float) strtod (pEnd,NULL);
    data->wSys.update(data->enc, 0.0, dt);
    //------------------------------------ Send acknowledgment --------------------------------
    send(tcp->server,"a", 2, 0);
    //---------------------------------- Print data for depuging ------------------------------
    if (print){
        printf("Connection delay (sec)= %+5.5f, %+5.5f, %+5.5f, %+5.5f\n",
               dt, data->enc[0], data->enc[1], data->enc[2]);
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
initializeParams:
******************************************************************************************/
void initializeParams(ros::NodeHandle& n, dataStruct* data){
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
    //------------------------------------  Get Motors Offsets -----------------------------------
    std::vector<double> offset;
    if (n.getParam("testbed/motors/offset", offset)){
        data->pwm_offset[0] = offset[0];
        data->pwm_offset[1] = offset[1];
        data->pwm_offset[2] = offset[2];
        data->pwm_offset[3] = offset[3];
        ROS_INFO("Found motor offset: m0[0] %f, m1[1] %f, m2[2] %f, m3[3] %f\n",data->pwm_offset[0],data->pwm_offset[1],data->pwm_offset[2],data->pwm_offset[3]);
    }
    else {
        ROS_INFO("Can't find offset of the motors");
        data->pwm_offset[0] = 0.0;
        data->pwm_offset[1] = 0.0;
        data->pwm_offset[2] = 0.0;
        data->pwm_offset[3] = 0.0;
    }
}

void printRecord(FILE* file,float data[20]){
    //---------------------------- Record data header -----------------------------------
    char buf[512];
    char *pos = buf;
    for (int i = 0; i != 20; i++) {
        pos += sprintf(pos, "%5.5f, ", data[i]);
    }
    fprintf(file, "%s\n", buf);
}

#endif // TESTBED_FULL



