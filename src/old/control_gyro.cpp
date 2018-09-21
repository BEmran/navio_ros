/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "testbed_navio/testbed_full_old.h"
#include <iostream>
#include <fstream>
using namespace std;
ofstream myfile;
char buffer[1024];
float adaptive(dataStruct* data, float cmd, float dt);

/*****************************************************************************************
main: Run main function
 ****************************************************************************************/
int main(int argc, char** argv) {
    //----------------------------------------- Welcome msg -------------------------------------
    printf("Start Program...\n");
    signal(SIGINT, ctrlCHandler);
    myfile.open ("data.txt");

    //------------------------------------- Define main variables --------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;
    data.is_sensor_ready = false;
    data.is_tcp_ready = false;
    data.is_control_ready = false;
    data.pwm_offset[0] = 0;
    data.pwm_offset[1] = 0;
    data.pwm_offset[2] = 0;
    data.pwm_offset[3] = 0;
    data.wSys = DynSys(3, *wdotDyn);
    data.w = data.wSys.getY();
    data.refSys = DynSys(2, *refdotDyn);
    data.ref = data.refSys.getY();
    struct tcpStruct tcp;
    tcp.portNum = 1500;
    //----------------------------------------- Start threads ---------------------------------------
    //pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);

    //------------------------------------------ initialize tcp ----------------------------------------
    data.is_tcp_ready = initTcp(&tcp);
    SamplingTime st(1000);
    float dt, dtsumm = 0;
    bool enablePrint;
    //-------------------------------------------- Min loop ------------------------------------------
    while(!_CloseRequested && data.is_tcp_ready){
        dt = st.tsCalculat();

        getTcpData(&tcp, &data, enablePrint);
        enablePrint = false;
        dtsumm += dt;
        if (dtsumm > 2) {
            dtsumm = 0;
            enablePrint = true;
            printf("main thread with %d Hz\n", int(1 / dt));
        }
    }

    //----------------------------------- Terminate tcp connection -----------------------------
    printf("\n\n=> Connection terminated");
    close(tcp.server);
    close(tcp.client);

    //----------------------------------------- Exit procedure -------------------------------------

    //pthread_cancel(_Thread_Sensors);
    pthread_cancel(_Thread_Control);
    pthread_cancel(_Thread_RosNode);
    printf("Close program\n");
    myfile.close();
    return 0;
}

/*****************************************************************************************
control: Perfourm control loop
******************************************************************************************/
void control(dataStruct* data, float dt){

    static float ei[3] = {0.0, 0.0, 0.0};
    float e[3];
    float cmd_max[3] = {0.2, 0.2, 0.5};
    float cmd_adj[3];
    float u_max[3] = {_MAX_ROLL,_MAX_PITCH,_MAX_YAW};
    //    float ang[3] = {data->imu.r,data->imu.p,data->imu.w};
    float ang[3] = {data->enc[0], data->enc[1],data->enc[2]};

    //float w[3] = {data->imu.gyro[0],data->imu.gyro[1],data->imu.gyro[2]};
    float w[3] = {data->enc_dot[0],data->enc_dot[1],data->enc_dot[2]};
    // LQR control
    for (int i = 0; i < 3; i++)
    {
        // adjust cmd
        cmd_adj[i] = sat(data->rosnode->_cmd_ang[i], ang[i] + cmd_max[i], ang[i] - cmd_max[i]);

        // traking error
        e[i] = cmd_adj[i] - ang[i];

        // control signal
        //float tmp = ang[i] * data->angCon.kp[i] - w[i] * data->angCon.kd[i] + ei[i] * data->angCon.ki[i];
        float tmp = e[i] * data->angCon.kp[i] - w[i] * data->angCon.kd[i] + ei[i] * data->angCon.ki[i];

        // saturation
        data->du[i] = sat(tmp, u_max[i], -u_max[i]);

        // integration
        ei[i] += e[i] * dt;
    }
    // Send control signal
    data->du[3] =  2.0;
    data->du[0] = adaptive(data,cmd_adj[0], dt);

}
float adaptive(dataStruct* data, float cmd, float dt){
    // States
    static float b_est = 1, a_est[2] = {15.0,1.2};
    static float W[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static float W_dot_old[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static float dtsum = 0;
    static float si = 0;

    // Parameters
    float bLimit = 0.1;
    float L = 2.0, K = 3.0;
    float a[2] = {11.3225,4.0};
    float gama = 10.0 , eta = 10.0;

    // Reference systems dynamic equations
    float vr = -a[0] * data->ref[0] - a[1] * data->ref[1] + a[0] * cmd;

    // Tracking Error
    float e[2];
    e[0] = data->enc[0] - data->ref[0];
    e[1] = data->w[0] - data->ref[1];
    e[0] = sat(e[0],0.001,-0.001);
    e[1] = sat(e[1],0.01,-0.01);
    float s = e[1] + L * e[0];

    // Adaptive Neural Network
    int node = 5, b = 1;
    float c[5] =  {-0.01,-0.05,0.0,0.05,0.01};
    float h, d_est = 0, W_dot;
    for (int j = 0; j < node; j++){
        //printf("e=%+2.4f\t c=%+2.4f\t pow=%+2.4f\t ff=%+2.4f\t h=%+2.4f\n",e[0],c[j],pow(abs(e[0] - c[j]), 2),- pow(abs(e[0] - c[j]), 2) / ( 2.0 * 0.1),h);
        h = exp( - pow(abs(e[0] - c[j]), 2) / ( 2.0 * 0.1));
        d_est += W[j]* h;
        W_dot = gama * (e[0] * h - 0.1 * abs(e[0]) * W[j]);
        W[j] += (W_dot + W_dot_old[j])/2.0 * dt;
        W_dot_old[j] = W_dot;
    }
//    int k = 5;
//    for (int j = 0; j < node; j++){
//        h = exp( - pow(abs(data->enc[0] - c[j]), 2) / ( 2.0 * b * b));
//        d_est += W[j+k]* h;
//        W_dot = gama * (e[0] * h - 0.1 * abs(e[0]) * W[j+k]);
//        W[j+k] += (W_dot + W_dot_old[j+k])/2.0 * dt;
//        W_dot_old[j+k] = W_dot;
//    }
//    k = 10;
//    for (int j = 0; j < node; j++){
//        h = exp( - pow(abs(data->w[0] - c[j]), 2) / ( 2.0 * b * b));
//       d_est += W[j+k]* h;
//        W_dot = gama * (e[0] * h - 0.1 * abs(e[0]) * W[j+k]);
//        W[j+k] += (W_dot + W_dot_old[j+k])/2.0 * dt;
//        W_dot_old[j+k] = W_dot;
//    }

    //d_est += -a_est[0] * data->enc[0] - a_est[1] * data->w[0];
    d_est = sat(d_est,10,-10);

    // Control Signal
    float v = - K * s - L * e[1] + vr + 1.0*si;
    si += s * dt;
    float u = ( - d_est + v ) / b_est;
    u = sat(u,1,-1);
    float vh = v - d_est - u * b_est;

    // Adaptive Law
    a_est[0]+= s * data->enc[0] / eta * dt;
    a_est[1]+= s * data->w[0] / eta * dt;
    float b_est_dot = s * u / eta;
    if (b_est_dot < 0 && b_est < bLimit){
            b_est_dot = 1 / eta;
    }
    b_est += b_est_dot * dt;

    // Output
    float ref_dot[1]= {vr - vh};
    data->refSys.update(ref_dot, 0.0, dt);

    sprintf(buffer,"%f,%f,%f,%f,%f,%f,%f,%f\n",dt,e[0],cmd,data->enc[0],data->ref[0],u,d_est,b_est);
    myfile << buffer;

    static int ii = 0;
    dtsum += dt;
    if (dtsum > 0.2){
        dtsum = 0;
        printf("%d: e = %+2.3f\t s = %+2.3f\t yr = %+2.3f\t d_est = %+2.3f\t b_est = %+2.3f\n",ii++,
        (float) e[0], (float) s, (float) data->ref[0], (float) d_est, (float) b_est);
    }

    return u;
}
