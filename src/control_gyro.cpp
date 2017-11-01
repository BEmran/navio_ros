/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "testbed_navio/basic.h"

/*****************************************************************************************
main: Run main function
 ****************************************************************************************/
int main(int argc, char** argv) {
    //----------------------------------------- Welcome msg -------------------------------------
    printf("Start Program...\n");
    signal(SIGINT, ctrlCHandler);

    //------------------------------------- Define main variables --------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;
    data.is_sensor_ready = false;
    data.is_tcp_ready = false;
    data.motors_offset[0] = 0;
    data.motors_offset[1] = 0;
    data.motors_offset[2] = 0;
    data.motors_offset[3] = 0;
    data.wSys = DynSys(3, *wdotDyn);
    data.w = data.wSys.getY();
    data.refSys = DynSys(2, *refdotDyn);
    data.ref = data.refSys.getY();
    struct tcpStruct tcp;
    tcp.portNum = 1500;
    //----------------------------------------- Start threads ---------------------------------------
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);
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

    pthread_cancel(_Thread_Sensors);
    pthread_cancel(_Thread_Control);
    pthread_cancel(_Thread_RosNode);
    printf("Close program\n");

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
    data->du[3] = 0.5;

    static float b_est = 1;
    static float W[5] = {0.0,0.0,0.0,0.0,0.0};
    float bLower = 0.1;
    float a0 = 30.31;
    float a1 = 8;
    float b = a0;
    //
    float vr = - a0 * data->ref[0] - a1 *data->ref[1] +  b * cmd_adj[0];
    float er        = ang[0]         - data->ref[0];
    float er_dot = data->w[0] - data->ref[1];
    if (abs(er)<0.001)
        er = 0;
    if (abs(er_dot)<0.01)
        er_dot = 0;
    float sr = er_dot + 10.0 * er;
    //
    float h[5], Wdot[5];
    float c[5] = {-1.0,-0.5,0.0,0.5,1.0};
    float d_est = 0.0;
    float gama = 1;
    for (int ii=0;ii<5;ii++){
        h[ii] = exp( - pow(sqrt((er - c[ii])*(er - c[ii])),2) / ( 2 * 2*2));
        d_est = d_est + W[ii] * h[ii];
        Wdot[ii] = gama * (er * h[ii] - 0.1 * abs(er) * W[ii]);
        W[ii] = W[ii] + Wdot[ii] * dt;
    }
    //
    float vi = - 11.0 * sr -10.0 * er_dot + vr;
    float ur = ( - d_est + vi ) / b_est;
    float uCmd = sat(ur,u_max[0], -u_max[0]);
    data->du[0] = uCmd;
    float vh = vi - d_est - uCmd * b_est;
    //

    float eta = 10;
    float b_est_dot = sr * ur * eta;
    if (b_est_dot < 0 && b_est < bLower)
            b_est_dot = 1 / eta;

    b_est = b_est + b_est_dot * dt;

    float xref_dot[1]= {vr - vh};
    data->refSys.update(xref_dot, 0.0, dt);
}
