/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on November 21, 2017, 2:36 PM
 */
#include "testbed_navio/testbed_basic.h"

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
    data.is_maintcp_ready = false;
    data.is_control_ready = false;
    data.pwm_offset[0] = 0;
    data.pwm_offset[1] = 0;
    data.pwm_offset[2] = 0;
    data.pwm_offset[3] = 0;
    float w0[3] = {50.0,50.0,50.0};
    data.wSys = DynSys(3, *wdotDyn,w0);
    data.w = data.wSys.getY();
    struct tcpStruct tcp;
    tcp.portNum = 1500;
    //----------------------------------------- Start threads ---------------------------------------
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);
    //------------------------------------------ initialize tcp ----------------------------------------
    data.is_maintcp_ready = initTcp(&tcp);
    SamplingTime st(_MAINTCP_FREQ);
    float dt, dtsumm = 0;
    bool enablePrint = false;
    //-------------------------------------------- Min loop ------------------------------------------
    while(!_CloseRequested && data.is_maintcp_ready){
        dt = st.tsCalculat();

        getTcpData(&tcp, &data, dt, enablePrint);
        dtsumm += dt;
        if (dtsumm > 2) {
            dtsumm = 0;
            printf("main    thread with %d Hz\n", int(1 / dt));
        }
    }
    //----------------------------------- Terminate tcp connection -----------------------------
    printf("\n\n=> Connection terminated");
    close(tcp.server);
    close(tcp.client);
    //----------------------------------------- Exit procedure -------------------------------------
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
    float u_max[3] = {_MAX_DU_R,_MAX_DU_P,_MAX_DU_Y};
    float ang[3] = {data->enc[0], data->enc[1],data->enc[2]};
    float w[3] = {data->w[0],data->w[1],data->w[2]};

    // LQR control
    for (int i = 0; i < 3; i++)
    {
        // adjust cmd
        cmd_adj[i] = sat(data->rosnode->_cmd_ang[i], ang[i] + cmd_max[i], ang[i] - cmd_max[i]);

        // traking error
        e[i] = cmd_adj[i] - ang[i];

        // control signal
        float tmp = e[i] * data->angCon.kp[i] - w[i] * data->angCon.kd[i] + ei[i] * data->angCon.ki[i];

        // saturation
        data->du[i] = sat(tmp, u_max[i], -u_max[i]);

        // integration
        ei[i] += e[i] * dt;
    }
    printf("%2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t \n",ang[2], w[2], data->rosnode->_cmd_ang[2], cmd_adj[2],e[2],data->du[2],ei[2]);
    // Send control signal
    data->du[3] = 2.0;
}

