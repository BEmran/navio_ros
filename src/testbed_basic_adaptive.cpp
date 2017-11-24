/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on November 22, 2017, 12:20 PM
 */
#include "testbed_navio/testbed_basic.h"

float adaptive(dataStruct* data, float cmd, float dt);
void refDyn(float* y, float* x, float* xdot, float* u, float t);
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
    float ref0[2] = {0,0};
    data.refSys = DynSys(2, *refDyn,ref0);
    data.ref = data.refSys.getY();
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
    //printf("%2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t \n",ang[2], w[2], data->rosnode->_cmd_ang[2], cmd_adj[2],e[2],data->du[2],ei[2]);
    data->du[0] = adaptive(data,cmd_adj[0],dt);
    // Send control signal
    data->du[3] = 2.0;
}

float adaptive(dataStruct* data, float cmd, float dt){
    static float best = 30, aest[2] = {15.0,1.2};
    static float W[5] = {0,0,0,0,0};
    float blimit = 5;
    float C[5] = {-0.1,0.05,0,0.05,0.1};
    float gama = 1;

    //
    float vrm = - 31.0 * data->ref[0] - 8.0 * data->ref[1] + 31.0 * cmd;
    //
    float e    = data->enc[0] - data->ref[0];
    float edot = data->w[0]   - data->ref[1];

    if (abs(e) < 0.005){
        e = 0;
    }
    if (abs(edot) < 0.01){
        edot = 0;
    }
    float s = edot + 4.0 * e;

    // adaptive
    float h = 0;
    float vad = 0;
    float Wdot = 0;
    for (int ii=0; ii<5; ii++){
        h = exp( (e-C[ii])*(e-C[ii]) / 2 / 0.5 / 0.5 );
        vad+= W[ii]*h;
        Wdot = 10.0 * gama * (e * h - 0.1 * abs(e) * W[ii]);
        W[ii] += Wdot * dt;
    }
    //
    vad = sat(vad,10,-10);
    //float v = - 3.0 * s - 4.0 * edot + vrm - aest[0] * data->enc[0] - aest[1] * data->w[0];
    float v = - 3.0 * s - 4.0 * edot + vrm;
    float u = (v - vad) / best;
    u = sat(u,0.5,-0.5);
    //
    float vh = v - vad - u * best;
    float rdot[1] = {vrm - vh};
    data->refSys.update(rdot,0,dt);
    //
    //aest[0] += gama * s * data->enc[0];
    //aest[1] += gama * s * data->w[0];
    float best_dot = gama * s * u;
    if (best_dot < 0 && best <= blimit)
        best_dot = 1;
    best += best_dot * dt;
    //
    static float dtsum=0;
    dtsum += dt;
    if (dtsum > 0.2){
        dtsum = 0;
        printf("e = %+2.3f\t s = %+2.3f\t yr = %+2.3f\t vad = %+2.3f\t best = %+2.3f\n",
        e, s, data->ref[0], vad, best);
    }    //
    return u;
}


/*****************************************************************************************
 wdotDyn: Dynamic system for a derivative + filter
 *****************************************************************************************/
void refDyn(float* y, float* x, float* xdot, float* u, float t)
{
    xdot[0] = x[1];
    xdot[1] = u[0];

    y[0] = x[0];
    y[1] = x[1];
}
