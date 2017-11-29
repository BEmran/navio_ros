/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on November 22, 2017, 12:20 PM
 */
#include "testbed_navio/testbed_basic.h"

void adaptive(dataStruct* data, float cmd[3], float u[3], float dt);
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
    float ref0[6] = {0,0,0,0,0,0};
    data.refSys = DynSys(6, *refDyn,ref0);
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

    static float best[3] = {20.0,20.0,15.0};
    static float W[27] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float blimit[3] = {15.0 ,15.0, 10.0};
    float C[9] = {-1.0,-0.5,-0.1,0.05,0,0.05,0.1,0.5,1.0};
    float gama = 1;
    float K[3] = {5.0,2.0,2.0}, L[3] = {4.0,4.0,1.0},Ki[3] = {20.0,15.0,2.0};
    float rdot[3];
    static float si[3] = {0.0,0.0,0.0};
    float cmd_max[3] = {0.2, 0.2, 0.5};
    float cmd[3];
    float u_max[3] = {_MAX_DU_R,_MAX_DU_P,_MAX_DU_Y};
    float u[3];

    for (int jj = 0 ; jj < 3 ; jj++){
        // adjust cmd
        cmd[jj] = sat(data->rosnode->_cmd_ang[jj], data->enc[jj] + cmd_max[jj], data->enc[jj] - cmd_max[jj]);

        //float vrm = - 757.7 * data->ref[0] - 40.0 * data->ref[1] + 757.7 * cmd;
        float vrm = - 31.0 * data->ref[jj] - 8.0 * data->ref[3+jj] + 31.0 * cmd[jj];

        //
        float e    = data->enc[jj] - data->ref[jj];
        float edot = data->w[jj]   - data->ref[3+jj];

        if (abs(e) < 0.001){
            e = 0;
        }
        if (abs(edot) < 0.01){
            edot = 0;
        }
        float s = edot + L[jj] * e;
        float ss = 0;
        if (s > 0)
            ss = 1;
        else if (s < 0)
            ss = -1;

        // adaptive
        float h = 0;
        float vad = 0;
        float Wdot = 0;
        for (int ii=0; ii<9; ii++){
            h = exp( (e-C[ii])*(e-C[ii]) / 2 / 1.0 / 1.0 );
            vad+= W[ii+jj*9]*h;
            Wdot = 10.0 * gama * (e * h - 0.1 * abs(e) * W[ii+jj*9]);
            W[ii+jj*9] += Wdot * dt;
        }
        //
        vad = sat(vad,10,-10);
        float v = - K[jj] * s - L[jj] * edot + vrm - 0.0 * ss - Ki[jj] * si[jj];
        u[jj] = (v - vad) / best[jj];
        u[jj] = sat(u[jj],0.5,-0.5);
        data->du[jj] = u[jj];
        si[jj] += s * dt;
        //
        float vh = v - vad - u[jj] * best[jj];
        rdot[jj] = vrm - vh;

        float best_dot = - 10 * gama * sat(s,1,-1) * u[jj];
        if (best_dot < 0 && best[jj] <= blimit[jj])
            best_dot = 1;
        best[jj] += best_dot * dt;

    }
    data->refSys.update(rdot,0,dt);
    //
    static float dtsum=0;
    dtsum += dt;
    if (dtsum > 0.2){
        dtsum = 0;
        printf("best = %+2.3f\t %+2.3f\t %+2.3f\n",best[0],best[1],best[2]);
    }
    // Send control signal
    data->du[3] = 2.0;
}


/*****************************************************************************************
 wdotDyn: Dynamic system for a derivative + filter
 *****************************************************************************************/
void refDyn(float* y, float* x, float* xdot, float* u, float t)
{
    xdot[0] = x[3];
    xdot[1] = x[4];
    xdot[2] = x[5];
    xdot[3] = u[0];
    xdot[4] = u[1];
    xdot[5] = u[2];

    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
    y[3] = x[3];
    y[4] = x[4];
    y[5] = x[5];

}
