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
    printf("Start Program\n");
    signal(SIGINT, ctrlCHandler);

    //------------------------------------- Define main variables --------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;
    data.is_sensor_ready = false;

    //----------------------------------------- Start threads ---------------------------------------
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);

    //------------------------------------------  Main loop ------------------------------------------
    while (!_CloseRequested) {
        printf(".\n");
        sleep(1);
    }

    //---------------------------------------- Exit procedure -------------------------------------
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
    float cmd_max[3] = {0.15, 0.15, 0.5};
    float cmd_adj[3];
    float u_max[3] = {_MAX_ROLL,_MAX_PITCH,_MAX_YAW};
//    float ang[3] = {data->imu.r,data->imu.p,data->imu.w};
    float ang[3] = {data->rosnode->_enc[0], data->rosnode->_enc[1],data->rosnode->_enc[2]};

    float w[3] = {data->imu.gyro[0],data->imu.gyro[1],data->imu.gyro[2]};
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
    // Send control signal
    data->du[3] = 0.5;
}
