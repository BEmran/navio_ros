/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on March 14, 2018
 */
#include "../include/testbed_navio/testbed.h"
/**************************************************************************************************
main: Run main function
**************************************************************************************************/
int main(int argc, char** argv) {
  // Initialization -------------------------------------------------------------------------------
  dataStruct* data = mainInitialize(argc, argv);

  // Main loop ------------------------------------------------------------------------------------
  ros::Rate loop_rate(_ROSNODE_FREQ);
  while (ros::ok() && !_CloseRequested)
  {

    // publish ros msgs
    float gyro[3] = {data->sensors->imu.gx, data->sensors->imu.gy, data->sensors->imu.gz};
    float  acc[3] = {data->sensors->imu.ax, data->sensors->imu.ay, data->sensors->imu.az};
    float  mag[3] = {data->sensors->imu.mx, data->sensors->imu.my, data->sensors->imu.mz};
    float  enc_dot[3] = {data->enc_dot[0], data->enc_dot[1], data->enc_dot[2]};
    data->rosnode->publishAllMsgs(gyro, acc, data->enc_ang_bias, mag, data->enc_angle, enc_dot, data->du);

    // Record data in a file
    printRecord(data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit procedure -------------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Close program\n");
  return 0;
}

/**************************************************************************************************
control: Perfourm control loop
**************************************************************************************************/
void control(dataStruct* data, float dt){
  for (int i = 0; i < 4; i++){
    data->du[i] = data->rosnode->_cmd_du[i];
  }
}
