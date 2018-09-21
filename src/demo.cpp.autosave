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
    float gyro[3] = {data->sensors->imu.gx, data->sensors->imu.gy, data->sensors->imu.gz};
    float  acc[3] = {data->sensors->imu.ax, data->sensors->imu.ay, data->sensors->imu.az};
    float  mag[3] = {data->sensors->imu.mx, data->sensors->imu.my, data->sensors->imu.mz};
    float a[3] = {1, 2, 3};
    float b[4] = {4, 5, 6, 7};
    data->rosnode->publishAllMsgs(gyro,
                                  acc,
                                  data->quat,
                                  mag,
                                  data->enc_angle,
                                  data->rpy,
                                  data->du);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit procedure -------------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Close program\n");
  return 0;
  pthread_cancel(_Thread_Sensors);
  //pthread_cancel(_Thread_RosNode);
  pthread_cancel(_Thread_Control);
}

/**************************************************************************************************
control: Perfourm control loop
**************************************************************************************************/
void control(dataStruct* data, float dt){
  for (int i = 0; i < 4; i++){
    data->du[i] = data->rosnode->_cmd_du[i];
  }
}
