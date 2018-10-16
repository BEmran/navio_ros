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
  ros::Rate loop_rate(_MAINFUN_FREQ);
  while (ros::ok() && !_CloseRequested)
  {
    loop(data);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit procedure -------------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Close program\n");
  return 0;
  pthread_cancel(_Thread_Sensors);
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
