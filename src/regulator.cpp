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
  static PID Wpid[3];
  static PID Apid[3];
  static bool init = true;
  if (init){
    init = false;
    for (int i=0; i<3 ; i++){
      Wpid[i].setGains(400.0, 200.0, 0, 2);
      Apid[i].setGains( 5.0,  3.0, 0, 0.1);
    }
    Wpid[2].setGains(800.0, 500.0, 0, 2);
  }

  data->du[0] = data->du_cmd[0];
  for (int i=0; i<3 ; i++){
    float tmp     = Apid[i].update(data->enc_angle[i], data->ang_cmd[i],  -2.0,   2.0, dt);
    data->du[i+1] = Wpid[i].update(        data->w[i],              tmp,-400.0, 400.0, dt);
  }
}
