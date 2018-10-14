#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "../include/testbed_navio/ros_node.h"
/**************************************************************************************************
 *
**************************************************************************************************/
int main(int argc, char** argv)
{
  // Ros initialization -------------------------------------------------------------------------
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  RosNode rosnode(nh, "test");
  ros::Rate loop_rate(100);
  float omega[4]={0,0,0,0};
  // Main loop ----------------------------------------------------------------------------------
  while (ros::ok()){
    omega[0] = (float) ros::Time::now().toSec();
    omega[1] = (float) ros::Time::now().toNSec();
    omega[2]++;
    omega[3] = 1;
    // publish du values
    rosnode.publishOmegaMsg(omega);
    // spin
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Exit procedure -----------------------------------------------------------------------------
  return(0);
}

