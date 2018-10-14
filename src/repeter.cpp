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
  std::vector<float> data;
  int i = 0;
  std::string label = "sec, nsec, iteration, 1, 2";
  // Main loop ----------------------------------------------------------------------------------
  while (ros::ok()){
    data.clear();
    i++;
    data.push_back(ros::Time::now().toSec());
    data.push_back(ros::Time::now().toNSec());
    data.push_back(i);
    data.push_back(1);
    data.push_back(2);
    // publish du values
    rosnode.publishDataMsg(data, label);
    // spin
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Exit procedure -----------------------------------------------------------------------------
  return(0);
}

