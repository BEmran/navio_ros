#include "../include/lib/Encoder.h"
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg

using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ednoder_node");
  ros::NodeHandle nh;
  ros::Publisher pub_enc = nh.advertise <geometry_msgs::Vector3Stamped>("encoders", 100);
  ros::Rate loop_rate(100);
  Encoder enc(true);
  float angle[3];
  geometry_msgs::Vector3Stamped enc_msg;

  while (ros::ok()){
    enc.updateCounts();
    enc.readAnglesRad(angle);
    enc_msg.header.stamp = ros::Time::now();
    enc_msg.header.seq++;
    enc_msg.vector.x = angle[0];
    enc_msg.vector.y = angle[1];
    enc_msg.vector.z = angle[2];
    pub_enc.publish(enc_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return(0);
}
