#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"     // du msg

using namespace std;
float sat(float x, float max, float min);
/**************************************************************************************************
 *
**************************************************************************************************/
class ROSNODE{
protected:
  int _queue_size;
  ros::NodeHandle _nh;
  ros::Publisher _pub_du;
  ros::Subscriber _sub_du;
  std::string _name;
public:
  float _du[4];
  ROSNODE(){}
  ROSNODE(ros::NodeHandle nh, std::string name){
      _du[0] = 0.0; _du[1] = 0.0; _du[2] = 0.0; _du[3] = 0.0;
      _nh = nh;
      _name = name;
      _queue_size = 10;
      _pub_du = _nh.advertise <geometry_msgs::Twist>("du", _queue_size);
      _sub_du = _nh.subscribe("du_matlab"   , _queue_size, &ROSNODE::duCallback , this);
  }
  ~ROSNODE(){}
  void publishDuMsg(const float du[4]){
      geometry_msgs::Twist msg_du;
      msg_du.linear.z  = du[0];
      msg_du.angular.x = du[1];
      msg_du.angular.y = du[2];
      msg_du.angular.z = du[3];
      _pub_du.publish(msg_du);
  }
  void duCallback(const geometry_msgs::Twist& msg){
      ROS_INFO("I heard\n");
      _du[0] = msg.linear.z;
      _du[1] = msg.angular.x;
      _du[2] = msg.angular.y;
      _du[3] = msg.angular.z;
  }
};
/**************************************************************************************************
 *
**************************************************************************************************/
int main(int argc, char** argv)
{
    // Ros initialization -------------------------------------------------------------------------
    ros::init(argc, argv, "repeter");
    ros::NodeHandle nh;
    ROSNODE rosnode (nh, "repeter");
    ros::Rate loop_rate(100);

    // Main loop ----------------------------------------------------------------------------------
    while (ros::ok()){
        // publish du values
        rosnode.publishDuMsg(rosnode._du);
        // spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Exit procedure -----------------------------------------------------------------------------
    float off[4] = {0.0, 0.0, 0.0, 0.0};
    rosnode.publishDuMsg(off);
    return(0);
}

