#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg

using namespace std;
/**************************************************************************************************
 *
**************************************************************************************************/
class ROSNODE{
protected:
  int _queue_size;
  ros::NodeHandle _nh;
  ros::Publisher _pub_du;
  ros::Subscriber _sub_enc;
  std::string _name;
public:
  float _ang[3];
  ROSNODE(){}
  ROSNODE(ros::NodeHandle nh, std::string name){
      _ang[0] = 0.0; _ang[1] = 0.0; _ang[2] = 0.0;
      _nh = nh;
      _name = name;
      _queue_size = 10;
      _pub_du = _nh.advertise <geometry_msgs::Vector3Stamped>("du", _queue_size);
      _sub_enc  = _nh.subscribe("du"   , _queue_size, &ROSNODE::angCallback , this);
  }
  ~ROSNODE(){}
  void publishDuMsg(const float du[4]){
      geometry_msgs::TwistStamped msg_du;
      msg_du.header.stamp = ros::Time::now();
      msg_du.header.seq++;
      msg_du.twist.linear.z  = du[0];
      msg_du.twist.angular.x = du[1];
      msg_du.twist.angular.y = du[2];
      msg_du.twist.angular.z = du[3];
      _pub_du.publish(msg_du);
  }
  void angCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
      _ang[0] = msg->vector.x;
      _ang[1] = msg->vector.y;
      _ang[2] = msg->vector.z;
  }
};
/**************************************************************************************************
 *
**************************************************************************************************/
void pid(float ang[], float du[]){
    static float ei = 0, e0 = 0;
    float Kp = 0.3, Ki = 0.1, Kd = 0.1;
    float e = 0 - ang[0];
    ei += e * 0.01;
    float ed = (e - e0) / 0.01;
    e0 = e;
    float uz = 0.3;
    float ur = Kp * e + Ki * ei + Kd * ed;
    float up = 0;
    du[0] = uz / 4.0 - up / 2.0;
    du[1] = uz / 4.0 - ur / 2.0;
    du[2] = uz / 4.0 + up / 2.0;
    du[3] = uz / 4.0 + ur / 2.0;
}
/**************************************************************************************************
 *
**************************************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid");
    ros::NodeHandle nh;
    ROSNODE rosnode (nh, "pid");
    ros::Rate loop_rate(100);
    float du[4];
    while (ros::ok()){
        pid(rosnode._ang, du);
        rosnode.publishDuMsg(du);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return(0);
}

