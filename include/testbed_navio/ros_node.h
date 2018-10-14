#ifndef ROS_NODE
#define ROS_NODE

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"                // Imu sensor msg
#include "sensor_msgs/MagneticField.h"      // Magnetic sensor msg
#include "geometry_msgs/Twist.h"     // du msg
#include "geometry_msgs/Vector3.h"   // encoder and RPY msg
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Header.h"

class RosNode
{

protected:
  int _queue_size;

  ros::NodeHandle _nh;
  ros::Publisher _pub_header;     // publish header message
  ros::Publisher _pub_imu;        // publish imu sensor message
  ros::Publisher _pub_mag;        // publish magnetometer message
  ros::Publisher _pub_enc;        // publish encoder message
  ros::Publisher _pub_enc_dot;    // publish encoder rate message
  ros::Publisher _pub_du;         // publish used duty cycle message
  ros::Publisher _pub_data;       // publish data messege
  ros::Subscriber _sub_du;        // subscriber to command duty cycle message from user
  ros::Subscriber _sub_ang;       // subscriber to command angle message from user

  std::string _name;

public:
  float _cmd_du[4];               // command duty cycle data
  float _cmd_ang[3];              // command angle data

  RosNode();
  RosNode(ros::NodeHandle nh, std::string name);
  ~RosNode(){}
  void publishAllMsgs(const float gyro[3], const float acc[3], const float quat[4], const float mag[3],
  const float enc[3], const float enc_dot[3], const float du[4], const std::vector<float> data, std::string label);
  void publishHeader();
  void publishIMUMsg(const float gyro[3], const float acc[3], const float quat[4]);
  void publishMagMsg(const float Mag[3]);
  void publishEncMsg(const float enc[3]);
  void publishEncDotMsg(const float enc_dot[3]);
  void publishDuMsg(const float du[3]);
  void publishDataMsg(const std::vector<float> data, std::string label);

  void cmdDuCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void cmdAngCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

#endif // ROS_NODE


