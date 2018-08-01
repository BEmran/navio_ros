#ifndef ROS_NODE
#define ROS_NODE

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"                // Imu sensor msg
#include "sensor_msgs/MagneticField.h"      // Magnetic sensor msg
#include "geometry_msgs/TwistStamped.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg
#include "geometry_msgs/QuaternionStamped.h"   // Quaternion msg
struct Quat{
  float x;
  float y;
  float z;
  float w;
};
class RosNode
{

protected:
  int _queue_size;

  ros::NodeHandle _nh;
  ros::Publisher _pub_imu;    // publish imu sensor message
  ros::Publisher _pub_mag;    // publish imu magnetometer message
  ros::Publisher _pub_enc;    // publish imu encoder message
  ros::Publisher _pub_rpy;    // publish roll pitch and yaw message got from filter
  ros::Publisher _pub_du;     // publish imu duty cycle message
  ros::Subscriber _sub_du;    // subscriber to desired duty cycle message from user
  ros::Subscriber _sub_ang;   // subscriber to desired angle message from user

  ros::Time _time;
  std::string _name;

public:
  float _enc[3];
  float _cmd_du[4];
  float _cmd_ang[3];
  Quat _cmd_quat;

  RosNode();
  RosNode(ros::NodeHandle nh, std::string name);
  ~RosNode();
  void publishAllMsgs(const float gyro[3], const float acc[3], const float quat[4], const float mag[3],
  const float enc[3], const float rpy[3], const float du[4]);
  void publishIMUMsg(const float gyro[3], const float acc[3], const float quat[4]);
  void publishMagMsg(const float Mag[3]);
  void publishEncMsg(const float enc[3]);
  void publishRPYMsg(const float rpy[3]);
  void publishDuMsg(const float du[3]);

  void cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void cmdQuatCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
};

#endif // ROS_NODE


