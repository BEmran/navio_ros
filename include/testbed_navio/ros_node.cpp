#include "ros_node.h"

RosNode::RosNode(){}

/*****************************************************************************************
RosNode: construct an object for ros node
******************************************************************************************/
RosNode::RosNode(ros::NodeHandle nh ,std::string name){
  _nh = nh;
  _name = name;
  _queue_size = 10;

  _pub_header   = _nh.advertise <std_msgs::Header>            ("testbed/header"                   , _queue_size);
  _pub_imu      = _nh.advertise <sensor_msgs::Imu>            ("testbed/sensors/row/imu"          , _queue_size);
  _pub_mag      = _nh.advertise <sensor_msgs::MagneticField>  ("testbed/sensors/row/mag"          , _queue_size);
  _pub_enc      = _nh.advertise <geometry_msgs::Vector3>      ("testbed/sensors/row/encoders"     , _queue_size);
  _pub_enc_dot  = _nh.advertise <geometry_msgs::Vector3>      ("testbed/sensors/row/encoders_dot" , _queue_size);
  _pub_du       = _nh.advertise <geometry_msgs::Twist>        ("testbed/motors/du"                , _queue_size);
  _pub_data     = _nh.advertise <std_msgs::Float32MultiArray> ("testbed/motors/omega"             , _queue_size);


  _sub_ang = _nh.subscribe("testbed/cmd/angle", _queue_size, &RosNode::cmdAngCallback, this);
  _sub_du  = _nh.subscribe("testbed/cmd/du"   , _queue_size, &RosNode::cmdDuCallback , this);

  _cmd_ang [0] = 0.0;
  _cmd_ang [1] = 0.0;
  _cmd_ang [2] = 0.0;
  _cmd_du[0] = 0.0;
  _cmd_du[1] = 0.0;
  _cmd_du[2] = 0.0;
  _cmd_du[3] = 0.0;
}

/*****************************************************************************************
publishMsgs: Publish all msg
******************************************************************************************/
void RosNode::publishAllMsgs(const float gyro[3], const float acc[3], const float quat[4], const float mag[3],
const float enc[3], const float enc_dot[3], const float du[4], const std::vector<float> data,
std::string label){
  publishHeader();
  publishIMUMsg(gyro, acc, quat);
  publishMagMsg(mag);
  publishEncMsg(enc);
  publishEncDotMsg(enc_dot);
  publishDuMsg(du);
  publishDataMsg(data, label);
}

/*****************************************************************************************
publishHeader: Publish header msgs
******************************************************************************************/
void RosNode::publishHeader(){
  std_msgs::Header msg;
  msg.stamp = ros::Time::now();
  msg.seq++;
  _pub_header.publish(msg);
}

/*****************************************************************************************
publishIMUMsg: Publish imu msgs
******************************************************************************************/
void RosNode::publishIMUMsg(const float gyro[3], const float acc[3], const float quat[4]){
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq++;
  msg.angular_velocity.x = gyro[0];
  msg.angular_velocity.y = gyro[1];
  msg.angular_velocity.z = gyro[2];
  msg.linear_acceleration.x = acc[0];
  msg.linear_acceleration.y = acc[1];
  msg.linear_acceleration.z = acc[2];
  msg.orientation.x = quat[0];
  msg.orientation.y = quat[1];
  msg.orientation.z = quat[2];
  msg.orientation.w = quat[3];
  _pub_imu.publish(msg);
}

/*****************************************************************************************
publishMagMsg: Publish magnetometer msgs
******************************************************************************************/
void RosNode::publishMagMsg(const float Mag[3]){
  sensor_msgs::MagneticField msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq++;
  msg.magnetic_field.x = Mag[0];
  msg.magnetic_field.y = Mag[1];
  msg.magnetic_field.z = Mag[2];
  _pub_mag.publish(msg);
}

/*****************************************************************************************
publishRPYMsg: Publish filtered roll pitch yaw angles msgs
******************************************************************************************/
void RosNode::publishEncDotMsg(const float enc_dot[3])
{
  geometry_msgs::Vector3 msg;
  msg.x = enc_dot[0];
  msg.y = enc_dot[1];
  msg.z = enc_dot[2];
  _pub_enc_dot.publish(msg);
}

/*****************************************************************************************
publishEncMsg: Publish encoder msgs
******************************************************************************************/
void RosNode::publishEncMsg(const float enc[3]){
  geometry_msgs::Vector3 msg;

  msg.x = enc[0];
  msg.y = enc[1];
  msg.z = enc[2];
  _pub_enc.publish(msg);
}

/*****************************************************************************************
publishMsgs: Publish motors' inputs du msgs
******************************************************************************************/
void RosNode::publishDuMsg(const float du[4]){
  geometry_msgs::Twist msg;

  msg.linear.z  = du[0];
  msg.angular.x = du[1];
  msg.angular.y = du[2];
  msg.angular.z = du[3];
  _pub_du.publish(msg);
}

/*****************************************************************************************
publishRPYMsg: Publish filtered roll pitch yaw angles msgs
******************************************************************************************/
void RosNode::publishDataMsg(const std::vector<float> data, std::string label = "")
{
  std_msgs::Float32MultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = data.size();
  msg.layout.dim[0].label = label;
  msg.layout.dim[0].stride = 1;
  msg.data.clear();
  msg.data.insert(msg.data.begin(), data.begin(), data.end());
  _pub_data.publish(msg);
}
/*****************************************************************************************
angCmdCallback: Read command angle
******************************************************************************************/
void RosNode::cmdAngCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  _cmd_ang[0] = msg->x;
  _cmd_ang[1] = msg->y;
  _cmd_ang[2] = msg->z;
}

/*****************************************************************************************
duCmdCallback: Read command duty cycle
******************************************************************************************/
void RosNode::cmdDuCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  _cmd_du[0] = msg->linear.z;
  _cmd_du[1] = msg->angular.x;
  _cmd_du[2] = msg->angular.y;
  _cmd_du[3] = msg->angular.z;

}
