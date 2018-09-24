#include "ros_node.h"

RosNode::RosNode()
{
}

/*****************************************************************************************
RosNode: construct an object for ros node
******************************************************************************************/
RosNode::RosNode(ros::NodeHandle nh ,std::string name){
  _nh = nh;
  _name = name;
  _queue_size = 10;

  _pub_imu = _nh.advertise <sensor_msgs::Imu>             ("testbed/sensors/row/imu"     , _queue_size);
  _pub_mag = _nh.advertise <sensor_msgs::MagneticField>   ("testbed/sensors/row/mag"     , _queue_size);
  _pub_enc = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/row/encoders", _queue_size);
  _pub_rpy = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/filtered/rpy", _queue_size);
  _pub_du  = _nh.advertise <geometry_msgs::TwistStamped>  ("testbed/motors/du"           , _queue_size);


  _sub_ang = _nh.subscribe("testbed/cmd/angle", _queue_size, &RosNode::cmdAngCallback, this);
  _sub_du  = _nh.subscribe("testbed/cmd/du"   , _queue_size, &RosNode::cmdDuCallback , this);

  _cmd_ang [0] = 0.0;
  _cmd_ang [1] = 0.0;
  _cmd_ang [2] = 0.0;
  _cmd_du[0] = 0.0;
  _cmd_du[1] = 0.0;
  _cmd_du[2] = 0.0;
  _cmd_du[3] = 0.0;
  _cmd_quat.x = 0;
  _cmd_quat.y = 0;
  _cmd_quat.z = 0;
  _cmd_quat.w = 1;
}

/*****************************************************************************************
publishMsgs: Publish all msg
******************************************************************************************/
void RosNode::publishAllMsgs(const float gyro[3], const float acc[3], const float quat[4], const float mag[3],
const float enc[3], const float rpy[3], const float du[4]){
  _time = ros::Time::now();
  publishIMUMsg(gyro, acc, quat);
  publishMagMsg(mag);
  publishEncMsg(enc);
  publishRPYMsg(rpy);
  publishDuMsg(du);
}

/*****************************************************************************************
publishIMUMsg: Publish imu msgs
******************************************************************************************/
void RosNode::publishIMUMsg(const float gyro[3], const float acc[3], const float quat[4]){
  sensor_msgs::Imu msg_imu;

  msg_imu.header.stamp = _time;
  msg_imu.header.seq++;
  msg_imu.angular_velocity.x = gyro[0];
  msg_imu.angular_velocity.y = gyro[1];
  msg_imu.angular_velocity.z = gyro[2];
  msg_imu.linear_acceleration.x = acc[0];
  msg_imu.linear_acceleration.y = acc[1];
  msg_imu.linear_acceleration.z = acc[2];
  msg_imu.orientation.x = quat[0];
  msg_imu.orientation.y = quat[1];
  msg_imu.orientation.z = quat[2];
  msg_imu.orientation.w = quat[3];
  _pub_imu.publish(msg_imu);
}

/*****************************************************************************************
publishMagMsg: Publish magnetometer msgs
******************************************************************************************/
void RosNode::publishMagMsg(const float Mag[3]){
  sensor_msgs::MagneticField msg_mag;

  msg_mag.header.stamp = _time;
  msg_mag.header.seq++;
  msg_mag.magnetic_field.x = Mag[0];
  msg_mag.magnetic_field.y = Mag[1];
  msg_mag.magnetic_field.z = Mag[2];
  _pub_mag.publish(msg_mag);
}

/*****************************************************************************************
publishRPYMsg: Publish filtered roll pitch yaw angles msgs
******************************************************************************************/
void RosNode::publishRPYMsg(const float rpy[3])
{
  geometry_msgs::Vector3Stamped msg_rpy;

  msg_rpy.header.stamp = _time;
  msg_rpy.header.seq++;
  msg_rpy.vector.x = rpy[0];
  msg_rpy.vector.y = rpy[1];
  msg_rpy.vector.z = rpy[2];
  _pub_rpy.publish(msg_rpy);
}

/*****************************************************************************************
publishEncMsg: Publish encoder msgs
******************************************************************************************/
void RosNode::publishEncMsg(const float enc[3]){
  geometry_msgs::Vector3Stamped msg_enc;

  msg_enc.header.stamp = _time;
  msg_enc.header.seq++;
  msg_enc.vector.x = enc[0];
  msg_enc.vector.y = enc[1];
  msg_enc.vector.z = enc[2];
  _pub_enc.publish(msg_enc);
}

/*****************************************************************************************
publishMsgs: Publish motors' inputs du msgs
******************************************************************************************/
void RosNode::publishDuMsg(const float du[4]){
  geometry_msgs::TwistStamped msg_du;

  msg_du.header.stamp = _time;
  msg_du.header.seq++;
  msg_du.twist.linear.z  = du[0];
  msg_du.twist.angular.x = du[1];
  msg_du.twist.angular.y = du[2];
  msg_du.twist.angular.z = du[3];
  _pub_du.publish(msg_du);
}

/*****************************************************************************************
angCmdCallback: Read command angle
******************************************************************************************/
void RosNode::cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  _cmd_ang[0] = msg->vector.x;
  _cmd_ang[1] = msg->vector.y;
  _cmd_ang[2] = msg->vector.z;
}

/*****************************************************************************************
duCmdCallback: Read command duty cycle
******************************************************************************************/
void RosNode::cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _cmd_du[0] = msg->twist.linear.z;
  _cmd_du[1] = msg->twist.angular.x;
  _cmd_du[2] = msg->twist.angular.y;
  _cmd_du[3] = msg->twist.angular.z;

}
/*****************************************************************************************
duCmdCallback: Read command quaternion vector
******************************************************************************************/
void RosNode::cmdQuatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
  _cmd_quat.x = msg->x;
  _cmd_quat.y = msg->y;
  _cmd_quat.z = msg->z;
  _cmd_quat.w = msg->w;

}
