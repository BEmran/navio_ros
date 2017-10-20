#include "basic_ros_node.h"

BasicRosNode::BasicRosNode()
{
}

/*****************************************************************************************
BasicRosNode: construct an object for ros node
******************************************************************************************/
BasicRosNode::BasicRosNode(ros::NodeHandle nh ,std::string name){
    _nh = nh;
    _name = name;
    queue_size = 10;
    _pub_imu = _nh.advertise <sensor_msgs::Imu>("testbed/sensors/imu", queue_size);
    _pub_mag = _nh.advertise <sensor_msgs::MagneticField>("testbed/sensors/mag", queue_size);
    _pub_rpy = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/rpy/filtered", queue_size);
    _pub_enc = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/encoders", queue_size);
    _pub_du = _nh.advertise <geometry_msgs::TwistStamped>("testbed/motors/du", queue_size);

    _sub_ang = _nh.subscribe("testbed/cmd/angle", queue_size, &BasicRosNode::cmdAngCallback,this);
    _sub_du = _nh.subscribe("testbed/cmd/du", queue_size, &BasicRosNode::cmdDuCallback,this);
//    _sub_enc = _nh.subscribe("testbed/sensors/encoders", queue_size, &BasicRosNode::encCallback,this);
}
/*****************************************************************************************
publishIMUMsg: Publish imu msgs
******************************************************************************************/
void BasicRosNode::publishIMUMsg(float gyro[3], float acc[3], float quat[4])
{
    _msg_imu.header.stamp = ros::Time::now();
    _msg_imu.header.seq++;
    _msg_imu.angular_velocity.x = gyro[0];
    _msg_imu.angular_velocity.y = gyro[1];
    _msg_imu.angular_velocity.z = gyro[2];
    _msg_imu.linear_acceleration.x = acc[0];
    _msg_imu.linear_acceleration.y = acc[1];
    _msg_imu.linear_acceleration.z = acc[2];
    _msg_imu.orientation.x = quat[0];
    _msg_imu.orientation.y = quat[1];
    _msg_imu.orientation.z = quat[2];
    _msg_imu.orientation.w = quat[3];
    _pub_imu.publish(_msg_imu);
}

/*****************************************************************************************
publishMagMsg: Publish magnetometer msgs
******************************************************************************************/
void BasicRosNode::publishMagMsg(float Mag[3])
{
    _msg_mag.header.stamp = ros::Time::now();
    _msg_mag.header.seq++;
    _msg_mag.magnetic_field.x = Mag[0];
    _msg_mag.magnetic_field.y = Mag[1];
    _msg_mag.magnetic_field.z = Mag[2];
    _pub_mag.publish(_msg_mag);
}

/*****************************************************************************************
publishRPYMsg: Publish roll pitch yaw angles msgs
******************************************************************************************/
void BasicRosNode::publishRPYMsg(float rpy[3])
{
    _msg_rpy.header.stamp = ros::Time::now();
    _msg_rpy.header.seq++;
    _msg_rpy.vector.x = rpy[0];
    _msg_rpy.vector.y = rpy[1];
    _msg_rpy.vector.z = rpy[2];
    _pub_rpy.publish(_msg_rpy);
}
/*****************************************************************************************
publishEncMsg: Publish encoder msgs
******************************************************************************************/
void BasicRosNode::publishEncMsg(float enc[3])
{
    _msg_enc.header.stamp = ros::Time::now();
    _msg_enc.header.seq++;
    _msg_enc.vector.x = enc[0];
    _msg_enc.vector.y = enc[1];
    _msg_enc.vector.z = enc[2];
    _pub_enc.publish(_msg_enc);
}
/*****************************************************************************************
publishMsgs: Publish motors' inputs du msgs
******************************************************************************************/
void BasicRosNode::publishDuMsg(float du[3])
{
    _msg_du.header.stamp = ros::Time::now();
    _msg_du.header.seq++;
    _msg_du.twist.angular.x = du[0];
    _msg_du.twist.angular.y = du[1];
    _msg_du.twist.angular.z = du[2];
    _msg_du.twist.linear.x = du[3];
    _pub_du.publish(_msg_du);
}

/*****************************************************************************************
publishMsgs: Publish motors' inputs du msgs
******************************************************************************************/
void BasicRosNode::publishAllMsgs(float gyro[3], float acc[3], float quat[4], float Mag[3], float rpy[3], float enc[3], float du[3])
{
    publishIMUMsg(gyro, acc, quat);
    publishMagMsg(Mag);
    publishRPYMsg(rpy);
    publishEncMsg(enc);
    publishDuMsg(du);
}

/*****************************************************************************************
angCmdCallback: Read encoders and map it to gloabal variable
******************************************************************************************/
void BasicRosNode::cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    _cmd_ang[0] = msg->vector.x;
    _cmd_ang[1] = msg->vector.y;
    _cmd_ang[2] = msg->vector.z;
}

/*****************************************************************************************
duCmdCallback: Read cmanded du values and map it to gloabal variable
******************************************************************************************/
void BasicRosNode::cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    _cmd_du[0] = msg->twist.angular.x;
    _cmd_du[1] = msg->twist.angular.y;
    _cmd_du[2] = msg->twist.angular.z;
    _cmd_du[3] = msg->twist.linear.z;
}

/*****************************************************************************************
encodersCallback: Read encoders and map it to gloabal variable
******************************************************************************************/
void BasicRosNode::encCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    _enc[0] = msg->vector.x;
    _enc[1] = msg->vector.y;
    _enc[2] = msg->vector.z;
}

