#include "ros_node_basic.h"

BasicRosNode::BasicRosNode()
{
}

/*****************************************************************************************
BasicRosNode: construct an object for ros node
******************************************************************************************/
BasicRosNode::BasicRosNode(ros::NodeHandle nh ,std::string name){
    _nh = nh;
    _name = name;
    _queue_size = 10;
    _pub_w   = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/w"       , _queue_size);
    _pub_du  = _nh.advertise <geometry_msgs::TwistStamped>  ("testbed/motors/du"       , _queue_size);
    _pub_enc = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/encoders", _queue_size);

    _sub_ang = _nh.subscribe("testbed/cmd/angle", _queue_size, &BasicRosNode::cmdAngCallback,this);
    _sub_du  = _nh.subscribe("testbed/cmd/du"   , _queue_size, &BasicRosNode::cmdDuCallback ,this);
    _cmd_ang [0] =0.0;
    _cmd_ang [1] =0.0;
    _cmd_ang [2] =0.0;
    _cmd_du[0] = 0.0;
    _cmd_du[1] = 0.0;
    _cmd_du[2] = 0.0;
    _cmd_du[3] = 0.0;
}


/*****************************************************************************************
publishMsgs: Publish motors' inputs du msgs
******************************************************************************************/
void BasicRosNode::publishWMsg(float w[3]){
    geometry_msgs::Vector3Stamped msg_w;

    msg_w.header.stamp = _time;
    msg_w.header.seq++;
    msg_w.vector.x = w[0];
    msg_w.vector.y = w[1];
    msg_w.vector.z = w[2];
    _pub_w.publish(msg_w);
}

/*****************************************************************************************
publishMsgs: Publish motors' inputs du msgs
******************************************************************************************/
void BasicRosNode::publishDuMsg(float du[3]){
    geometry_msgs::TwistStamped msg_du;

    msg_du.header.stamp = _time;
    msg_du.header.seq++;
    msg_du.twist.angular.x = du[0];
    msg_du.twist.angular.y = du[1];
    msg_du.twist.angular.z = du[2];
    msg_du.twist.linear.z  = du[3];
    _pub_du.publish(msg_du);
}

/*****************************************************************************************
publishEncMsg: Publish encoder msgs
******************************************************************************************/
void BasicRosNode::publishEncMsg(float enc[3]){
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
void BasicRosNode::publishAllMsgs( float w[3], float du[3], float enc[3]){
    _time = ros::Time::now();
    publishWMsg(w);
    publishDuMsg(du);
    publishEncMsg(enc);
}

/*****************************************************************************************
angCmdCallback: Read encoders and map it to gloabal variable
******************************************************************************************/
void BasicRosNode::cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    _cmd_ang[0] = msg->vector.x;
    _cmd_ang[1] = msg->vector.y;
    _cmd_ang[2] = msg->vector.z;
}

/*****************************************************************************************
duCmdCallback: Read cmanded du values and map it to gloabal variable
******************************************************************************************/
void BasicRosNode::cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    _cmd_du[0] = msg->twist.angular.x;
    _cmd_du[1] = msg->twist.angular.y;
    _cmd_du[2] = msg->twist.angular.z;
    _cmd_du[3] = msg->twist.linear.z;
}
