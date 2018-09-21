#ifndef ROS_NODE_BASIC_H
#define ROS_NODE_BASIC_H

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // w and encoder msg

class BasicRosNode
{

    protected:
        int _queue_size;

        ros::NodeHandle _nh;
        ros::Publisher _pub_w;
        ros::Publisher _pub_du;
        ros::Publisher _pub_enc;
        ros::Subscriber _sub_du;
        ros::Subscriber _sub_ang;

        ros::Time _time;
        std::string _name;

    public:
        float _enc[3];
        float _cmd_du[4];
        float _cmd_ang[3];

        BasicRosNode();
        BasicRosNode(ros::NodeHandle nh, std::string name);
        ~BasicRosNode();

        void publishAllMsgs(float enc[3], float w[3], float du[3]);
        void publishWMsg(float w[3]);
        void publishDuMsg(float du[3]);
        void publishEncMsg(float enc[3]);

        void cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
};

#endif // ROS_NODE_BASIC_H


