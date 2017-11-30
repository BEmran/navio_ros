#ifndef ROS_NODE_FULL_H
#define ROS_NODE_FULL_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"                // IMU sensor msg
#include "sensor_msgs/MagneticField.h"      // Magnetic sensor msg
#include "geometry_msgs/TwistStamped.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // RPY msg

class FullRosNode
{

    protected:
        int _queue_size;

        ros::NodeHandle _nh;
        ros::Publisher _pub_w;
        ros::Publisher _pub_du;
        ros::Publisher _pub_enc;
        ros::Publisher _pub_imu;
        ros::Publisher _pub_mag;
        ros::Publisher _pub_rpy;
        ros::Subscriber _sub_du;
        ros::Subscriber _sub_ang;

        ros::Time _time;
        std::string _name;

    public:
        float _enc[3];
        float _cmd_du[4];
        float _cmd_ang[3];

        FullRosNode();
        FullRosNode(ros::NodeHandle nh, std::string name);
        ~FullRosNode();
        void publishAllMsgs(float gyro[3], float acc[3], float quat[4], float mag[3], float rpy[3], float w[3], float du[3], float enc[3]);
        void publishWMsg(float w[3]);
        void publishDuMsg(float du[3]);
        void publishEncMsg(float enc[3]);
        void publishIMUMsg(float gyro[3], float acc[3], float quat[4]);
        void publishMagMsg(float Mag[3]);
        void publishRPYMsg(float rpy[3]);

        void cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
};

#endif // ROS_NODE_FULL_H


