#ifndef BASIC_ROS_NODE_H
#define BASIC_ROS_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"                              // IMU sensor msg
#include "sensor_msgs/MagneticField.h"            // Magnetic sensor msg
#include "geometry_msgs/TwistStamped.h"       // du msg
#include "geometry_msgs/Vector3Stamped.h"   // RPY msg

class BasicRosNode
{

    protected:
        ros::NodeHandle _nh;

        ros::Publisher _pub_imu;
        ros::Publisher _pub_mag;
        ros::Publisher _pub_rpy;
        ros::Publisher _pub_du;

        ros::Subscriber _sub_ang;
        ros::Subscriber _sub_du;
        ros::Subscriber _sub_enc;

        sensor_msgs::Imu _msg_imu;
        sensor_msgs::MagneticField _msg_mag;
        geometry_msgs::Vector3Stamped _msg_rpy;
        geometry_msgs::TwistStamped _msg_du;

        int queue_size;
        std::string _name;

    public:
        float _enc[3];
        float _cmd_du[4];
        float _cmd_ang[3];

        BasicRosNode();
        BasicRosNode(ros::NodeHandle nh, std::string name);
        ~BasicRosNode();

        void publishAllMsgs(float gyro[3], float acc[3], float quat[4],float Mag[3],float rpy[3],float du[3]);
        void publishIMUMsg(float gyro[3], float acc[3], float quat[4]);
        void publishMagMsg(float Mag[3]);
        void publishRPYMsg(float rpy[3]);
        void publishDuMsg(float du[3]);

        void cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void cmdAngCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void encCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

};

#endif // BASIC_ROS_NODE_H


