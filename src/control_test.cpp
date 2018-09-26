#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"               // time sampling library
#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg
/**************************************************************************************************
 *
**************************************************************************************************/
class ROSNODE{
protected:
    int _queue_size;
    ros::NodeHandle _nh;
    ros::Publisher _pub_ang;    // publish angle encoder message
    ros::Subscriber _sub_du;    // subscriber to desired duty cycle
    std::string _name;
public:
    float _du[4];
    ROSNODE(){}
    ROSNODE(ros::NodeHandle nh, std::string name){
        _du[0] = 0.0; _du[1] = 0.0; _du[2] = 0.0; _du[3] = 0.0;
        _nh = nh;
        _name = name;
        _queue_size = 10;
        _pub_ang = _nh.advertise <geometry_msgs::Vector3Stamped>("encoders", _queue_size);
        _sub_du  = _nh.subscribe("du", _queue_size, &ROSNODE::cmdDuCallback , this);
    }
    ~ROSNODE(){}
    void publishAngMsg(const float ang[3]){
        geometry_msgs::Vector3Stamped msg_ang;
        msg_ang.header.stamp = ros::Time::now();
        msg_ang.header.seq++;
        msg_ang.vector.x = ang[0];
        msg_ang.vector.y = ang[1];
        msg_ang.vector.z = ang[2];
        _pub_ang.publish(msg_ang);
    }
    void cmdDuCallback(const geometry_msgs::Twist& msg){
	ROS_INFO("x=%f, z=%f\n",msg.angular.x, msg.angular.z);
        _du[0] = msg.linear.z;
        _du[1] = msg.angular.x;
        _du[2] = msg.angular.y;
        _du[3] = msg.angular.z;
    }
};
/**************************************************************************************************
 *
**************************************************************************************************/
using namespace std;
pthread_t _Thread_Control;
bool _CloseRequested = false;
void ctrlCHandler(int signal);
struct dataStruct {
    bool is_rosnode_ready;
    float cmd[4];
    float ang[3];
    ROSNODE *rosnode;
};
/**************************************************************************************************
 *
**************************************************************************************************/
void* controlThread(void *data)
{
    // initialization -----------------------------------------------------------------------------
    printf("Start Control thread\n");
    struct dataStruct *data_;
    data_ = (struct dataStruct *) data;
    PWM *pwm;
    initializePWM(pwm, 0);
    TimeSampling ts(100);

    // Main loop ----------------------------------------------------------------------------------
    while (!data_->is_rosnode_ready);
    float dt, dtsumm = 0;
    while (!_CloseRequested)
    {
        // calculate sampling time
        dt = ts.updateTs();

        // Send PWM
        setPWMDuty(pwm, data_->rosnode->_du);

        // Display info for user every 5 second
        dtsumm += dt;
        if (dtsumm > 5) {
            dtsumm = 0;
            printf("Control thread: running fine with %4d Hz\n", int(1 / dt));
        }
    }

    // Exit procedure -----------------------------------------------------------------------------
    ctrlCHandler(0);
    printf("Exit control thread\n");
    pthread_exit(NULL);
}
/**************************************************************************************************
 *
**************************************************************************************************/
void ctrlCHandler(int signal) {
    _CloseRequested = true;
    printf("Ctrl+c have been detected\n");
}
/**************************************************************************************************
 *
**************************************************************************************************/
int main(int argc, char** argv)
{
    // initialization -----------------------------------------------------------------------------
    printf("Start Program\n");
    dataStruct data;
    data.is_rosnode_ready = false;
    signal(SIGINT, ctrlCHandler);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);

    // Ros node -----------------------------------------------------------------------------------
    printf("initiate ros node\n");
    ros::init(argc, argv, "control_test");
    ros::NodeHandle nh;
    data.rosnode = new ROSNODE (nh, "control_test");
    ros::Rate loop_rate(800);
    Encoder enc(true);
    data.is_rosnode_ready = true;
    // Main loop ----------------------------------------------------------------------------------
    while (ros::ok()){
        // read encoder and convert it to radian
        enc.updateCounts();
        enc.readAnglesRad(data.ang);

        // publish encoders' angle
        data.rosnode->publishAngMsg(data.ang);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Exit procedure -----------------------------------------------------------------------------
    printf("Close program\n");
    ctrlCHandler(0);
    //pthread_cancel(_Thread_Control);
    return(0);
}

