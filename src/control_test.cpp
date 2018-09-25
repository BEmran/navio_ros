#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"               // time sampling library
#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg
using namespace std;
pthread_t _Thread_Control;
bool _CloseRequested = false;
void ctrlCHandler(int signal);
/**************************************************************************************************
 *
**************************************************************************************************/
class ROSNODE{
protected:
    int _queue_size;
    ros::NodeHandle _nh;
    ros::Publisher _pub_ang;    // publish imu encoder message
    ros::Subscriber _sub_du;    // subscriber to desired duty cycle message from user
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
    void cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        _du[0] = msg->twist.linear.z;
        _du[1] = msg->twist.angular.x;
        _du[2] = msg->twist.angular.y;
        _du[3] = msg->twist.angular.z;
    }
};
/**************************************************************************************************
 *
**************************************************************************************************/
struct dataStruct {
    float cmd[4];
    float ang[3];
    ROSNODE *rosnode;
};
/**************************************************************************************************
 *
**************************************************************************************************/
void* controlThread(void *data)
{
    // Initialize mapping data
    struct dataStruct *data_;
    data_ = (struct dataStruct *) data;

    PWM *pwm;
    initializePWM(pwm, 0);
    TimeSampling ts(100);
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
    dataStruct data;
    signal(SIGINT, ctrlCHandler);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);

    ros::init(argc, argv, "control_test");
    ros::NodeHandle nh;
    data.rosnode = new ROSNODE (nh, "control_test");
    ros::Rate loop_rate(400);
    Encoder enc(true);

    while (ros::ok()){
        enc.updateCounts();
        enc.readAnglesRad(data.ang);
        data.rosnode->publishAngMsg(data.ang);

        ros::spinOnce();
        loop_rate.sleep();
    }
    printf("Close program\n");
    ctrlCHandler(0);
    pthread_cancel(_Thread_Control);
    return(0);
}

