#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"               // time sampling library
#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg

using namespace std;
float sat(float x, float max, float min);
/**************************************************************************************************
 *
**************************************************************************************************/
class ROSNODE{
protected:
  int _queue_size;
  ros::NodeHandle _nh;
  ros::Publisher _pub_ang;
  ros::Subscriber _sub_cmd;
  std::string _name;
public:
  float _cmd[3];
  ROSNODE(){}
  ROSNODE(ros::NodeHandle nh, std::string name){
      _cmd[0] = 0.0; _cmd[1] = 0.0; _cmd[2] = 0.0;
      _nh = nh;
      _name = name;
      _queue_size = 10;
      _pub_ang = _nh.advertise <geometry_msgs::Vector3Stamped>("encoders", _queue_size);
      _sub_cmd  = _nh.subscribe("cmd" , _queue_size, &ROSNODE::cmdCallback , this);

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
  void cmdCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
      _cmd[0] = msg->vector.x;
      _cmd[1] = msg->vector.y;
      _cmd[2] = msg->vector.z;
  }
};
/**************************************************************************************************
 *
**************************************************************************************************/
void pid(float ang[], float des[], float du[]){
    static float eir = 0.0, eip = 0.0;
    static float dr = 0.0, dp = 0.0;
    static float r0 = 0.0, p0 = 0.0;

    float r = -ang[0];
    float p = -ang[1];

    float Kp = 0.2, Ki = 0.05, Kd = 0.09;

    float er = des[0] - r;
    float ep = des[1] - p;

    float uz = 0.5 * 4;
    float ur = sat(Kp * er + Ki * eir + Kd * dr, 0.4, -0.4);
    float up = sat(Kp * ep + Ki * eip + Kd * dp, 0.4, -0.4);

    du[0] = (uz / 4.0 - up / 2.0) * 1;
    du[1] = (uz / 4.0 - ur / 2.0) * 1;
    du[2] = (uz / 4.0 + up / 2.0) * 1;
    du[3] = (uz / 4.0 + ur / 2.0) * 1;

    eir += er * 0.01;
    eip += ep * 0.01;

    dr = 0.7788 * dr + 50 * r - 50 * r0;
    dp = 0.7788 * dp + 50 * p - 50 * p0;

    r0 = r;
    p0 = p;
}
/**************************************************************************************************
 *
**************************************************************************************************/
float sat(float x, float max, float min){
    float y;
    if (x >= max)
        y = max;
    else if (x <= min)
        y = min;
    else
        y = x;
    return y;
}
/**************************************************************************************************
 *
**************************************************************************************************/
using namespace std;
pthread_t _Thread_Control;
bool _CloseRequested = false;
void ctrlCHandler(int signal);
struct dataStruct {
    bool is_rosnode_ready;
    float ang[3];
    float du[4];
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
    TimeSampling ts(200);

    // Main loop ----------------------------------------------------------------------------------
    while (!data_->is_rosnode_ready);
    float dt, dtsumm = 0;
    while (!_CloseRequested)
    {
        // calculate sampling time
        dt = ts.updateTs();

        // Send PWM
        setPWMDuty(pwm, data_->du);

        // Display info for user every 5 second
        dtsumm += dt;
        if (dtsumm > 5) {
            dtsumm = 0;
            printf("Control thread: running fine with %4d Hz\n", int(1 / dt));
        }
    }

    // Exit procedure -----------------------------------------------------------------------------
    setOffPWM(pwm);
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
    ros::init(argc, argv, "pid");
    ros::NodeHandle nh;
    data.rosnode = new ROSNODE (nh, "pid");
    ros::Rate loop_rate(200);
    Encoder enc(true);
    data.is_rosnode_ready = true;
    float du[4];
    // Main loop ----------------------------------------------------------------------------------
    while (ros::ok()){
        // read encoder and convert it to radian
        enc.updateCounts();
        enc.readAnglesRad(data.ang);
        // calculate pid values
        data.rosnode->publishAngMsg(data.ang);
        pid(data.ang, data.rosnode->_cmd, du);

        for (int i=0; i<4; i++)
            data.du[i] = du[i];
        // spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Exit procedure -----------------------------------------------------------------------------
    return(0);
}

