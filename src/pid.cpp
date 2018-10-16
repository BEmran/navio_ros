#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"                // time sampling library
#include "../include/lib/ode.h"                         // ODE library
#include "../include/lib/rotor.h"                       // ODE library
#include "../include/lib/pid.h"                         // PID library
#include <iostream>
#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg

/**************************************************************************************************
 *
**************************************************************************************************/
vec dynFilter(vec& x, vec& xdot, vec& u, vec& par){
  vec y(x.size());
  xdot[0] =        - 50.0 * x[0] +    1 * u[0];
  y[0]    = - 50.0 * 50.0 * x[0] + 50.0 * u[0];
  return y;
}
/**************************************************************************************************
 *
**************************************************************************************************/
class RosNode{
protected:
  int _queue_size;
  ros::NodeHandle _nh;
  ros::Publisher _pub_ang, _pub_w, _pub_du;	// publish angle encoder message
  ros::Subscriber _sub_du,_sub_ang;    			// subscriber to desired duty cycle
  std::string _name;
public:
  float _du[4], _ang[3];
  RosNode(){}
  RosNode(ros::NodeHandle nh, std::string name){
    _du[0] = 0.0; _du[1] = 0.0; _du[2] = 0.0; _du[3] = 0.0;
    _ang[0] = 0.0; _ang[1] = 0.0; _ang[2] = 0.0;
    _nh = nh;
    _name = name;
    _queue_size = 10;
    _pub_ang = _nh.advertise <geometry_msgs::Vector3Stamped>("encoders", _queue_size);
    _pub_w   = _nh.advertise <geometry_msgs::Vector3Stamped>("w", _queue_size);
    _pub_du  = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/du", _queue_size);
    _sub_du   = _nh.subscribe("du", _queue_size, &RosNode::cmdDuCallback , this);
    _sub_ang  = _nh.subscribe("ang", _queue_size, &RosNode::cmdAngCallback , this);
  }
  ~RosNode(){}
  void publishAngMsg(const float ang[3]){
    geometry_msgs::Vector3Stamped msg_ang;
    msg_ang.header.stamp = ros::Time::now();
    msg_ang.header.seq++;
    msg_ang.vector.x = ang[0];
    msg_ang.vector.y = ang[1];
    msg_ang.vector.z = ang[2];
    _pub_ang.publish(msg_ang);
  }
  void publishWMsg(const float w[3]){
    geometry_msgs::Vector3Stamped msg_w;
    msg_w.header.stamp = ros::Time::now();
    msg_w.header.seq++;
    msg_w.vector.x = w[0];
    msg_w.vector.y = w[1];
    msg_w.vector.z = w[2];
    _pub_w.publish(msg_w);
  }
  void publishDuMsg(const float du[3]){
    geometry_msgs::Vector3Stamped msg_du;
    msg_du.header.stamp = ros::Time::now();
    msg_du.header.seq++;
    msg_du.vector.x = du[0];
    msg_du.vector.y = du[1];
    msg_du.vector.z = du[2];
    _pub_du.publish(msg_du);
  }
  void cmdDuCallback(const geometry_msgs::Twist& msg){
    ROS_INFO("x=%f, y=%f, z=%f\n",msg.angular.x, msg.angular.y, msg.angular.z);
    _du[0] = msg.linear.z;
    _du[1] = msg.angular.x;
    _du[2] = msg.angular.y;
    _du[3] = msg.angular.z;
  }
  void cmdAngCallback(const geometry_msgs::Vector3Stamped& msg){
    ROS_INFO("x=%f, y=%f, z=%f\n",msg.vector.x, msg.vector.y, msg.vector.z);
    _ang[0] = msg.vector.x;
    _ang[1] = msg.vector.y;
    _ang[2] = msg.vector.z;
  }
};
/**************************************************************************************************
 *
**************************************************************************************************/
using namespace std;
pthread_t _Thread_Control;
pthread_t _Thread_Sensors;
bool _CloseRequested = false;
void ctrlCHandler(int signal);
struct dataStruct {
  bool is_rosnode_ready;
  float du[4], ang[3];
  float du_cmd[4], ang_cmd[3];
  float W[3];
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

  Navio nav;
  nav.initializePWM(0);

  float freq = 100;
  TimeSampling ts(freq);

  PID Wpid[3];
  PID Apid[3];
  for (int i=0; i<3 ; i++){
    Wpid[i].setGains(400.0, 200.0, 0, 2);
    Apid[i].setGains( 5.0,  3.0, 0, 0.1);
  }
  Wpid[2].setGains(800.0, 500.0, 0, 2);
  // du max and minimum values
  float min[] = {   0.0, -400.0, -400.0, -400.0};
  float max[] = {2000.0, +400.0, +400.0, +400.0};
  float du[4] ={0, 0, 0, 0};
  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0;
  while (!_CloseRequested)
  {
    // calculate sampling time
    dt = ts.updateTs();
    if (data_->is_rosnode_ready)
    {
      data_->du[0] = data_->du_cmd[0];
      //du[3] = data_->du_cmd[3];
      for (int i=0; i<3 ; i++){
        float tmp = Apid[i].update(data_->ang[i], data_->ang_cmd[i],  -2.0,   2.0, dt);
        data_->du[i+1] = Wpid[i].update(data_->W[i],                 tmp,-400.0, 400.0, dt);
      }
      //send to motor
      nav.toMotor(data_->du, min, max, 1/freq);
    }
    else
      nav.send(du);

    // Display info for user every 5 second
    dtsumm += dt;
    if (dtsumm > 5) {
      dtsumm = 0;
      printf("Control thread: running fine with %4d Hz\n", int(1 / dt));
    }
  }

  // Exit procedure -----------------------------------------------------------------------------
  nav.setOffPWM();
  ctrlCHandler(0);
  printf("Exit control thread\n");
  pthread_exit(NULL);
}
/**************************************************************************************************
 *
**************************************************************************************************/
void* sensorsThread(void *data)
{
  // initialization -----------------------------------------------------------------------------
  printf("Start Sensors thread\n");
  struct dataStruct *data_;
  data_ = (struct dataStruct *) data;
  float freq = 100;
  TimeSampling ts(freq);
  Encoder enc(true);
  ODE Wdyn[3];
  for(int i=0; i<3; i++)
    Wdyn[i] = ODE(1, dynFilter);
  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0, dtsumEnc = 0;
  while (!_CloseRequested)
  {
    // calculate sampling time
    dt = ts.updateTs();

    dtsumEnc += dt;
    if (dtsumEnc > 0.01){
      dtsumEnc = 0;
      // read encoder and convert it to radian
      enc.updateCounts();
      enc.readAnglesRad(data_->ang);
      // differentiate encoder data
      for(int i=0; i<3; i++){
        data_->ang[i] = -data_->ang[i];
        vec ang_vec = {data_->ang[i]};
        vec tmp = Wdyn[i].update(ang_vec,0.01);
        data_->W[i] = tmp[0];
      }
    }

    // Display info for user every 5 second
    dtsumm += dt;
    if (dtsumm > 5) {
      dtsumm = 0;
      printf("Sensor thread: running fine with %4d Hz\n", int(1 / dt));
    }
  }

  // Exit procedure -----------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Exit Sensors thread\n");
  pthread_exit(NULL);
}/**************************************************************************************************
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
  pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);

  // Ros node -----------------------------------------------------------------------------------
  printf("initiate ros node\n");
  ros::init(argc, argv, "control_test");
  ros::NodeHandle nh;
  RosNode rosnode(nh, "control_test");
  float freq = 50;
  ros::Rate loop_rate(freq);
  // ----------------------------------------------------------------------------------------------
  int x = 0;
  while (x == 0) {
    printf("Enter 1 to start control\n");
    cin >> x;
    sleep(1);
  }
  data.is_rosnode_ready = true;
  // Main loop ----------------------------------------------------------------------------------
  while (ros::ok()){
    //publish encoders' angle
    rosnode.publishAngMsg(data.ang);
    rosnode.publishWMsg(data.W);
    rosnode.publishDuMsg(data.du);
    for (int i=0; i<4; i++)
      data.du_cmd[i] = rosnode._du[i];
    for (int i=0; i<3; i++)
      data.ang_cmd[i] = rosnode._ang[i];
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit procedure -----------------------------------------------------------------------------
  printf("Close program\n");
  ctrlCHandler(0);
  pthread_cancel(_Thread_Control);
  pthread_cancel(_Thread_Sensors);
  return(0);
}
