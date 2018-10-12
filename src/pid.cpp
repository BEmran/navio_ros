#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"                // time sampling library
#include "../include/lib/ode.h"                         // ODE library
#include "../include/lib/rotor.h"                         // ODE library
#include <iostream>

#include <signal.h>                         // signal ctrl+c
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"     // du msg
#include "geometry_msgs/Vector3Stamped.h"   // encoder and RPY msg

void pidW(vec w, float du[], float dt);
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
  ros::Subscriber _sub_du;    			// subscriber to desired duty cycle
  std::string _name;
public:
  float _du[4];
  RosNode(){}
  RosNode(ros::NodeHandle nh, std::string name){
    _du[0] = 0.0; _du[1] = 0.0; _du[2] = 0.0; _du[3] = 0.0;
    _nh = nh;
    _name = name;
    _queue_size = 10;
    _pub_ang = _nh.advertise <geometry_msgs::Vector3Stamped>("encoders", _queue_size);
    _pub_w   = _nh.advertise <geometry_msgs::Vector3Stamped>("w", _queue_size);
    _pub_du  = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/du", _queue_size);

    _sub_du  = _nh.subscribe("du", _queue_size, &RosNode::cmdDuCallback , this);
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
pthread_t _Thread_Sensors;
bool _CloseRequested = false;
void ctrlCHandler(int signal);
struct dataStruct {
  bool is_rosnode_ready;
  float ang[3];
  RosNode *rosnode;
  Rotor rotors[4];
  PID Wpid[3];
  ODE Wdyn[3];
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
  PWM *pwm;
  initializePWM(pwm, 0);
  float freq = 100;
  TimeSampling ts(freq);
  for (int i=0; i<4 ; i++)
    data_->rotors[i] = Rotor();
  for (int i=0; i<3 ; i++){
    data_->Wpid[i] = PID();
    data_->Wpid[i].setGains(400.0, 600.0, 2, 0);
  }
  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0;
  while (!_CloseRequested)
  {
    // calculate sampling time
    dt = ts.updateTs();
    if (data_->is_rosnode_ready)
    {
      float du[4];

      du[0] = data_->rosnode->_du[0];
      for (int i=0; i<3 ; i++)
        du[i+1] = data_->Wpid[i].update(data_->W[i], data_->rosnode->_du[i+1], -400.0, 400.0, 1.0/freq);

      float dz = sat(du[0],    0.0, 2000.0) / 4.0;
      float dr = sat(du[1], -400.0,  400.0) / 2.0;
      float dp = sat(du[2], -400.0,  400.0) / 2.0;
      float dw = sat(du[3], -400.0,  400.0) / 4.0;

      float msg[3] = {dr, dp, dw};
      data_->rosnode->publishDuMsg(msg);

      // du to PWM
      float uPWM[4];
      uPWM[0] = dz - dp - dw;
      uPWM[1] = dz - dr + dw;
      uPWM[2] = dz + dp - dw;
      uPWM[3] = dz + dr + dw;

      // rotor control
      float r[4];
      for (int i=0; i<4 ; i++)
        r[i] = data_->rotors[i].update(uPWM[i], 1.0/freq);
      //float tmp[3] = {data_->rosnode->_du[0], res, data_->r1.x[0]};

      // send PWM
      setPWMDuty(pwm, r);
    }
    else{
      float r[4] ={0, 0, 0, 0};
      setPWMDuty(pwm, r);
    }
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
void* sensorsThread(void *data)
{
  // initialization -----------------------------------------------------------------------------
  printf("Start Sensors thread\n");
  struct dataStruct *data_;
  data_ = (struct dataStruct *) data;
  PWM *pwm;
  initializePWM(pwm, 0);
  float freq = 100;
  TimeSampling ts(freq);
  Encoder enc(true);
  for(int i=0; i<3; i++)
    data_->Wdyn[i] = ODE(1, dynFilter);
  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0, dtsumEnc = 0;
  while (!_CloseRequested)
  {
    // calculate sampling time
    dt = ts.updateTs();

    // read encoder and convert it to radian
    //enc.updateCounts();
    //enc.readAnglesRad(data_->ang);

    //
    vec empty;
    for(int i=0; i<3; i++){
      vec ang_vec = {-data_->ang[i]};
      vec tmp = data_->Wdyn[i].update(ang_vec,empty,1.0/freq);
      data_->W[i] = tmp[0];
    }
    dtsumEnc += dt;
    if (dtsumEnc > 0.01){
      dtsumEnc = 0;
      // read encoder and convert it to radian
      enc.updateCounts();
      enc.readAnglesRad(data_->ang);
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
  data.rosnode = new RosNode (nh, "control_test");
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
    data.rosnode->publishAngMsg(data.ang);
    data.rosnode->publishWMsg(data.W);
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
