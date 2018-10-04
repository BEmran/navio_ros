#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"                // time sampling library
#include "../include/lib/ode.h"                         // ODE library
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
vec dyn(vec& x, vec& xdot, vec& u, vec& par){
  vec y = x;
  xdot[0] =                x[1];
  xdot[1] = -300*x[0] - 35*x[1] + 300*u[0];
  return y;
}
/**************************************************************************************************
 *
**************************************************************************************************/
class PID{
private:
  float _ei, _e0;
  float _dt;
  float _Kp, _Ki, _Kd, _Kt;
public:
  PID(){}
  ~PID(){}
  PID(float dt){
    _dt = dt;
    _ei = 0.0;
    _e0 = 0.0;
  }
  void setGains(float Kp = 1, float Ki = 0, float Kt = 0, float Kd = 0){
    _Kp = Kp;
    _Ki = Ki;
    _Kt = Kt;
    _Kd = Kd;
  }

  float update(float x, float xdes, float m, float M){
    float e = xdes - x;
    float ed = (e - _e0) / _dt;
    float u = _Kp * e + _Ki * _ei + _Kd * ed;
    float usat;
    if (u >= M)
      usat = M;
    else if (u <= m)
      usat = m;
    else
      usat = u;

    _ei += e * _dt * (_Kt * (-u+usat));
    _e0 = e;
    return u;
  }
  };
/**************************************************************************************************
 *
**************************************************************************************************/
class Rotor{
private:
  ODE ode;
  PID pid;
public:
  vec x;
  Rotor (float dt){
    ode = ODE(2, dyn);
    x = ode.getX();
    pid = PID(dt);
    pid.setGains(2.5, 20.5, 21.5/20.5, 0);
  }
  Rotor(){}
  ~Rotor(){}
  float update(float Wdes, float dt){
    // PI conrol with anti windup procedure
    Wdes = Wdes * (1.0/10000.0) * (60.0/2.0/3.14); // Scalling from RPM to 1e-4*RPM to rad/sec

    // Applay pid control
    float u = pid.update(x[0], Wdes, 0.0, 2.2);

    float usat;
    // PWM signal conditioning
    if (u > 0)
      usat = u * 0.4177 + 0.04252;
    else
      usat = 0;

    // System dynamics
    vec empty;
    vec input = {usat};
    x = ode.update(input, empty, dt);
    return usat;
  }
};
/**************************************************************************************************
 *
**************************************************************************************************/
class RosNode{
protected:
  int _queue_size;
  ros::NodeHandle _nh;
  ros::Publisher _pub_ang, _pub_w;    // publish angle encoder message
  ros::Subscriber _sub_du;    // subscriber to desired duty cycle
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
    data_->rotors[i] = Rotor(1.0/freq);
  for (int i=0; i<3 ; i++){
    data_->Wpid[i] = PID(1.0/freq);
    data_->Wpid[i].setGains(10.0, 2, 1.5, 0);
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
        du[i+1] = data_->Wpid[i].update(data_->W[i], data_->rosnode->_du[i+1], -400.0, 400.0);

      float dz = sat(du[0],    0.0, 2000.0) / 4.0;
      float dr = sat(du[1], -400.0,  400.0) / 2.0;
      float dp = sat(du[2], -400.0,  400.0) / 2.0;
      float dw = sat(du[3], -400.0,  400.0) / 4.0;

      // du to PWM
      float uPWM[4];
      uPWM[0] = dz - dp - dw;
      uPWM[1] = dz - dr + dw;
      uPWM[2] = dz + dp - dw;
      uPWM[3] = dz + dr + dw;

      // rotor control
      float r[4];
      for (int i=0; i<4 ; i++)
        r[i] = data_->rotors[i].update(uPWM[0], 1.0/freq);
      //float tmp[3] = {data_->rosnode->_du[0], res, data_->r1.x[0]};
      //data_->rosnode->publishAngMsg(tmp);

      // Send PWM
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
  float freq = 800;
  TimeSampling ts(freq);
  Encoder enc(true);
  for(int i=0; i<3; i++)
    data_->Wdyn[i] = ODE(1, dynFilter);
  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0;
  while (!_CloseRequested)
  {
    // calculate sampling time
    dt = ts.updateTs();

    // read encoder and convert it to radian
    enc.updateCounts();
    enc.readAnglesRad(data_->ang);

    //
    vec empty;
    for(int i=0; i<3; i++){
      vec ang_vec = {data_->ang[i]};
      vec tmp = data_->Wdyn[i].update(ang_vec,empty,1.0/freq);
      data_->W[i] = tmp[0];
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
