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
  for (int i=0; i < x.size(); i++){
    xdot[i] = - 50.0 * x[i] - 50.0 * 50.0 * u[i];
    y[i]    =          x[i] +        50.0 * u[i];
  }
  return y;
}
/**************************************************************************************************
 *
**************************************************************************************************/
vec dyn(vec& x, vec& xdot, vec& u, vec& par){
  vec y = x;
  xdot[0] =                x[1];
  xdot[1] = -300*x[0] - 35*x[1] + 300*u[0];
  //printf("u=%+3.1f x[0]=%+3.1f x[1]=%+3.1f\n",u[0], x[0], x[1]);
  return y;
}
/**************************************************************************************************
 *
**************************************************************************************************/
class Rotor{
private:
  ODE ode;
  float ei;
  float dt;

public:
  vec x;
  Rotor (float Dt){
    dt = Dt;
    ei = 0;
    ode = ODE(2, dyn);
    x = ode.getX();
  }
  Rotor(){}
  ~Rotor(){}
  float update(float Wdes){
    // calculate error
    float RPMe_4 = Wdes * (1.0/10000.0) * (60.0/2.0/3.14);
    float e = RPMe_4 - x[0];
    // PI conrol with anti windup procedure
    float Kp = 2.5, Ki = 20.5, Kt = 1.0/Ki;
    float u = Kp * e + Ki * ei;
    float usat;
    if (u > 2.2)
      usat = 2.2;
    else if (u < 0)
      usat = 0;
    else
      usat = u;

    ei = ei + e * dt * (1 + Kt * (-u+usat));

    // PWM signal conditioning
    if (usat > 0)
      usat = usat * 0.4177 + 0.04252;
    else
      usat = 0;

    vec empty;
    vec input={usat};
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
bool _CloseRequested = false;
void ctrlCHandler(int signal);
struct dataStruct {
  bool is_rosnode_ready;
  float ang[3];
  RosNode *rosnode;
  Rotor r0, r1, r2, r3;
  ODE Wdyn;
  vec W;
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
  TimeSampling ts(500);
  data_->r0 = Rotor(1.0/500.0);
  data_->r1 = Rotor(1.0/500.0);
  data_->r2 = Rotor(1.0/500.0);
  data_->r3 = Rotor(1.0/500.0);
  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0;
  while (!_CloseRequested)
  {
    // calculate sampling time
    dt = ts.updateTs();
    if (!data_->is_rosnode_ready)
    {
      // pid
      float du[4];
      pidW(data_->W, du, 1.0/500);

      // rotor control
      float pwm0 = data_->r0.update(du[0]);
      float pwm1 = data_->r1.update(du[1]);
      float pwm2 = data_->r2.update(du[2]);
      float pwm3 = data_->r3.update(du[3]);
      //float tmp[3] = {data_->rosnode->_du[0], res, data_->r1.x[0]};
      //data_->rosnode->publishAngMsg(tmp);

      // Send PWM
      float r[4] ={pwm0, pwm1, pwm2, pwm3};
      setPWMDuty(pwm, r);
    }
    else
      setOffPWM(pwm);
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
  ros::init(argc, argv, "control_test");
  ros::NodeHandle nh;
  data.rosnode = new RosNode (nh, "control_test");
  ros::Rate loop_rate(800);
  Encoder enc(true);
  data.Wdyn = ODE(3, dynFilter);
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
    // read encoder and convert it to radian
    enc.updateCounts();
    enc.readAnglesRad(data.ang);

    //
    vec ang_vec = {data.ang[0], data.ang[1], data.ang[2]};
    vec empty;
    data.W = data.Wdyn.update(ang_vec,empty,1/800);

    //publish encoders' angle
    data.rosnode->publishAngMsg(data.ang);
    float W[3]= {data.W[0], data.W[1], data.W[2]};
    data.rosnode->publishWMsg(W);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit procedure -----------------------------------------------------------------------------
  printf("Close program\n");
  ctrlCHandler(0);
  //pthread_cancel(_Thread_Control);
  return(0);
}

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
  //float ur = sat(Kp * er + Ki * eir + Kd * dr, 0.4, -0.4);
  //float up = sat(Kp * ep + Ki * eip + Kd * dp, 0.4, -0.4);
  float ur = 0;
  float up = 0;

  du[0] = (uz / 4.0 - up / 2.0) * 1;
  du[1] = (uz / 4.0 - ur / 2.0) * 1;
  du[2] = (uz / 4.0 + up / 2.0) * 1;
  du[3] = (uz / 4.0 + ur / 2.0) * 1;

  eir += er * 0.01;
  eip += ep * 0.01;

  dr = 0.75 * dr - 50 * r + 50 * r0;
  dp = 0.75 * dp - 50 * p + 50 * p0;
  r0 = r;
  p0 = p;
}
/**************************************************************************************************
 *
**************************************************************************************************/
void pidW(vec w, float du[], float dt){
  static float eip = 0.0, eiq = 0.0, eir = 0.0;

  float p = -w[0];
  float q = -w[1];
  float r = -w[2];

  float Kp = 10, Ki = 2, Kt= (1+1/Ki);

  float ep = 0 - p;
  float eq = 0 - q;
  float er = 0 - r;

  float uz = 400;
  float up = Kp * ep + Ki * eip;
  float uq = Kp * eq + Ki * eiq;
  float ur = Kp * er + Ki * eir;
  //float up = 0;
  //float uq = 0;
  //float ur = 0;
  float upsat = sat(up,-400,400);
  float uqsat = sat(uq,-400,400);
  float ursat = sat(ur,-400,400);

  eip += ep * dt * (Kt * (-up+upsat));
  eiq += eq * dt * (Kt * (-uq+uqsat));
  eir += er * dt * (Kt * (-ur+ursat));

  du[0] = (uz / 4.0 - uq / 2.0) * 1;
  du[1] = (uz / 4.0 - up / 2.0) * 1;
  du[2] = (uz / 4.0 + uq / 2.0) * 1;
  du[3] = (uz / 4.0 + up / 2.0) * 1;

  eip += ep * dt;
  eiq += eq * dt;
  eir += er * dt;
}