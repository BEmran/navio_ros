/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */
#ifndef TESTBED
#define TESTBED

/**************************************************************************************************
Header files
**************************************************************************************************/
#include <signal.h>                               // signal ctrl+c
#include <stdio.h>                                // printf
#include "lib/TimeSampling.h"                     // time sampling library
#include <testbed_navio/ros_node.h>               // ros node class
#include <testbed_navio/navio_interface.h>        // navio interface pwm, sensors ...
#include "lib/ode.h"

/**************************************************************************************************
Global variables
**************************************************************************************************/
#define _SENSORS_FREQ   400                       // Sensors thread frequency in Hz
#define _ROSNODE_FREQ   100                       // Rosnode thread frequency in Hz
#define _CONTROL_FREQ   200                       // Control thread frequency in Hz

pthread_t _Thread_Sensors;
pthread_t _Thread_Rosnode;
pthread_t _Thread_Control;

bool _CloseRequested = false;                     // close request for ctrl+c
using namespace std;
typedef vector<float> vec;
/**************************************************************************************************
Define structures
**************************************************************************************************/
struct controlStruct {                            // getting parameters form rosparm
  std::vector<double> kp;
  std::vector<double> ki;
  std::vector<double> kd;
  std::vector<double> kr;
  std::vector<double> kw;
};
struct dataStruct {                               // main data structure
  bool is_control_ready;
  bool is_rosnode_ready;
  bool is_sensors_ready;

  float du[4];                // output PWM signal
  int enc_dir[3];
  float enc_ang_bias[3];
  float pwm_offset[4];
  float record[25];           // stored data to print each samplig time
  float enc_angle[3];         // store encoder angle in rad
  float rpy[3];
  float quat[4];
  float du_max[4], du_min[4]; // maximum and minimum du values
  float info[5];              // extra information to be recorded
  vec w;
  ODE wSys;

  FILE *file;

  RosNode* rosnode;
  Sensors* sensors;
  Encoder* encoder;
  controlStruct angConGain;

  int argc;
  char** argv;
};

/**************************************************************************************************
Functions prototype
**************************************************************************************************/
dataStruct* mainInitialize(int argc, char** argv);
void ctrlCHandler(int signal);
void *sensorsThread(void *data);
void *rosNodeThread(void *data);
void *controlThread(void *data);
void initializeParams(ros::NodeHandle& n, dataStruct* data);
void printRecord(FILE* file, float data[]);
void control(dataStruct* data, float dt);
vec diffDyn(vec& x, vec& xdot, vec& u, vec& par);
/**************************************************************************************************
 ctrlCHandler: Detect ctrl+c to quit program
**************************************************************************************************/
void ctrlCHandler(int signal) {
  _CloseRequested = true;
  printf("Ctrl+c have been detected\n");
}
/**************************************************************************************************
 mainInitialize: main initialization
**************************************************************************************************/
dataStruct* mainInitialize(int argc, char** argv)
{
  // Welcome msg ----------------------------------------------------------------------------------
  printf("Start Program...\n");
  // conncet ctrl+c Handler
  signal(SIGINT, ctrlCHandler);

  // Define main variables ------------------------------------------------------------------------
  struct dataStruct* data;
  data->argc = argc;
  data->argv = argv;
  data->is_control_ready = false;
  data->is_rosnode_ready = false;
  data->is_sensors_ready = false;

  TimeSampling st(_ROSNODE_FREQ);
  for (int i = 0; i < 25; ++i) {
    data->record[i] = 0.0;       // initialize record data with zeros
  }

  // Start threads --------------------------------------------------------------------------------
  pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
  pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);

  // Create new record file -----------------------------------------------------------------------
  char file_name[64];
  int fileNumber = 0;
  // find avaliable file name & number
  do {
    sprintf(file_name, "/home/pi/testbed_data_%.2d.csv", fileNumber++);
  } while (access(file_name, F_OK) == 0);
  // open avaliable file
  data->file = fopen(file_name, "w");
  // check file
  if (data->file == NULL) {
    printf("Error creating file!\n");
    exit(1);
  }

  // Display Information for user
  printf("A file successfully created to record testbed data\n");
  printf("Record file path and name: %s\n",file_name);

  // Record starting time of test -------------------------------------------
  time_t rawtime;
  time (&rawtime);
  struct tm * timeinfo = localtime (&rawtime);
  fprintf(data->file,"Current local time and date: %s", asctime(timeinfo));

  // Print data header
  fprintf(data->file, "time,"
                     "ax,ay,az,"
                     "gx,gy,gz,"
                     "mx,my,mz,"
                     "enc0,enc1,enc2,"
                     "enc0dot,enc1dot,enc2dot,"
                     "ur,up,uw,uz,"
                     "d0,d1,d2,d3,d4\n");

  // Initialize ROS -----------------------------------------------------------
  while(!data->is_sensors_ready);                 // wait for sensor thread to be ready
  string name = "testbed_navio";                  // define ros node name
  ros::init(data->argc, data->argv, name);        // initialize ros
  ros::NodeHandle nh;                             // define ros handle
  data->rosnode = new RosNode (nh,name);          // define RosNode object
  initializeParams(nh, data);                     // initialize ros parameter

  // Wait for user to be ready ------------------------------------------------
  while(!data->is_control_ready || !data->is_sensors_ready);
  int x = 0;
  while (x == 0) {
    printf("Enter 1 to start control\n");
    cin >> x;
    sleep(1);
  }
  data->is_rosnode_ready = true;
}

/**************************************************************************************************
 controlThread: Perform control loop and send PWM output
**************************************************************************************************/
void *controlThread(void *data) {

  // Starting Control thread ----------------------------------------------------------------------
  printf("Start Control thread\n");

  // Initialize mapping data
  struct dataStruct *my_data;
  my_data = (struct dataStruct *) data;

  // Initialize sampling time
  TimeSampling ts(_CONTROL_FREQ);

  // Initialize PWM
  PWM* pwm;
  initializePWM(pwm, 0);
  my_data->du[0] = 0.0;
  my_data->du[1] = 0.0;
  my_data->du[2] = 0.0;
  my_data->du[3] = 0.0;

  // Announce control thread is ready
  printf("control is ready\n");
  my_data->is_control_ready = true;

  // Wait for main function to be ready
  while(!my_data->is_mainfun_ready);

  // Main loop ------------------------------------------------------------------------------------
  float dt, dtsumm = 0;
  while (!_CloseRequested) {

    // Calculate sampling time
    dt = ts.updateTs();

    // Check if rosnode is ready
    if (!my_data->is_mainfun_ready) {
      // Check sampling
      if (dt < 0.02) {
        // Run control function
        control(my_data, dt);
        // Send data to motors
        sendToMotors(pwm, my_data->du, my_data->du_min, my_data->du_max);
      }
      else {
        printf("Control thread: sampling time is too big = %f\n", dt);
        // Send minimum pwm value
        setMinPWM(pwm);
      }
    }
    else {
      // Send minimum pwm value
      setMinPWM(pwm);
    }

    // Display info for user every 5 second
    dtsumm += dt;
    if (dtsumm > 5.0) {
      dtsumm = 0;
      printf("Control thread: running with %4d Hz\n", int(1 / dt));
    }
  }

  // Exit procedure -----------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Exit control thread\n");
  pthread_exit(NULL);
}

/**************************************************************************************************
 sensorsThread: read navio sensors (IMU +...) and perfourm AHRS
 *************************************************************************************************/
void *sensorsThread(void *data) {

  // Initialize sensors node thread ---------------------------------------------------------------
  printf("Start Sensors thread\n");

  // Initialize mapping data
  struct dataStruct *my_data;
  my_data = (struct dataStruct *) data;

  // Initialize IMU
  my_data->sensors = new Sensors("mpu", false);
  my_data->sensors->getInitialOrientation();
  float tmpx = my_data->sensors->init_Orient[0];
  float tmpy = my_data->sensors->init_Orient[1];
  float tmpz = my_data->sensors->init_Orient[2];
  float tmp_bias = atan2(tmpy , tmpz);
  if (tmp_bias > 0)
    my_data->enc_ang_bias[0] = (3.14 - tmp_bias);
  else
    my_data->enc_ang_bias[0] = (3.14 + tmp_bias);
  my_data->enc_ang_bias[1] = -atan2(- tmpx , sqrt(tmpy * tmpy + tmpz * tmpz));
  my_data->enc_ang_bias[2] = 0.0;
  printf("Correct in roll= %5.5f\t  pitch= %5.5f\n", my_data->enc_ang_bias[0], my_data->enc_ang_bias[1]);
  //Initialize encoder
  my_data->encoder = new Encoder(0);
  my_data->enc_dir[0] = 1;
  my_data->enc_dir[1] = 1;
  my_data->enc_dir[2] = 1;
  // Announce sensors thread is ready
  my_data->is_sensors_ready = true;

  // Main loop ------------------------------------------------------------------------------------
  TimeSampling ts(_SENSORS_FREQ);
  float dt, dtsumm = 0;
  printf("sensor is ready now\n");
  while (!_CloseRequested) {
    dt = ts.updateTs();                 // calculate sampling time
    my_data->sensors->update();         // update Sensor
    my_data->encoder->updateCounts();   // update encoders counts
    my_data->encoder->readAnglesRad(my_data->enc_angle);
    my_data->enc_angle[0] = (my_data->enc_angle[0] - my_data->enc_ang_bias[0]) * my_data->enc_dir[0]; // change angle direction
    my_data->enc_angle[1] = (my_data->enc_angle[1] - my_data->enc_ang_bias[1]) * my_data->enc_dir[1]; // change angle direction
    my_data->enc_angle[2] = (my_data->enc_angle[2] - my_data->enc_ang_bias[2]) * my_data->enc_dir[2]; // change angle direction
    vec enc_angle_tmp(my_data->enc_angle, my_data->enc_angle+3);
    vec p{50,50,50};
    my_data->w = my_data->wSys.update(enc_angle_tmp, p, 0.005);

    // Update record values
    my_data->record[1] = my_data->sensors->imu.ax;
    my_data->record[2] = my_data->sensors->imu.ay;
    my_data->record[3] = my_data->sensors->imu.az;
    my_data->record[4] = my_data->sensors->imu.gx;
    my_data->record[5] = my_data->sensors->imu.gy;
    my_data->record[6] = my_data->sensors->imu.gz;
    my_data->record[7] = my_data->sensors->imu.mx;
    my_data->record[8] = my_data->sensors->imu.my;
    my_data->record[9] = my_data->sensors->imu.mz;
    my_data->record[10] = my_data->enc_angle[0];
    my_data->record[11] = my_data->enc_angle[1];
    my_data->record[12] = my_data->enc_angle[2];
    my_data->record[13] = my_data->rpy[0];
    my_data->record[14] = my_data->rpy[1];
    my_data->record[15] = my_data->rpy[2];
    my_data->record[16] = my_data->du[0];
    my_data->record[17] = my_data->du[1];
    my_data->record[18] = my_data->du[2];
    my_data->record[19] = my_data->du[3];
    my_data->record[20] = my_data->info[0];
    my_data->record[21] = my_data->info[1];
    my_data->record[22] = my_data->info[2];
    my_data->record[23] = my_data->info[3];
    my_data->record[24] = my_data->info[4];
    // Record data in a file
    printRecord(my_data->file, my_data->record);

    // Display info for user every 5 second
    dtsumm += dt;
    if (dtsumm > 5) {
      dtsumm = 0;
      printf("Sensors thread: running fine with %4d Hz\n", int(1 / dt));
    }
  }

  // Exit procedure -------------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Exit sensor thread\n");
  pthread_exit(NULL);
}

/**************************************************************************************************
 rosNodeThread: ROS Node thread
**************************************************************************************************/
void *rosNodeThread(void *data) {

  // Initialize ros node thread -------------------------------------------------------------------
  printf("Start ROS Node thread\n");

  // Initialize mapping data
  struct dataStruct *my_data;
  my_data = (struct dataStruct *) data;

  // Initialize ROS
  while(!my_data->is_sensors_ready);              // wait for sensor thread to be ready
  string name = "testbed_navio";                  // define ros node name
  ros::init(my_data->argc, my_data->argv, name);  // initialize ros
  ros::NodeHandle nh;                             // define ros handle
  my_data->rosnode = new RosNode (nh,name);       // define RosNode object
  ros::Rate loop_rate(_ROSNODE_FREQ);             // define ros frequency
  initializeParams(nh, my_data);                  // initialize ros parameter

  printf("ros is ready\n");
  my_data->is_rosnode_ready = true;

  // Main loop ----------------------------------------------------------------------------------
  while (ros::ok() && !_CloseRequested)
  {
    float gyro[3]= {my_data->sensors->imu.gx,my_data->sensors->imu.gy,my_data->sensors->imu.gz};
    float acc[3]= {my_data->sensors->imu.ax,my_data->sensors->imu.ay,my_data->sensors->imu.az};
    float mag[3]= {my_data->sensors->imu.mx,my_data->sensors->imu.my,my_data->sensors->imu.mz};
    float a[3]={1,2,3};
    float b[4]={4,5,6,7};
    my_data->rosnode->publishAllMsgs(gyro,
                                     acc,
                                     my_data->quat,
                                     mag,
                                     my_data->enc_angle,
                                     my_data->rpy,
                                     my_data->du);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit procedure -----------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Exit ROS Node thread\n");
  pthread_exit(NULL);
}

/**************************************************************************************************
initializeParams: initialize parameter using rosparm package
**************************************************************************************************/
void initializeParams(ros::NodeHandle& n, dataStruct* data){

  // Get Control Parameter ------------------------------------------------------------------------
  // Kp angle gains
  if (n.getParam("testbed/control/angle/gains/kp", data->angConGain.kp))
    ROS_INFO("Found angle control kp gains");
  else {
    ROS_INFO("Can't find angle control kp gains");
    data->angConGain.kp.assign(0,0.4);
    data->angConGain.kp.assign(1,0.4);
    data->angConGain.kp.assign(2,0.8);
  }
  // Ki angle gains
  if (n.getParam("testbed/control/angle/gains/ki", data->angConGain.ki))
    ROS_INFO("Found angle control ki gains");
  else {
    ROS_INFO("Can't find angle control ki gains");
    data->angConGain.ki.assign(0,1.0);
    data->angConGain.ki.assign(1,1.0);
    data->angConGain.ki.assign(2,2.0);
  }

  // Kd angle gains
  if (n.getParam("testbed/control/angle/gains/kd", data->angConGain.kd))
    ROS_INFO("Found angle control kd gains");
  else {
    ROS_INFO("Can't find angle control kd gains");
    data->angConGain.kd.assign(0,1.0);
    data->angConGain.kd.assign(1,1.0);
    data->angConGain.kd.assign(2,2.0);
  }

  // Kr gains
  if (n.getParam("testbed/control/angle/gains/kr", data->angConGain.kr))
    ROS_INFO("Found angle control kr gains");
  else {
    ROS_INFO("Can't find angle control kr gains");
    data->angConGain.kr.assign(0,0.2);
    data->angConGain.kr.assign(1,0.2);
    data->angConGain.kr.assign(2,0.2);
  }
  // Kw gains
  if (n.getParam("testbed/control/angle/gains/kw", data->angConGain.kw))
    ROS_INFO("Found angle control kw gains");
  else {
    ROS_INFO("Can't find angle control kw gains");
    data->angConGain.kw.assign(0,0.1);
    data->angConGain.kw.assign(1,0.1);
    data->angConGain.kw.assign(2,0.1);
  }

  // Get Motors Offsets ---------------------------------------------------------------------------
  std::vector<double> offset;
  if (n.getParam("testbed/motors/offset", offset)){
    ROS_INFO("Found motor offset gains");
    data->pwm_offset[0] = offset[0];
    data->pwm_offset[1] = offset[1];
    data->pwm_offset[2] = offset[2];
    data->pwm_offset[3] = offset[3];
  }
  else {
    ROS_INFO("Can't find offset of the motors");
    data->pwm_offset[0] = 0.0;
    data->pwm_offset[1] = 0.0;
    data->pwm_offset[2] = 0.0;
    data->pwm_offset[3] = 0.0;
  }

  // Get du max min values ------------------------------------------------------------------------
  std::vector<double> du;
  if (n.getParam("ground_station/du_command/thrust", du)){
    data->du_min[0] = du[0];
    data->du_max[0] = du[1];
  }
  else {
    data->du_min[0] = 0.0;
    data->du_max[0] = 2.0;
  }
  if (n.getParam("testbed/du_command/roll", du)){
    data->du_min[1] = du[0];
    data->du_max[1] = du[1];
  }
  else {
    data->du_min[1] = -0.2;
    data->du_max[1] = +0.2;
  }
  if (n.getParam("testbed/du_command/pitch", du)){
    data->du_min[2] = du[0];
    data->du_max[2] = du[1];
  }
  else {
    data->du_min[2] = -0.2;
    data->du_max[2] = +0.2;
  }
  if (n.getParam("testbed/du_command/yaw", du)){
    data->du_min[3] = du[0];
    data->du_max[3] = du[1];
  }
  else {
    data->du_min[3] = -0.1;
    data->du_max[3] = +0.1;
  }

  // Get encoderes direction ----------------------------------------------------------------------
  std::vector<double> enc_dir;
  if (n.getParam("testbed/encoders_direction", enc_dir)){
    ROS_INFO("Found encoders direction");
  }
  else {
    ROS_INFO("Can't find encoders direction");
    enc_dir.assign(0,1);
    enc_dir.assign(1,1);
    enc_dir.assign(2,1);
  }
  data->enc_dir[0] = enc_dir[0];
  data->enc_dir[1] = enc_dir[1];
  data->enc_dir[2] = enc_dir[2];

  // print result ---------------------------------------------------------------------------------
  ROS_INFO("control kp gains are set to: kp[0] %f, kp[1] %f, kp[2] %f\n",
           data->angConGain.kp[0],data->angConGain.kp[1],data->angConGain.kp[2]);
  ROS_INFO("Control ki gains are set to: ki[0] %f, ki[1] %f, ki[2] %f\n",
           data->angConGain.ki[0],data->angConGain.ki[1],data->angConGain.ki[2]);
  ROS_INFO("Control kd gains are set to: kd[0] %f, kd[1] %f, kd[2] %f\n",
           data->angConGain.kd[0],data->angConGain.kd[1],data->angConGain.kd[2]);
  ROS_INFO("Motor offset they are set to: m0[0] %f, m1[1] %f, m2[2] %f, m3[3] %f\n",
           data->pwm_offset[0],data->pwm_offset[1],data->pwm_offset[2],data->pwm_offset[3]);
  ROS_INFO("Maximum and minimum values for du command signals are set to:\n");
  ROS_INFO(" - Thrust = [%+6.2f - %+6.2f] \n",data->du_min[0],data->du_max[0]);
  ROS_INFO(" - Roll   = [%+6.2f - %+6.2f] \n",data->du_min[1],data->du_max[1]);
  ROS_INFO(" - Pitch  = [%+6.2f - %+6.2f] \n",data->du_min[2],data->du_max[2]);
  ROS_INFO(" - Yaw    = [%+6.2f - %+6.2f] \n",data->du_min[3],data->du_max[3]);
  ROS_INFO("Encoders direction:\n");
  ROS_INFO(" - roll  = %+d\n", data->enc_dir[0]);
  ROS_INFO(" - pitch = %+d\n", data->enc_dir[1]);
  ROS_INFO(" - yaw   = %+d\n", data->enc_dir[2]);
}

/**************************************************************************************************
printRecord: print recorded data in a file
**************************************************************************************************/
void printRecord(FILE* file, float data[]){

  int size = 25;                                  // record data 0-24
  struct timeval tv;                              // Timing data
  char buf[1024];                                 // Record data header
  char *pos = buf;

  // Calculate delta time
  gettimeofday(&tv, NULL);
  long tmp = 1000000L * tv.tv_sec + tv.tv_usec;
  data[0] = tmp / 1000000.0;

  // get data stored in data array
  for (int i = 0; i < size; ++i) {
    pos += sprintf(pos, "%+9.3f, ", data[i]);
  }

  // print collected data
  fprintf(file, "%s\n", buf);
}

/*****************************************************************************************
 diffDyn: Dynamic system for a derivative + filter
 *****************************************************************************************/
vec diffDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  // define output vector
  vec y(x.size());

  // apply numerical differential dynamics for all input
    xdot[0] =        - 50.0 * x[0] +  1.0 * u[0];
    y[0]    = - 50.0 * 50.0 * x[0] + 50.0 * u[0];

  // return differentiated signal
  return y;
}
#endif // TESTBED



