/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on March 14, 2018
 */
#include "testbed_navio/testbed_demo.h"
#include <eigen3/Eigen/Core>
///#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
typedef Matrix3f mat3;
typedef Vector3f vec3;
mat3 skew(const vec3& x);
vec3 skewInv(const mat3& x);
void WdDyn(float* y, float* x, float* xdot, float* u, float t);

/******************************************************************************
main: Run main function
******************************************************************************/
int main(int argc, char** argv) {

  // Welcome msg ------------------------------------------------------------
  printf("Start Program...\n");
  // conncet ctrl+c Handler
  signal(SIGINT, ctrlCHandler);

  // Define main variables --------------------------------------------------
  struct dataStruct data;
  data.argc = argc;
  data.argv = argv;
  data.is_mainfun_ready = false;
  data.is_control_ready = false;
  data.is_rosnode_ready = false;
  data.is_sensors_ready = false;

  data.wSys = ODE(3, *diffDyn);
  data.w.push_back(0); data.w.push_back(0); data.w.push_back(0);

  TimeSampling st(_MAINFUN_FREQ);
  for (int i = 0; i < 25; ++i) {
    data.record[i] = 0.0;       // initialize record data with zeros
  }

  // Start threads ----------------------------------------------------------
  pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);
  pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
  pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);

  // Create new record file -------------------------------------------------
  char file_name[64];
  int fileNumber = 0;
  // find avaliable file name & number
  do {
    sprintf(file_name, "/home/pi/testbed_data_%.2d.csv", fileNumber++);
  } while (access(file_name, F_OK) == 0);
  // open avaliable file
  data.file = fopen(file_name, "w");
  // check file
  if (data.file == NULL) {
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
  fprintf(data.file,"Current local time and date: %s", asctime(timeinfo));

  // Print data header
  fprintf(data.file, "time,"
                     "ax,ay,az,"
                     "gx,gy,gz,"
                     "mx,my,mz,"
                     "enc0,enc1,enc2,"
                     "roll,pitch,yaw,"
                     "ur,up,uw,uz,"
                     "d0,d1,d2,d3,d4\n");

  // Wait for user to be ready ----------------------------------------------
  while(!data.is_rosnode_ready || !data.is_control_ready || !data.is_sensors_ready);
  int x = 0;
  while (x == 0) {
    printf("Enter 1 to start control\n");
    cin >> x;
    sleep(1);
  }
  data.is_mainfun_ready = true;

  // Main loop --------------------------------------------------------------
  float dt, dtsumm = 0;
  while(!_CloseRequested){
    dt = st.updateTs();
    dtsumm += dt;
    //printf("dt = %f\n",dt);
    if (dtsumm > 1) {
      dtsumm = 0;
      printf("Mainfun thread: running fine with %4d Hz\n", int(1 / dt));
    }
  }

  // Exit procedure ---------------------------------------------------------
  ctrlCHandler(0);
  printf("Close program\n");
  return 0;
}

/*****************************************************************************************
control: Perfourm control loop
******************************************************************************************/
void control(dataStruct* data, float dt){

  //    static float ei[3] = {0.0, 0.0, 0.0};

  //    float e[3];
  //    float cmd_max[3] = {0.2, 0.2, 0.5};
  //    float cmd_adj[3];
  //    float ang[3] = {data->enc_angle[0], data->enc_angle[1],data->enc_angle[2]};
  //    float w[3] = {0.0, 0.0, 0.0};

  //    data->du[0] = 2.0;
  //    // LQR control
  //    for (int i = 0; i < 3; i++)
  //    {
  //        // adjust cmd
  //        cmd_adj[i] = sat(data->rosnode->_cmd_ang[i], ang[i] + cmd_max[i], ang[i] - cmd_max[i]);

  //        // traking error
  //        e[i] = cmd_adj[i] - ang[i];

  //        // control signal
  //        data->du[i+1] = e[i] * data->angConGain.kp[i] - w[i] * data->angConGain.kd[i] + ei[i] * data->angConGain.ki[i];

  //        // integration
  //        ei[i] += e[i] * dt;
  //    }
  //    printf("%2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t \n",
  //           ang[0], w[0], data->rosnode->_cmd_ang[0], cmd_adj[0],e[0],data->du[1+0],ei[0]);


  //T. Lee Method
  static vec3 thR; thR <<0,0,0;

  static ODE ode_Wd_dot(3, *diffDyn);
  static ODE ode_Rd_dot(9, *diffDyn);

  mat3 Rd = mat3::Identity(); 

  vec Rd_par = {50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0};
  vec Rdv(Rd.data(), Rd.data() + Rd.rows() * Rd.cols());
  mat3 Rd_dot(ode_Rd_dot.update(Rdv, Rd_par, 0.01).data());

  vec3 Wd = skewInv(Rd.transpose() * Rd_dot);
  vec Wdv(Wd.data(), Wd.data() + Wd.rows() * Wd.cols());
  vec Wd_par = {50,50,50};
  vec3 Wd_dot(ode_Wd_dot.update(Wdv, Wd_par, 0.01).data());

  float Jxy = 0.01, Jz = 0.1;
  mat3 J; J <<  Jxy,   0,  0, 0, Jxy,  0, 0,   0, Jz;
  mat3 Kr, Kw;
  Kr << data->angConGain.kr[0], 0, 0, 0, data->angConGain.kr[1], 0, 0, 0, data->angConGain.kr[2];
  Kw << data->angConGain.kw[0], 0, 0, 0, data->angConGain.kw[1], 0, 0, 0, data->angConGain.kw[2];

  mat3 R;
  R =   AngleAxisf(data->enc_angle[2], vec3::UnitZ())
      * AngleAxisf(data->enc_angle[1], vec3::UnitY())
      * AngleAxisf(data->enc_angle[0], vec3::UnitX());
  vec3 W{data->w[0], data->w[1], data->w[2]};
  vec3 er = 0.5 * skewInv(Rd.transpose() * R - R.transpose() * Rd);
  vec3 A = R.transpose() * Rd * Wd;
  vec3 ew = W - A;
  mat3 WR;
  WR << W[2]*W[3], 0,0,0, W[1]*W[3],0,0,0, W[1]*W[1];
  vec3 M = - Kr * er - Kw * ew + skew(A) * J * A + J * R.transpose() * Rd * Wd_dot;// - WR * thR;

  static int ii = 0;
  ii++;
  if (ii = 200) {
    ii = 0;
    cout << " Rd\n"       << Rd     << endl;
    cout << " Rd_dot\n"   << Rd_dot << endl;
    cout << " Wd_dot\n"   << Wd_dot << endl;
    cout << " R\n"        << R      << endl;
    cout << " W\n"        << W      << endl;
    cout << " er\n"       << er     << endl;
    cout << " WR\n"       << WR     << endl;
  }
//  vec3 thR_dot = 0.1 * WR.transpose()*(er - 0.1 * ew);
//  thR = thR + thR_dot*0.01;

//  data->du[0] = 2.0;
//  data->du[1] = M[0];
//  data->du[2] = M[1];
//  data->du[3] = M[2];

  // My method
  /*static const float* wd;
  static DynSys wdSys = DynSys(3, *WdDyn);
  //static DynSys rdSys = DynSys(3, *RdDyn);
  wd = wdSys.getY();
  //rd = rdSys.getY();

  Quaternionf qd;
  qd.x() = data->rosnode->_cmd_quat.x;
  qd.y() = data->rosnode->_cmd_quat.x;
  qd.z() = data->rosnode->_cmd_quat.x;
  qd.w() = data->rosnode->_cmd_quat.x;
  //Matrix3f Rd = qd.normalized().toRotationMatrix();

  Matrix3f Rd(Matrix3f::Identity());
  Matrix3f Rd_dot; Rd_dot.setZero();
  float Jxy = 0.01, Jz = 0.1;
  Matrix3f J;
  J <<  Jxy,   0,  0,
      0, Jxy,  0,
      0,   0, Jz;
  Matrix3f Kr, Kw;
  Kr << data->angConGain.kr[0], 0, 0, 0, data->angConGain.kr[1], 0, 0, 0, data->angConGain.kr[2];
  Kw << data->angConGain.kw[0], 0, 0, 0, data->angConGain.kw[1], 0, 0, 0, data->angConGain.kw[2];
  ////////////////////
  Matrix3f R;
  R =   AngleAxisf(data->enc_angle[2], Vector3f::UnitZ())
      * AngleAxisf(data->enc_angle[1], Vector3f::UnitY())
      * AngleAxisf(data->enc_angle[0], Vector3f::UnitX());
  Vector3f W;
  W << data->w[0], data->w[1], data->w[2];
  ////////////////////
  Matrix3f tmp_er = Matrix3f::Identity() - Rd.transpose()*R;
  Vector3f er;
  er <<  tmp_er(2,1), tmp_er(0,2), tmp_er(1,0);
  Vector3f temp_qr = - Kr * er;
  Matrix3f qr;
  qr <<         0, -temp_qr(2),  temp_qr(1),
       temp_qr(2),           0, -temp_qr(0),
      -temp_qr(2),  temp_qr(0),           0;
  Matrix3f temp_wc = (R.transpose()*Rd)*(-Rd_dot.transpose()*R + qr);
  Vector3f Wc;
  Wc <<  temp_wc(2,1), temp_wc(0,2), temp_wc(1,0);
  ////////////////////
  float wc[3] = {Wc(0),Wc(1),Wc(3)};
  wdSys.update(wc,0,0.005);
  Vector3f Wd;
  Wd << wd[0], wd[1], wd[2];
  Vector3f Wd_dot;
  float wf = 100;
  Wd_dot << wf * ( -wd[0] + wc[0]),
            wf * ( -wd[1] + wc[1]),
            wf * ( -wd[2] + wc[2]);
  ////////////////////
  Matrix3f th(Matrix3f::Identity());
  Vector3f ew = W - Wd;
  Vector3f qw = Wd_dot + Kw * ew;
  Vector3f M = th.inverse() * qw;

  static int ii = 0;
  ii++;
  if (ii = 200) {
    ii = 0;
    printf("e[0]= %+5.2f\t e[1]= %+5.2f\t e[2]= %+5.2f\n", er[0],er[1],er[2]);
  }
  data->du[0] = 2.0;
  data->du[1] = M[0];
  data->du[2] = M[1];
  data->du[3] = M[2];
  */
}

/*****************************************************************************************
 @skew: return a skew matrix of the input vector
 *****************************************************************************************/
mat3 skew(const vec3& x)
{
  mat3 y;
  y <<    0, -x(2),  x(1),
       x(2),     0, -x(0),
      -x(1),  x(0),     0;
  return y;
}
/*****************************************************************************************
 @skewInv: return a skew inverse vector of the the input matrix
 *****************************************************************************************/
vec3 skewInv(const mat3& x)
{
  vec3 y;
  y << x(2,1), x(0,2), x(1,0);
  return y;
}
/*****************************************************************************************
 @mat2vec: matrix 3x3 3x3 to a vector 9x1
 *****************************************************************************************/
VectorXf mat2vec(mat3& M)
{
  Map<RowVectorXf> V(M.data(), M.size());
  return V;
}
/*****************************************************************************************
 @vec2mat: convert a vector 9x1 to matrix 3x3
 *****************************************************************************************/
mat3 vec2mat(VectorXf& V)
{
  Map<mat3> M(V.data());
  return M;
}
///*****************************************************************************************
// RdDotDyn: Dynamic system: filter
// *****************************************************************************************/
//vec diffDyn(vec& x, vec& xdot, vec& u, float t)
//{
//    vec y(x.size());
//    float wf = 50;

//    for (int i=0; i < x.size(); i++){
//      xdot[i] = -wf[i] * x[i] - wf[i] * wf[i] * u[i];
//         y[i] =          x[i] +         wf[i] * u[i];
//    }
//    return y;
//}

///*****************************************************************************************
// WdDyn: Dynamic system: filter
// *****************************************************************************************/
//vec filterDyn(vec& x, vec& xdot, vec& u)
//{
//  vec y(x.size());
//  float wf = 100;

//  for (int i=0; i < x.size(); i++){
//    xdot[i] = -wf[i] * x[i] - wf[i] * wf[i] * u[i];
//       y[i] =          x[i] +         wf[i] * u[i];
//  }
//  return y;
//}

