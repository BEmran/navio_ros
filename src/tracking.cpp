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
vec diffRDyn(vec& x, vec& xdot, vec& u, vec& par);
vec diffWDyn(vec& x, vec& xdot, vec& u, vec& par);

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
  static mat3 Rd = mat3::Identity();
  static vec3 Wd = vec3::Zero(3,1);
  static MatrixXf VV = MatrixXf::Random(3,9);
  vec Rdvec(Rd.data(), Rd.data() + Rd.rows() * Rd.cols());
  vec VVvec(VV.data(), VV.data() + VV.rows() * VV.cols());
  static ODE ode_Wd_dot(3, *diffWDyn);
  static ODE ode_Rd_dot(9, *diffRDyn, Rdvec);
  static ODE ode_RBF(18);
  ode_RBF.setX(VVvec);
  // get input data -----------------------------------------------------------
  mat3 R, Rc;
  R =  AngleAxisf(data->enc_angle[2], vec3::UnitZ())
      * AngleAxisf(data->enc_angle[1], vec3::UnitY())
      * AngleAxisf(data->enc_angle[0], vec3::UnitX());
  Rc =  AngleAxisf(data->rosnode->_cmd_ang[2], vec3::UnitZ())
      * AngleAxisf(data->rosnode->_cmd_ang[1], vec3::UnitY())
      * AngleAxisf(data->rosnode->_cmd_ang[0], vec3::UnitX());
  vec3 W{data->w[0], data->w[1], data->w[2]};
  // generate Rd_dot ----------------------------------------------------------
  vec Rd_par = {50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0};
  vec Rd_dotvec = {0,0,0,0,0,0,0,0,0};
  vec Rcvec(Rc.data(), Rc.data() + Rc.rows() * Rc.cols());
  Rd = mat3 (ode_Rd_dot.update(Rcvec, Rd_dotvec, Rd_par, 0.01).data());
  mat3 Rd_dot(Rd_dotvec.data());
  // system and control parameters --------------------------------------------
  float Jxy = 0.01, Jz = 0.1;
  mat3 J; J <<  Jxy,   0,  0, 0, Jxy,  0, 0,   0, Jz;
  mat3 Kr, Kw;
  Kr << data->angConGain.kr[0], 0, 0, 0, data->angConGain.kr[1], 0, 0, 0, data->angConGain.kr[2];
  Kw << data->angConGain.kw[0], 0, 0, 0, data->angConGain.kw[1], 0, 0, 0, data->angConGain.kw[2];
  /////////////////////////////////////////////////////////////////////////////
  /// T. Lee Method
  /*
  static vec3 thR; thR <<0,0,0;
  // traking error ------------------------------------------------------------
  vec3 er = 0.5 * skewInv(Rd.transpose() * R - R.transpose() * Rd);
  vec3 A = R.transpose() * Rd * Wd;
  vec3 ew = W - A;
  vec3 Wd = skewInv(Rd.transpose() * Rd_dot);
  vec Wdvec(Wd.data(), Wd.data() + Wd.rows() * Wd.cols());
  vec Wd_dotvec = {0,0,0};
  ode_Wd_dot.update(Wdvec, Wd_dotvec, Wd_par, 0.01).data();
  vec3 Wd_dot(Wd_dotvec.data());
  //mat3 WR ={W[2]*W[3], 0,0,0, W[1]*W[3],0,0,0, W[1]*W[1]}
  vec3 M = - Kr * er - Kw * ew + skew(A) * J * A + J * R.transpose() * Rd * Wd_dot;// - WR * thR;
  //  vec3 thR_dot = 0.1 * WR.transpose()*(er - 0.1 * ew);
  //  thR = thR + thR_dot*0.01;
  */
  /////////////////////////////////////////////////////////////////////////////
  /// DSC method
  //  mat3 th(mat3::Identity());
  //  vec3 er = skewInv(mat3::Identity() - Rd.transpose()*R);
  //  vec3 qr = - Kr * er;
  //  vec3 Wc = skewInv((R.transpose()*Rd).inverse()*(-Rd_dot.transpose()*R + skew(qr)));
  //  vec wc = {Wc(0),Wc(1),Wc(2)};
  //  vec Wd_dotvec = {0,0,0};
  //  vec3 Wd(ode_Wd_dot.update(wc, Wd_dotvec, Wd_par, 0.01).data());
  //  vec3 Wd_dot(Wd_dotvec.data());
  //  vec3 ew = W - Wd;
  //  vec3 f = - W.cross(J*W);
  //  vec3 qw = Wd_dot - f - Kw * ew;
  //  vec3 M = th.inverse() * qw;
  /////////////////////////////////////////////////////////////////////////////
  /// DSC method 2
  mat3 th = mat3::Identity();
  mat3 Rtelda = Rd.transpose() * R;
  vec3 er = 0.5 * skewInv(Rtelda - Rtelda.transpose());
  mat3 A = 0.5 * (mat3::Identity() * Rtelda.trace() - Rtelda.transpose());
  vec3 qr = - Kr * er;
  vec3 C = skewInv(Rd_dot.transpose()*R - R.transpose()*Rd_dot);
  vec3 Wc = A.inverse() * (qr - C);
  // generate Wd_dot ----------------------------------------------------------
  vec wcvec = {Wc(0),Wc(1),Wc(2)};
  vec Wd_par = {50,50,50};
  vec Wd_dotvec = {0,0,0};
  Wd = vec3 (ode_Wd_dot.update(wcvec, Wd_dotvec, Wd_par, 0.01).data());
  vec3 Wd_dot(Wd_dotvec.data());
  vec3 ew = W - Wd;
  // RBF-NN -------------------------------------------------------------------
  VectorXf cen(9); cen << -2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0;
  float sigma = 20;
  float eta = 10;
  vec3 dist;
  MatrixXf VV_dot = MatrixXf::Zero(3,9);
  VectorXf tmp(9);
  for (int i=0; i < 3; i++){
    for (int j=0; i < 9; i++){
      float z = ew[i] - cen[j];
      tmp[j] = exp(-z/2.0/sigma/sigma);
    }
    VectorXf p = tmp / sqrt(2 * PI) / sigma;
    dist[i] = VV.row(i) * p;
    VV_dot.row(i)  = eta * (ew[i] * p) - 0.01 * norm(ew[i]) * VV.row(i);
  }
  //
  vec3 f = - W.cross(J*W);
  vec3 qw = Wd_dot - f - dist - Kw * ew;
  vec3 M = th.inverse() * qw;
  // integration
  vec vv_par;
  vec VV_dotvec(VV_dot.data(), VV_dot.data() + VV_dot.rows() * VV_dot.cols());
  vec tmp_vv = ode_RBF.update(VV_dotvec, vv_par, 0.01);
  VV = Eigen::Map<Matrix<float,3,9>>(tmp_vv.data());
  /////////////////////////////////////////////////////////////////////////////
  // print info ---------------------------------------------------------------
  static int ii = 0;
  ii++;
  if (ii == 100) {
    ii = 0;
    cout << " Rd\n"       << Rd     << endl;
    cout << " Rd_dot\n"   << Rd_dot << endl;
    cout << " Wd_dot\n"   << Wd_dot << endl;
    cout << " R\n"        << R      << endl;
    cout << " W\n"        << W      << endl;
    cout << " er\n"       << er     << endl;
    cout << " W\n"        << W      << endl;
  }

  // send output data ---------------------------------------------------------
  data->du[0] = 2.0;
  data->du[1] = M[0];
  data->du[2] = M[1];
  data->du[3] = M[2];
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
vec diffRDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  vec y(x.size());

  for (int r=0; r < 3; r++){
    for (int c=0; c < 3; c++){
      int i = r*3 + c;
      xdot[i] = -par[i] * x[i] - par[i] * par[i] * u[i];
      y[i] =           x[i] +          par[i] * u[i];
    }
  }
  return y;
}

///*****************************************************************************************
// WdDyn: Dynamic system: filter
// *****************************************************************************************/
vec diffWDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  vec y(x.size());

  for (int i=0; i < x.size(); i++){
    xdot[i] = -par[i] * x[i] - par[i] * par[i] * u[i];
    y[i] =           x[i] +          par[i] * u[i];
  }
  return y;
}

