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
vec diffRdDyn(vec& x, vec& xdot, vec& u, vec& par);
vec diffWdDyn(vec& x, vec& xdot, vec& u, vec& par);
vec RdDyn(vec& x, vec& xdot, vec& u, vec& par);
vec WdDyn(vec& x, vec& xdot, vec& u, vec& par);
vec3 rotation_control(dataStruct* data);
vec3 anguler_control(dataStruct* data, vec3 Wc);
vec3 motor(vec3 M);
vec3 RBF(vec3 e);
vec3 sat(const vec3& x, float max, float min);
mat3 angle2dcm(  const float roll, const float pitch, const float yaw );
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
  vec3 Wc = rotation_control(data);
  vec3 M = anguler_control(data, Wc);
  M = motor(M);
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
 /*
  *  static mat3 Rd = mat3::Identity();
  static vec3 Wd = vec3::Zero(3,1);
  static MatrixXf VV = MatrixXf::Random(3,9);
  vec Rdvec(Rd.data(), Rd.data() + Rd.rows() * Rd.cols());
  vec VVvec(VV.data(), VV.data() + VV.rows() * VV.cols());
  static ODE ode_Wd(3, *WdDyn);
  static ODE ode_Rd(9, *RdDyn, Rdvec);
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
  Rd = mat3 (ode_Rd.update(Rcvec, Rd_dotvec, Rd_par, 0.01).data());
  mat3 Rd_dot(Rd_dotvec.data());
  // system and control parameters --------------------------------------------
  float Jxy = 0.01, Jz = 0.1;
  mat3 J; J <<  Jxy,   0,  0, 0, Jxy,  0, 0,   0, Jz;
  mat3 Kr, Kw;
  Kr << data->angConGain.kr[0], 0, 0, 0, data->angConGain.kr[1], 0, 0, 0, data->angConGain.kr[2];
  Kw << data->angConGain.kw[0], 0, 0, 0, data->angConGain.kw[1], 0, 0, 0, data->angConGain.kw[2];
*/
  /////////////////////////////////////////////////////////////////////////////
  /// T. Lee Method
/*
  static vec3 thR; thR <<0,0,0;
  // traking error ------------------------------------------------------------
  vec3 er = 0.5 * skewInv(Rd.transpose() * R - R.transpose() * Rd);
  vec3 A = R.transpose() * Rd * Wd;
  vec3 ew = W - A;
  Wd = skewInv(Rd.transpose() * Rd_dot);
  vec wdvec(Wd.data(), Wd.data() + Wd.rows() * Wd.cols());
  vec Wd_dotvec = {0,0,0};
  vec Wd_par = {50, 50, 50};
  Wd = vec3 (ode_Wd.update(wdvec, Wd_dotvec, Wd_par, 0.01).data());
  vec3 Wd_dot(Wd_dotvec.data());
  //mat3 WR ={W[2]*W[3], 0,0,0, W[1]*W[3],0,0,0, W[1]*W[1]}
  vec3 M = - Kr * er - Kw * ew + skew(A) * J * A + J * R.transpose() * Rd * Wd_dot;// - WR * thR;
  //  vec3 thR_dot = 0.1 * WR.transpose()*(er - 0.1 * ew);
  //  thR = thR + thR_dot*0.01;
  */
  /////////////////////////////////////////////////////////////////////////////
  /*
/// DSC method
  //  mat3 th(mat3::Identity());
  //  vec3 er = skewInv(mat3::Identity() - Rd.transpose()*R);
  //  vec3 qr = - Kr * er;
  //  vec3 Wc = skewInv((R.transpose()*Rd).inverse()*(-Rd_dot.transpose()*R + skew(qr)));
  //  vec wc = {Wc(0),Wc(1),Wc(2)};
  //  vec Wd_dotvec = {0,0,0};
  //  Wd = vec3 (ode_Wd.update(wcvec, Wd_dotvec, Wd_par, 0.01).data());
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
  vec3 C = 0.5 * skewInv(Rd_dot.transpose()*R - R.transpose()*Rd_dot);
  vec3 Wc = A.inverse() * (qr - C);
  // generate Wd_dot ----------------------------------------------------------
  vec wcvec = {Wc[0],Wc[1],Wc[2]};
  vec Wd_par = {50, 50, 50};
  vec Wd_dotvec = {0, 0, 0};
  Wd = vec3 (ode_Wd.update(wcvec, Wd_dotvec, Wd_par, 0.01).data());
  vec3 Wd_dot(Wd_dotvec.data());
  vec3 ew = W - Wd;

/*
  // RBF-NN -------------------------------------------------------------------
  //VectorXf cen(9); cen << -2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0;
  //float sigma = 20;
  //float eta = 10;
  //vec3 dist;
  //MatrixXf VV_dot = MatrixXf::Zero(3,9);
  //VectorXf tmp(9);
  //for (int i=0; i < 3; i++){
   // for (int j=0; i < 9; i++){
     // float z = ew[i] - cen[j];
     // tmp[j] = exp(-z/2.0/sigma/sigma);
   // }
    //VectorXf p = tmp / sqrt(2 * PI) / sigma;
  //  dist[i] = VV.row(i) * p;
   // VV_dot.row(i)  = eta * (ew[i] * p) - 0.01 * norm(ew[i]) * VV.row(i);
  //}
*/
  //
  //vec3 ew = W;
  //vec3 Wd_dot = vec3::Zero(3);
 // vec3 f = - W.cross(J*W);
 // vec3 qw = - f - Kw * ew + Wd_dot;
 // vec3 M = qw;//th.inverse() * qw;
/*
  // integration
  //vec vv_par;
  //vec VV_dotvec(VV_dot.data(), VV_dot.data() + VV_dot.rows() * VV_dot.cols());
  //vec tmp_vv = ode_RBF.update(VV_dotvec, vv_par, 0.01);
*/  //VV = Eigen::Map<Matrix<float,3,9>>(tmp_vv.data());
  /////////////////////////////////////////////////////////////////////////////
  // print info ---------------------------------------------------------------
/*  static int ii = 0;
  ii++;
  if (ii == 100) {
    ii = 0;
//    cout << " Rd\n"       << Rd     << endl;
//    cout << " Rd_dot\n"   << Rd_dot << endl;
//    cout << " Wd_dot\n"   << Wd_dot << endl;
//    cout << " R\n"        << R      << endl;
    cout << " eW\n"       << ew      << endl;
    cout << " er\n"       << er     << endl;
    cout << " W\n"        << W      << endl;
  }
  */

  // send output data ---------------------------------------------------------
  //vec3 M = vec3::Zero(3);
  data->du[0] = 1.5;
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
// diffRDyn: Dynamic system: filter
// *****************************************************************************************/
vec diffRdDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  vec y(x.size());

  for (int r=0; r < 3; r++){
    for (int c=0; c < 3; c++){
      int i = r*3 + c;
      xdot[i] = -par[i] * x[i] - par[i] * par[i] * u[i];
      y[i]    =           x[i] +          par[i] * u[i];
    }
  }
  return y;
}

///*****************************************************************************************
// diffWDyn: Dynamic system: filter
// *****************************************************************************************/
vec diffWdDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  vec y(x.size());

  for (int i=0; i < x.size(); i++){
    xdot[i] = - par[i] * x[i] - par[i] * par[i] * u[i];
    y[i]    =            x[i] +          par[i] * u[i];
  }
  return y;
}
///*****************************************************************************************
// RdDyn: Dynamic system: filter
// *****************************************************************************************/
vec RdDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  vec y(x.size());

  for (int k=0; k < 9; k++){
    xdot[k] = par[k] * (u[k] - x[k]);
    y[k]    =   x[k];
  }
  return y;
}

///*****************************************************************************************
// WdDyn: Dynamic system: filter
// *****************************************************************************************/
vec WdDyn(vec& x, vec& xdot, vec& u, vec& par)
{
  vec y(x.size());

  for (int k=0; k < 3; k++){
    xdot[k] = par[k] * (u[k] - x[k]);
    y[k]    =   x[k];
  }
  return y;
}
/*****************************************************************************************
 @rotation_control: apply rotation control system
 *****************************************************************************************/
vec3 rotation_control(dataStruct* data)
{
  static bool init = true;
  static mat3 Rd = mat3::Identity();
  static ODE ode_Rd(9);
  if (init)
  {
    init = false;
    vec tmp = vec(Rd.data(), Rd.data() + Rd.rows() * Rd.cols());
    ode_Rd.setX(tmp);
  }
  // parameters
  mat3 KR;
  KR << data->angConGain.kr[0], 0, 0, 0, data->angConGain.kr[1], 0, 0, 0, data->angConGain.kr[2];
  mat3 KF = 50 * mat3::Identity();
  float maxW = 1;
  // get info
  mat3 R, Rc;
  R =  AngleAxisf(data->enc_angle[2], vec3::UnitZ())
      * AngleAxisf(data->enc_angle[1], vec3::UnitY())
      * AngleAxisf(data->enc_angle[0], vec3::UnitX());
  Rc =  AngleAxisf(data->rosnode->_cmd_ang[2], vec3::UnitZ())
      * AngleAxisf(data->rosnode->_cmd_ang[1], vec3::UnitY())
      * AngleAxisf(data->rosnode->_cmd_ang[0], vec3::UnitX());
  // Dynamics
  mat3 Rd_dot = KF * (Rc - Rd);
  // tracking error
  mat3 Rt = Rd.transpose() * R;
  vec3 eR = 0.5 * skewInv((Rt - Rt.transpose()));
  vec3 C = 0.5 * skewInv(Rd_dot.transpose()*R - R.transpose()*Rd_dot);
  mat3 A33 = 0.5 * (mat3::Identity() * Rt.trace() - Rt.transpose());
  // virtual control
  vec3 Vr = - KR * eR;
  vec3 Wc = A33.inverse() * (-C + Vr);
  // saturation
  vec3 Wc_sat = sat(Wc, maxW, -maxW);
  MatrixXf A39(3,9);
  A39 <<      0,       0,       0, -R(0,2), -R(1,2), -R(2,2),  R(0,1),  R(1,1),  R(2,1),
         R(0,2),  R(1,2),  R(2,2),       0,       0,       0, -R(0,0), -R(1,0), -R(2,0),
        -R(0,1), -R(1,1), -R(2,1),  R(0,0),  R(1,0),  R(2,0),       0,       0,       0;
  A39 = 0.5 * A39;
  VectorXf tmp = A39.colPivHouseholderQr().solve(Vr - (A33 * Wc_sat));
  Rd_dot -= Map<MatrixXf> (tmp.data(), 3,3);
  // integration
  vec empty;
  vec Rd_dot_vec = vec (Rd_dot.data(), Rd_dot.data() + Rd_dot.rows() * Rd_dot.cols());
  Rd = mat3 (ode_Rd.update(Rd_dot_vec, empty, 0.01).data());
/*
  cout << "R\n"         << R            << endl;
  cout << "Rc\n"        << Rc           << endl;
  cout << "KR\n"        << KR           << endl;
  cout << "Rd\n"        << Rd           << endl;
  cout << "Rd_dot\n"    << Rd_dot_temp  << endl;
  cout << "eR\n"        << eR           << endl;
  cout << "C\n"         << C            << endl;
  cout << "A33\n"       << A33          << endl;
  cout << "Vr\n"        << Vr           << endl;
  cout << "Wc\n"        << Wc           << endl;
*/
  cout << "Wc_sat\n"    << Wc_sat       << endl;
/*
  cout << "A39\n"       << A39          << endl;
  cout << "tmp\n"       << tmp          << endl;
  cout << "Rd_dot\n"    << Rd_dot       << endl;
  cout << "Rd_dot\n"    << Rd_dot_vec[0] << "  "
       << Rd_dot_vec[1] << "  "
       << Rd_dot_vec[2] << endl;
*/
  // control signal
  return Wc_sat;
}

/*****************************************************************************************
 @angle2dcm: convert euler angles to direction cousine matrix
 *****************************************************************************************/
mat3 angle2dcm( const float roll, const float pitch, const float yaw )
{
  mat3 R;
  R = AngleAxisf (yaw,   Vector3f::UnitZ())*
      AngleAxisf (pitch, Vector3f::UnitY())*
      AngleAxisf (roll,  Vector3f::UnitX());
  return R;
}
/*****************************************************************************************
 @sat: apply saturation
 *****************************************************************************************/
vec3 sat(const vec3& x, float max, float min)
{
  vec3 y;
  for (int i=0; i<3; i++){
    if (x[i] > max)
      y[i] = max;
    else if (x[i] < min)
      y[i] = min;
    else
      y[i] = x[i];
  }
  return y;
}
/*****************************************************************************************
 @anguler_control: apply anguler control system
 *****************************************************************************************/
vec3 anguler_control(dataStruct* data, vec3 Wc)
{
  static bool init = true;
  static vec3 Wd = vec3::Zero();
  static ODE ode_Wd(3);
  if (init)
  {
    init = false;
    vec tmp = vec(Wd.data(), Wd.data() + Wd.rows() * Wd.cols());
    ode_Wd.setX(tmp);
  }
  // system and control parameters --------------------------------------------
  float Jxy = 0.01, Jz = 0.02;
  mat3 J; J <<  Jxy,   0,  0, 0, Jxy,  0, 0,   0, Jz;
  mat3 B; B << 0.01,   0,  0, 0, 0.01,  0, 0,   0, 0.001;
  B = J.inverse() * B;  mat3 Kw;
  Kw << data->angConGain.kw[0], 0, 0, 0, data->angConGain.kw[1], 0, 0, 0, data->angConGain.kw[2];
  float maxM = 0.2;
  mat3 KF = 50 * mat3::Identity();
  // get input data -----------------------------------------------------------
  vec3 W{data->w[0], data->w[1], data->w[2]};
  // Dynamics
  vec3 Wd_dot = KF * (Wc - Wd);
  // tracking error
  vec3 eW = W - Wd;
  vec3 f = - J.inverse() * W.cross(J*W);
  // virtual control
  vec3 Vw = - Kw * eW;
  vec3 dist_est = vec3::Zero();//RBF(eW);
  vec3 M =  B.inverse() * (-f + Vw + Wd_dot - dist_est);
  // saturation
  vec3 M_sat = sat(M, maxM, -maxM);
  VectorXf tmp = Vw - (B * M_sat - f - dist_est);
  Wd_dot -= tmp;
  // integration
  vec empty;
  vec Wd_dot_vec = vec (Wd_dot.data(), Wd_dot.data() + Wd_dot.rows() * Wd_dot.cols());
  Wd = vec3 (ode_Wd.update(Wd_dot_vec, empty, 0.01).data());

/*
  cout << "W\n"         << W            << endl;
  cout << "Wc\n"        << Wc           << endl;
  cout << "KW\n"        << KW           << endl;
  cout << "Wd\n"        << Wd           << endl;
  cout << "eW\n"        << eW           << endl;
  cout << "Vw\n"        << Vw           << endl;
  cout << "M\n"         << M           << endl;
  cout << "f\n"         << f            << endl;
  cout << "B\n"         << B            << endl;
  cout << "M_sat\n"     << M_sat        << endl;
  cout << "tmp\n"       << tmp          << endl;
  cout << "Wd_dot\n"    << Wd_dot       << endl;
*/
  // control signal
  return M_sat;
}

vec3 RBF(vec3 e){
  const float _PI = 3.14159;
  MatrixXf V = MatrixXf::Random(3,9);
  static bool init = true;
  static ODE ode_V(27);
  if (init)
  {
    init = false;
    vec tmp = vec(V.data(), V.data() + V.rows() * V.cols());
    ode_V.setX(tmp);
  }
  // Parameters
  VectorXf cen(9); cen << -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0;
  float sigma = 2;
  float eta = 2;
  vec3 dist_est(3);
  MatrixXf V_dot(3,9);
  VectorXf p(9);
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<9; j++)
    {
      float z = (e(i) - cen(j));
      float tmp = exp( -z / 2 / sigma/sigma);
      p(j) = tmp / sqrt(2.0 * _PI) / sigma;
    }
    dist_est(i) = V.row(i) * p;
    V_dot.row(i) = eta * (e(i) * p.transpose()) - 0.01 * abs(e(i)) * V.row(i);
  }
  // integration
  vec empty;
  vec V_dot_vec = vec (V_dot.data(), V_dot.data() + V_dot.rows() * V_dot.cols());
  V = mat3 (ode_V.update(V_dot_vec, empty, 0.01).data());
  return dist_est;
}

  vec3 motor(vec3 Oc){
  vec3 Od = vec3::Zero(3);
  vec3 O = vec3::Zero(3);
  static bool init = true;
  static ODE ode_Motor(3);
  static ODE ode_MotorControl(3);
  if (init)
  {
    init = false;
    vec tmpOd = vec(Od.data(), Od.data() + Od.rows() * Od.cols());
    ode_MotorControl.setX(tmpOd);
    vec tmpO = vec(O.data(), O.data() + O.rows() * O.cols());
    ode_Motor.setX(tmpO);
  }
  // Parameters
  float tau = 18.8;
  float maxM = 1;
  float Kw = 10;
  mat3 KF = 50 * mat3::Identity();
  // Dynamics
  vec3 Od_dot = KF * (Oc - Od);
  // tracking error
  vec3 eO = O - Od;
  // virtual control
  vec3 Vw = - Kw * eO;
  vec3 M =  (Vw + Od_dot) / tau + O;
  // saturation
  vec3 M_sat = sat(M, maxM, -maxM);
  VectorXf tmp = Vw - (tau * M_sat);
  Od_dot -= tmp;
  // integration
  vec empty;
  vec Od_dot_vec = vec (Od_dot.data(), Od_dot.data() + Od_dot.rows() * Od_dot.cols());
  M = vec3 (ode_MotorControl.update(Od_dot_vec, empty, 0.01).data());

  vec3 O_dot = tau * (M - O);
  vec O_dot_vec = vec (O_dot.data(), O_dot.data() + O_dot.rows() * O_dot.cols());
  ode_Motor.update(O_dot_vec, empty, 0.01);

  return M;
}
