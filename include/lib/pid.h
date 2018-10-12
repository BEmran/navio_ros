#ifndef PID_H
#define PID_H
/**************************************************************************************************
 *
**************************************************************************************************/
class PID{
private:
  float _ei, _e0;
  float _Kp, _Ki, _Kd, _Kt;
public:
  ~PID(){}
  PID(){
    _ei = 0.0;
    _e0 = 0.0;
  }
  void setGains(float Kp = 1, float Ki = 0, float Kd = 0, float Kt = 0){
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _Kt = Kt;
  }

  float update(float x, float xdes, float m, float M, float dt){
    float e = xdes - x;
    float ed = (e - _e0) / dt;
    float u = _Kp * e + _ei + _Kd * ed;
    float usat;
    if (u >= M)
      usat = M;
    else if (u <= m)
      usat = m;
    else
      usat = u;

    _ei += (e * _Ki + _Kt * (-u+usat)) * dt;
    _e0 = e;
    return u;
  }
  };
#endif // PID_H
