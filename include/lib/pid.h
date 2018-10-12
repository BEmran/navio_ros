#ifndef PID_H
#define PID_H
/**************************************************************************************************
 *
**************************************************************************************************/
class PID{
public:
  PID(){
    _ei = 0.0;
    _e0 = 0.0;
  }

  ~PID(){}

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
    float usat = sat(u, m, M);
    float anti_windup = _Kt * (-u + usat);
    _ei += (e * _Ki + anti_windup) * dt;
    _e0 = e;
    return u;
  }

private:
  float _ei, _e0;
  float _Kp, _Ki, _Kd, _Kt;
  float sat(float u, float min, float max){
    float y;
    if (u >= max)
      y = max;
    else if (u <= min)
      y = min;
    else
      y = u;
    return y;
  }
};

#endif // PID_H
