#ifndef ROTOR_H
#define ROTOR_H
#include "pid.h"
#include "ode.h"
/**************************************************************************************************
 *
**************************************************************************************************/
class Rotor{
private:
  ODE _ode;
  PID _pid;
public:
  vec x;
  Rotor (ODE ode){
    _ode = ode;
    x = _ode.getX();
    _pid.setGains(2.5, 5.5, 0, 0.2);
  }
  Rotor(){}
  ~Rotor(){}
  float update(float Wdes, float dt){
    // PI conrol with anti windup procedure
    Wdes = Wdes * (1.0/10000.0) * (60.0/2.0/3.14); // Scalling from RPM to 1e-4*RPM to rad/sec

    // Applay pid control
    float u = _pid.update(x[0], Wdes, 0.0, 2.2, dt);

    // PWM signal conditioning
    vec input = {signalConditioning(u)};

    // System dynamics
    vec empty;
    x = _ode.update(input, empty, dt);
    return input[0];
  }
  float signalConditioning(float u){
    float y;
    // PWM signal conditioning
    if (u > 0)
      y = u * 0.4177 + 0.04252;
    else
      y = 0;
    return y;
  }
};

#endif // ROTOR_H
