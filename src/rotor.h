#ifndef ROTOR_H
#define ROTOR_H
#include "pid.h"
#include "ode.h"
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

#endif // ROTOR_H
