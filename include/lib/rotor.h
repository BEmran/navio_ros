#ifndef ROTOR_H
#define ROTOR_H
#include "pid.h"
#include "ode.h"
#define _ROTOR_Kp 2.50
#define _ROTOR_Ki 20.5
#define _ROTOR_Kd 1.10
#define _ROTOR_Kt 0.00

static  vec dyn(vec& x, vec& xdot, vec& u, vec& par){
    vec y = x;
    xdot[0] =                    x[1];
    xdot[1] = -300.0*x[0] - 35.0*x[1] + 300.0*u[0];
    return y;
  }
/**************************************************************************************************
 *
**************************************************************************************************/
class Rotor{

public:
  vec x;

  Rotor (void){
    ode = ODE(2, dyn);
    x = ode.getX();
    pid.setGains(_ROTOR_Kp, _ROTOR_Ki, _ROTOR_Kd, _ROTOR_Kt);
  }

  ~Rotor(){}

  float update(float Wdes, float dt){
    // PI conrol with anti windup procedure
    Wdes = Wdes * (1.0/10000.0) * (60.0/2.0/3.14); // Scalling from RPM to 1e-4*RPM to rad/sec

    // Applay pid control
    float u = pid.update(x[0], Wdes, 0.0, 2.2, dt);

    // PWM signal conditioning
    vec input = {signalCondition(u)};

    // System dynamics
    x = ode.update(input, dt);
    return input[0];
}
private:
  ODE ode;
  PID pid;

  float signalCondition(float u){
    float y;
    if (u > 0)
      y = u * 0.4177 + 0.04252;
    else
      y = 0;
    return y;
  }
};

#endif // ROTOR_H
