///////////////////////////////////////////////////////////////////////////////
///
///
///
///
/// Done by: Bara Emran
/// Date: Aug. 09, 2018
///////////////////////////////////////////////////////////////////////////////
#ifndef ODE_H
#define ODE_H

#include <vector>

using namespace std;

// main data type
typedef vector<float> vec;

namespace ode{
/******************************************************************************
 * @g_Fun: general deferential equation represents integration with no dynamics
 * parameters:
   - x: state vector
   - xdot: state vector deferentiation
   - u: input vector
 * returns: output vector
 * dynamic system: dx/dt = u
                       y = x
******************************************************************************/
  vec g_Fun (vec& x, vec& xdot, vec& u, vec& par)
  {
    xdot = u;           // apply dynamic equations, dx/dt = u
    vec y = x;          // copy state to output
    return y;           // return output vector
  }
}

class ODE
{

#define DT 0.01   // default time increment
#define SIZE 1    // default vector size

public:
  // define object by size
  //ODE(int size = SIZE);
  // define object by size and deferential function
  ODE(int size = SIZE, vec (*fun) (vec&, vec&, vec&, vec&) = ode::g_Fun);
  // define object by size and deferential function and initialize vector
  ODE(int size, vec (*fun) (vec&, vec&, vec&, vec&), vec &x0);
  // initialize state vector
  void setX(vec& x0);
  // get x vector value
  vec getX(void) const;
  // get xdot vector value
  vec getXdot(void) const;
  // main method, applies deferential equations and integrations
  vec update(vec& u, vec& par, float dt = DT);
  // main method applies deferential equations and returns xdot
  vec update(vec& u, vec& xdot, vec& par, float dt = DT);

private:
  // vector size
  int size_;
  // time increment for integration
  float dt_;
  // opject ineternal vectors
  vec x_, xdot_, xdot0_;
  // general deferential equation with no dynamics dxdt = u
  vec (*dynFun_) (vec& x, vec& xdot, vec& u, vec& par);
  // initialize internal object vectors by zeros
  void initVector(void);
  // apply trapezoidal integration
  void tapzInteg(float dt);
};

#endif // ODE_H
