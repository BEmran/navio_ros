#include "ode.h"

using namespace std;
typedef vector<float> vec;

/******************************************************************************
 * @Dyn: create dynamic system object
 * parameters:
   - size: size of system state, if size is unkown it will be 1
 * returns: object signiture
******************************************************************************/
//ODE::ODE(int size):
//  size_(size), dynFun_(ode::g_Fun){
//  initVector();         // initialize system vector by zeros
//}

/******************************************************************************
 * @Dyn: create dynamic system object
 * parameters:
   - size: size of system state.
   - fun: function pointer define the deferential equations.
 * returns: object signiture
******************************************************************************/
ODE::ODE(int size, vec (*fun) (vec&, vec&, vec&, vec&)):
  size_(size), dynFun_(*fun){
  initVector();         // initialize system vector by zeros
}

/******************************************************************************
 * @Dyn: create dynamic system object
 * parameters:
   - size: size of system state.
   - fun: function pointer define the deferential equations.
   - x0: initial values for the states.
 * returns: object signiture
******************************************************************************/
ODE::ODE(int size, vec (*fun) (vec&, vec&, vec&, vec&), vec &x0):
  size_(size), dynFun_(fun){
  initVector();         // initialize system vectors by zeros
  setX(x0);             // initialize the states by x0
}

/******************************************************************************
 * @setX: set the initial value for the system states
 * parameter:
   - x0: initial values for the states.
******************************************************************************/
void ODE::setX(vec& x0){
  // copy values of vector x0 to vector x
  x_ = x0;
}

/******************************************************************************
 * @initVector: initialize the object vetors to zero
******************************************************************************/
void ODE::initVector(){
  // initialize vectors by zeros
  for (int i=0; i < size_; i++){
    x_.push_back(0.0);
    xdot_.push_back(0.0);
    xdot0_.push_back(0.0);
  }
}

/******************************************************************************
 * @getX: get the system states vector
 * returns: a copy of the state vector
******************************************************************************/
vec ODE::getX(void) const{
  // define vector same size as the states and copy the state value
  vec tmpX = x_;
  return tmpX;                // return the vetor
}

/******************************************************************************
 * @getXdot: get the system xdot vector
 * returns: a copy of the xdot vector
******************************************************************************/
vec ODE::getXdot(void) const{
  // define vector same size as the states and copy the xdot value
  vec tmpX = xdot_;
  return tmpX;                // return the vetor
}

/******************************************************************************
 * @update: main method that apply deferential equations and integration
 * parameter:
   - u: input vector
   - par: dynamic function parameters
   - dt: time increment. If not defined equal to 0.01
 * returns: a copy of the output vector
******************************************************************************/
vec ODE::update(vec& u, vec& par, float dt){
  vec y = dynFun_(x_, xdot_, u, par);    // apply dynamic equations
  tapzInteg(dt);                    // apply integration
  return y;                         // return output vector
}

/******************************************************************************
 * @update: main method that apply deferential equations and integration
 * parameter:
   - u: input vector
   - xdot: returns xdot vector
   - par: dynamic function parameters
   - dt: time increment. If not defined equal to 0.01
 * returns: a copy of the output vector
******************************************************************************/
vec ODE::update(vec& u, vec& xdot, vec& par, float dt){
  vec y = dynFun_(x_, xdot_, u, par);    // apply dynamic equations
  tapzInteg(dt);              // apply integration
  xdot = xdot_;
  return y;                   // return output vector
}

/******************************************************************************
 * @tapzInteg: apply trapezoidal integration
 * parameter:
   - dt: time increment
 * integration: x_new = x_old + dt * (dxdt + dxdt_old) / 2
******************************************************************************/
void ODE::tapzInteg(float dt){
  // apply integration
  for (int i = 0; i < x_.size(); i++){
    x_[i] = x_[i] + dt * (xdot_[i] + xdot0_[i]) / 2.0;
    xdot0_[i] = xdot_[i];
  }
}
