#include "../include/testbed_navio/navio_interface.h"
#include <iostream>

using namespace std;
int main(int argc, char** argv)
{
  cout << "hi" << endl;
  // Initialize PWM
  PWM *pwm;
  initializePWM(pwm);
  float high[4] = {1.0, 1.0, 1.0, 1.0};
  float low[4] = {0.0, 0.0, 0.0, 0.0};

  int x = 0;
  cout << "Enter 1 to send high" << endl;
  cin >> x;
  setPWMDuty(pwm, high);

  cout << "Enter 1 to send low" << endl;
  cin >> x;
  setPWMDuty(pwm, low);

  cout << "Exit" << endl;
  return 0;
}
