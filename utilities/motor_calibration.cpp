#include "../include/testbed_navio/navio_interface.h"
#include "../include/lib/TimeSampling.h"               // time sampling library
#include <iostream>
#include <signal.h>                         // signal ctrl+c

using namespace std;
pthread_t _Thread_Control;
bool _CloseRequested = false;

struct dataStruct {
  float cmd;
};

void ctrlCHandler(int signal) {
  _CloseRequested = true;
  printf("Ctrl+c have been detected\n");
}

void* controlThread(void *data)
{
  // Initialize mapping data
  struct dataStruct *data_;
  data_ = (struct dataStruct *) data;

  PWM *pwm;
  initializePWM(pwm, 0);
  data_->cmd = 0;
  while (!_CloseRequested)
  {
    float du[4] = {data_->cmd, data_->cmd, data_->cmd, data_->cmd};
    usleep(10000);
    setPWMDuty(pwm, du);
  }

  // Exit procedure -----------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Exit control thread\n");
  pthread_exit(NULL);
}


int main(int argc, char** argv)
{
  cout << "hi" << endl;
  dataStruct data;
  signal(SIGINT, ctrlCHandler);
  pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);

  int x = 0;

  cout << "Enter 1 to send high" << endl;
  cin >> x;
  data.cmd = 1;

  cout << "Enter 1 to send low" << endl;
  cin >> x;
  data.cmd = 0;

  while (x != 0 && !_CloseRequested)
  {
    cout << "enter value" << endl;
    cin >> x;
    data.cmd = x/100.0;
  }

  printf("Close program\n");
  ctrlCHandler(0);
  pthread_cancel(_Thread_Control);
  return(0);
}

