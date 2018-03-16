#include "TimeSampling.h"

/******************************************************************************
TimeSampling: Create object
- Inputs:
        1- freq: requested frequency for the tme sampling in Hz
******************************************************************************/
TimeSampling::TimeSampling(const float freq){
    setFreq(freq);
    _ptime = calTime();
}
/******************************************************************************
~TimeSampling: destroy object
******************************************************************************/

TimeSampling::~TimeSampling() {
}

/******************************************************************************
updateTs: calculate time stamp
1- Find time difference (dt) between current time (ctime) and previsos time (_ptime)
2- sleep dt if it is less than frequency (_freq)
- returns time difference dt
******************************************************************************/
float TimeSampling::updateTs(void) {
    long ctime = calTime();                     // Calculate current time
    float dt = (ctime - _ptime) / 1000000.0;    // Calculate dt

    // sleep until next sampling time
    if (dt < (1/_freq)) {
        long delay = (1/_freq - dt) * 1000000L; // Find required delay
        usleep(delay);
        ctime = calTime();                      // calculate update current time

        dt = (ctime - _ptime) / 1000000.0;      // calculate updated dt
    }
    _ptime = ctime;                             // store for the next time
    return dt;
}

/******************************************************************************
calTime: calculate current time
- returns lond integer as in (sec * 10^6)
******************************************************************************/
long TimeSampling::calTime(){
    gettimeofday(&_tval, NULL);
    // return current time in micro sec
    return 1000000L * _tval.tv_sec + _tval.tv_usec;
}

/******************************************************************************
setFreq: set frequency
******************************************************************************/

void TimeSampling::setFreq(const float freq){
    _freq = freq;
}
