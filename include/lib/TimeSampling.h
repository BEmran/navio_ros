#ifndef TIMESAMPLING_H
#define TIMESAMPLING_H

#include <iostream>     // localtime
#include <sys/time.h>   // gettimeofday
#include <unistd.h>     // usleep

class TimeSampling {
public:
    TimeSampling(const float freq);
    ~TimeSampling();
    float updateTs(void);
    void setFreq(const float freq);

private:
    time_t _t;
    float _dt, _freq;
    long _ptime;
    struct timeval _tval;
    long calTime(void);
};

#endif /* TIMESAMPLING_H */
