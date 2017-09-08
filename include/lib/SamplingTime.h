
#ifndef SAMPLINGTIME_H
#define SAMPLINGTIME_H

#include <cstdio>       // printf
#include <iostream>     // localtime
#include <sys/time.h>   // gettimeofday
#include <unistd.h>     // usleep

class SamplingTime {
public:
    SamplingTime(float Freq);
    virtual ~SamplingTime();
    float tsCalculat(void);
    void setFreq(float Freq);
    float getTS(void) const; 

private:
    time_t t;
    float ts, freq;
    long ptime, ctime;
    struct timeval tval;
    long calTime(void);
};

#endif /* SAMPLINGTIME_H */