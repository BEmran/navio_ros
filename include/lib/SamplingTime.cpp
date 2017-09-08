#include "SamplingTime.h"

SamplingTime::SamplingTime(float Freq) {
    setFreq(Freq);
    ptime = calTime();
}

SamplingTime::~SamplingTime() {
}

/*******************************************************************************
dtCalculat: calculate time stamp
 *******************************************************************************/
float SamplingTime::tsCalculat(void) {
    ctime = calTime();  // Calculate current time
    ts = (ctime - ptime) / 1000000.0; // Calculate ts

    //--------------------- sleep until next sampling time ---------------------
    if (ts < (1 / freq)) {
        usleep((1 / freq - ts)*1000000);
        ctime = calTime();
        ts = (ctime - ptime) / 1000000.0;
    }
    ptime = ctime; // store for the next time
    return ts;
}

long SamplingTime::calTime(){
    gettimeofday(&tval, NULL);
    return 1000000 * tval.tv_sec + tval.tv_usec;  // return current time in nsec
}

void SamplingTime::setFreq(float Freq){
    freq = Freq;
}

float SamplingTime::getTS() const{
    return 1/ts;
}