#ifndef SENSORS_H
#define SENSORS_H

#include "Common/MPU9250.h"
#include "Navio2/LSM9DS1.h"
#include "Common/Util.h"
#include <unistd.h>
#include <string>
#include <stdio.h>	// file, printf
#include <sys/time.h>	// time

namespace {
    #define G_SI 9.80665
    #define PI   3.14159
}

struct imu_struct{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
};

class Sensors{

public:
    bool isISEnabled;
    struct imu_struct imu;
    struct imu_struct bias;

    Sensors ();
    Sensors (std::string sensor_name, bool debug);
    void update();
    void calibrateGyro();

private:
    bool is_debug;
    long unsigned time_now;
    InertialSensor *is;
    FILE * row_data_file;   // File to store row data

    void storeData();
    void getTime();
};

#endif //SENSORS_H
