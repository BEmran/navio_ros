/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef NAVIO_BASIC
#define NAVIO_BASIC

// ********************************************************** //
// Header files:
#include <signal.h>       // signal ctrl+c
#include <stdio.h>        // printf
#include <cstdlib>
#include <unistd.h>      // usleep
#include <pthread.h>   // create thread

#include "../Navio/MPU9250.h"      // mpu sensor
#include "../Navio/LSM9DS1.h"     // lsm sensor
#include "../lib/AHRS.hpp"              // Mahony AHRS
#include "../lib/SamplingTime.h"    // samplig time

// ********************************************************** //
// Global variables:
#define G_SI 9.80665
bool _CloseRequested = false;
pthread_t _Thread_Sensors;
// ********************************************************** //
// Define structures:
struct imuStruct{
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float r, p, w;
};
struct dataStruct {
        imuStruct imu;
        InertialSensor *ins;
        AHRS ahrs;
        int argc;
        char** argv;
};

// ********************************************************** //
// Functions prototype:
void *sensorsThread(void *data);
void ctrlCHandler(int signal);
InertialSensor* imuSetup(AHRS *ahrs, char *sensor_name);
void getIMU(InertialSensor *ins, AHRS *ahrs, imuStruct* imu, float dt);

#endif // NAVIO_BASIC

