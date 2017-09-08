/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "../include/testbed_navio/navio_basic.h"

/*****************************************************************************************
main: Run main function
 ****************************************************************************************/
int main(int argc, char** argv) {
    // Welcome msg
    printf("Start Program\n");
    signal(SIGINT, ctrlCHandler);

    // Define main variables
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;

    // Start threads
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);

    // Main loop to read encoders value
    while (!_CloseRequested) {
        printf("main\n");
        sleep(1);
    }

    // Exit procedure
    pthread_cancel(_Thread_Sensors);
    printf("Close program\n");

    return 0;
}

/*****************************************************************************************
 ctrlCHandler: Detect ctrl+c to quit program
 ****************************************************************************************/
void ctrlCHandler(int signal) {
    _CloseRequested = true;
    printf("Ctrl+c have been detected\n");
}

/*****************************************************************************************
 sensorsThread: read navio sensors (IMU +...) and perfourm AHRS
 *****************************************************************************************/
void *sensorsThread(void *data) {
    printf("Start Sensors thread\n");

    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;
    char imu_name[] = "mpu";
    my_data->ins = imuSetup(&my_data->ahrs, imu_name);
    if (my_data->ins == NULL) {
        printf("Cannot initialize imu sensor\n");
        pthread_exit(NULL);
    }
    printf("Initialized imu sensor\n");
    SamplingTime st(500);
    float dt, dtsumm = 0;
    // Main loop
    while (!_CloseRequested) {
        dt = st.tsCalculat();
        //-------------------------------------- Read Sensor --------------------------------------
        getIMU(my_data->ins, &my_data->ahrs, &my_data->imu, dt);
        dtsumm += dt;
        if (dtsumm > 1) {
            dtsumm = 0;
            printf("Sensors thread with ROLL: %+03.2f PITCH: %+03.2f YAW: %+03.2f %d Hz\n"
                   , my_data->imu.r, my_data->imu.p, my_data->imu.w * -1, int(1 / dt));
        }
    }
    // Exit procedure
    printf("Exit sensor thread\n");
    pthread_exit(NULL);
}

/*****************************************************************************************
 imuSetup: Initialize IMU sensor and calibrate gyro sensor
 *****************************************************************************************/
InertialSensor* imuSetup(AHRS *ahrs, char *sensor_name) {
    InertialSensor* ins;
    //--------------------------------------- Create IMU --------------------------------------------
    if (!strcmp(sensor_name, "mpu")) {
        printf("Selected: MPU9250\n");
        ins = new MPU9250();
    } else if (!strcmp(sensor_name, "lsm")) {
        printf("Selected: LSM9DS1\n");
        ins = new LSM9DS1();
    } else {
        return NULL;
    }

    //----------------------------------- MPU initialization ---------------------------------------
    ins->initialize();

    //------------------------------------- Calibrate gyro ------------------------------------------
    printf("Beginning Gyro calibration...\n");
    float gx, gy, gz;
    float offset[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 1000; i++) {
        ins->update();
        ins->read_gyroscope(&gx, &gy, &gz);
        offset[0] += -gx;
        offset[1] += -gy;
        offset[2] += -gz;
        usleep(10000);
    }
    offset[0] /= 100.0;
    offset[1] /= 100.0;
    offset[2] /= 100.0;

    //----------------------------- Set & display offset result ------------------------------------
    printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
    ahrs->setGyroOffset(offset[0], offset[1], offset[2]);

    return ins;
}

/*****************************************************************************************
 getIMU: Read IMU and update AHRS
 *****************************************************************************************/
void getIMU(InertialSensor *ins, AHRS *ahrs, imuStruct* imu, float dt) {

    //-------- Read raw measurements from the MPU and update AHRS --------------
    ins->update();
    ins->read_accelerometer(&imu->ax, &imu->ay, &imu->az);
    ins->read_gyroscope(&imu->gx, &imu->gy, &imu->gz);
    ins->read_magnetometer(&imu->mx, &imu->my, &imu->mz);
    // Scale Accelerometer measurement by dividing by 9.81
    imu->ax /= G_SI;
    imu->ay /= G_SI;
    imu->az /= G_SI;
    //
    if (imu->gz <= 0.01 && imu->gz >= -0.01)
        imu->gz = 0.0;
    // Accelerometer + Gyro
    ahrs->updateIMU(imu->ax, imu->ay, imu->az, imu->gx, imu->gy, imu->gz, dt);
    // Accelerometer + Gyro + Magnetometer
    //ahrs->update(imu->ax, imu->ay, imu->az,imu->gx,imu->gy, imu->gz,
    //imu->mx, imu->my, -imu->mz, dt);

    //------------------------------------ Read Euler angles ---------------------------------------
    ahrs->getEuler(&imu->r, &imu->p, &imu->w);
}
