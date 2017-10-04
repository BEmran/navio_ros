/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "navio_interface.h"

/*****************************************************************************************
 controlThread: Perfourmcontrol loop and send PWM output
*****************************************************************************************/
void initializePWM(PWM *pwm) {
    //--------------------------------------- Initialize PWM ------------------------------------------
    pwm->init(_MOTOR1);
    pwm->init(_MOTOR2);
    pwm->init(_MOTOR3);
    pwm->init(_MOTOR4);
    pwm->set_period(_MOTOR1, _FREQ);
    pwm->set_period(_MOTOR2, _FREQ);
    pwm->set_period(_MOTOR3, _FREQ);
    pwm->set_period(_MOTOR4, _FREQ);
    pwm->set_duty_cycle(_MOTOR1, _SERVO_MIN);
    pwm->set_duty_cycle(_MOTOR2, _SERVO_MIN);
    pwm->set_duty_cycle(_MOTOR3, _SERVO_MIN);
    pwm->set_duty_cycle(_MOTOR4, _SERVO_MIN);
    pwm->enable(_MOTOR1);
    pwm->enable(_MOTOR2);
    pwm->enable(_MOTOR3);
    pwm->enable(_MOTOR4);
}

/*****************************************************************************************
 imuSetup: Initialize IMU sensor and calibrate gyro sensor
*****************************************************************************************/
InertialSensor* imuSetup(AHRS *ahrs, char *sensor_name, imuStruct* imu) {
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
    gyroCalibrate(ins, ahrs);

    //------------------------------------- Calibrate mag ------------------------------------------
    printf("Start moving the testbed around...\n");
    sleep(2);
    magCalibrate(ins,imu->mag_offset,imu->mag_scale);
    //----------------------------------- Return INS object ---------------------------------------

    return ins;
}
/*****************************************************************************************
 gyroCalibrate: calibrate gyro sensor
*****************************************************************************************/
void gyroCalibrate(InertialSensor *ins, AHRS *ahrs) {
    //------------------------------------- Calibrate gyro ------------------------------------------
    printf("Beginning Gyro calibration...\n");
    float gx, gy, gz;
    float offset[3] = {0.0, 0.0, 0.0};
    int i_max = 1000;
    for (int i = 0; i < i_max; i++) {
        ins->update();
        ins->read_gyroscope(&gx, &gy, &gz);
        offset[0] += -gy;		// rotating gyro axis by rotating +90 around z-axis gx = gy
        offset[1] += -(-1*gx);	// gy = gx * -1
        offset[2] += -gz;
        usleep(5000);
    }
    offset[0] /= i_max;
    offset[1] /= i_max;
    offset[2] /= i_max;

    //----------------------------- Set & display offset result ------------------------------------
    printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
    ahrs->setGyroOffset(offset[0], offset[1], offset[2]);
}

/*****************************************************************************************
 magCalibrate: calibrate magntic sensor
*****************************************************************************************/
void magCalibrate(InertialSensor *ins,float mag_bias[3], float mag_scale[3]) {
    //------------------------------------- Calibrate gyro ------------------------------------------
    printf("Beginning Mag calibration...\n");
    int i_max = 1000;
    float mag_max[3] = {-32767, -32767, -32767};
    float mag_min[3] = {32767, 32767, 32767};
    float mag_temp[3] = {0, 0, 0};
    float dest2[3];
    for (int i = 0; i < i_max; i++) {
        ins->update();
        ins->read_magnetometer(&mag_temp[0], &mag_temp[1], &mag_temp[2]);
        float tmp = mag_temp[0];
        mag_temp[0] = mag_temp[1];
        mag_temp[1] = -tmp;

        for (int j= 0; j < 3; j++) {
            if(mag_temp[j] > mag_max[j])
                mag_max[j] = mag_temp[j];
            if(mag_temp[j] < mag_min[j])
                mag_min[j] = mag_temp[j];
        }
        usleep(10000);
    }

    // Get correction
    for (int j= 0; j < 3; j++) {
        // hard iron correction estimate
        mag_bias[j]  = (mag_max[j] + mag_min[j])/2;  // get average j mag bias in counts
        // Soft iron correction estimate
        mag_scale[j]  = (mag_max[j] - mag_min[j])/2;  // get average j axis max chord length in counts
    }

    float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2])/3.0;

    dest2[0] = avg_rad/(mag_scale[0]);
    dest2[1] = avg_rad/(mag_scale[1]);
    dest2[2] = avg_rad/(mag_scale[2]);

    //----------------------------- Set & display offset result ------------------------------------
    printf("mag_bias are: %f %f %f\n", mag_bias[0], mag_bias[1], mag_bias[2]);
    printf("mag_scale are: %f %f %f\n", mag_scale[0], mag_scale[1], mag_scale[2]);
    printf("avg_rad is: %f\n", avg_rad);
    printf("dest2 are: %f %f %f\n", dest2[0], dest2[1], dest2[2]);
}

/*****************************************************************************************
 getIMU: Read IMU and update AHRS
*****************************************************************************************/
void getIMU(InertialSensor *ins, imuStruct* imu) {

    //-------- Read raw measurements from the MPU and update AHRS --------------
    ins->update();
    ins->read_accelerometer(&imu->ax, &imu->ay, &imu->az);
    ins->read_gyroscope(&imu->gx, &imu->gy, &imu->gz);
    ins->read_magnetometer(&imu->mx, &imu->my, &imu->mz);

    //----------------- Rotating gyro axis by rotating +90 around z-axis -------------------
    float tmpgx = imu->gx, tmpax = imu->ax;
    imu->gx = imu->gy;				// gx = gy
    imu->gy = -tmpgx;		  		// gy = -1 * gx
    imu->ax = imu->ay;				// ax = ay
    imu->ay = -tmpax;			  	// ay = -1 * ax

    //------------ Scale Accelerometer measurement by dividing by 9.81---------------
    imu->ax /= _G_SI;
    imu->ay /= _G_SI;
    imu->az /= _G_SI;
}

/*****************************************************************************************
 gyroCalibrate: calibrate gyro sensor
*****************************************************************************************/
void imuFiltering(InertialSensor *ins, AHRS *ahrs) {
    //------------------------------------- Calibrate gyro ------------------------------------------
    printf("Beginning Gyro calibration...\n");
    float gx, gy, gz;
    float offset[3] = {0.0, 0.0, 0.0};
    int i_max = 1000;
    for (int i = 0; i < i_max; i++) {
        ins->update();
        ins->read_gyroscope(&gx, &gy, &gz);
        offset[0] += -gy;		// rotating gyro axis by rotating +90 around z-axis gx = gy
        offset[1] += -(-1*gx);	// gy = gx * -1
        offset[2] += -gz;
        usleep(10000);
    }
    offset[0] /= i_max;
    offset[1] /= i_max;
    offset[2] /= i_max;

    //----------------------------- Set & display offset result ------------------------------------
    printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
    ahrs->setGyroOffset(offset[0], offset[1], offset[2]);
}


/*****************************************************************************************
 setPWMADuty: send PWM signal to motor
*****************************************************************************************/
void setPWMDuty(PWM* pwm, float uPWM[4]) {
    //------------------------------ apply saturation for PWM ------------------------------------
    uPWM[0] = sat(uPWM[0], _SERVO_MAX, _SERVO_MIN);
    uPWM[1] = sat(uPWM[1], _SERVO_MAX, _SERVO_MIN);
    uPWM[2] = sat(uPWM[2], _SERVO_MAX, _SERVO_MIN);
    uPWM[3] = sat(uPWM[3], _SERVO_MAX, _SERVO_MIN);

    //------------------------------------- set PWM duty --------------------------------------------
    pwm->set_duty_cycle(_MOTOR1, uPWM[0]);
    pwm->set_duty_cycle(_MOTOR2, uPWM[1]);
    pwm->set_duty_cycle(_MOTOR3, uPWM[2]);
    pwm->set_duty_cycle(_MOTOR4, uPWM[3]);
}

/*****************************************************************************************
 sat: apply saturation
*****************************************************************************************/
float sat(float x, float upper, float lower) {
    if (x <= lower)
        x = lower;
    else if (x >= upper)
        x = upper;
    return x;
}

/*****************************************************************************************
 doAHRS: perfourm AHRS
*****************************************************************************************/
void doAHRS(AHRS *ahrs, imuStruct* imu, float dt) {

    //--------------------- Perfourm AHRS for Accelerometer + Gyro -----------------------
    ahrs->updateIMU(imu->ax, imu->ay, imu->az, imu->gx, imu->gy, imu->gz, dt);

    //------------ Perfourm AHRS for Accelerometer + Gyro + Magnetometer ---------
    //ahrs->update(imu->ax, imu->ay, imu->az,imu->gx,imu->gy, imu->gz,
    //imu->mx, imu->my, -imu->mz, dt);

    //------------------------------------ Read Euler angles ---------------------------------------
    ahrs->getEulerRad(&imu->r, &imu->p, &imu->w);
    imu->qx = ahrs->getX();
    imu->qy = ahrs->getY();
    imu->qz = ahrs->getZ();
    imu->qw = ahrs->getW();
}


/*****************************************************************************************
 doComplementaryFilter: perfourm Complementary Filter
*****************************************************************************************/
void doComplementaryFilter(imu_tools::ComplementaryFilter* comp_filter, imuStruct* imu, float dt){

    //------------------------------------- Update the filter ----------------------------------------
    comp_filter->update(imu->ax, imu->ay, imu->az,imu->gx,imu->gy, imu->gz,imu->mx, imu->my, -imu->mz, dt);

    //------------------------------------ Get the orientation ---------------------------------------
    double q0, q1, q2, q3;
    comp_filter->getOrientation(q0, q1, q2, q3);
    imu->qx = q1;
    imu->qy = q2;
    imu->qz = q3;
    imu->qw = q0;
    Quaternion2Euler(imu->r,imu->p,imu->w,q1, q2, q3, q0);

    //------------------------------------ Account for biases ---------------------------------------
    if (comp_filter->getDoBiasEstimation())
    {
        imu->gx -= comp_filter->getAngularVelocityBiasX();
        imu->gy -= comp_filter->getAngularVelocityBiasY();
        imu->gz -= comp_filter->getAngularVelocityBiasZ();
    }
}

/*****************************************************************************************
 Quaternion2Euler: convert Quaternion two Euler angles in rad
*****************************************************************************************/
void Quaternion2Euler(float& roll, float& pitch, float& yaw, float q0, float q1, float q2, float q3)
{
    roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
    pitch = asin(2*(q0*q2-q3*q1));
    yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}
