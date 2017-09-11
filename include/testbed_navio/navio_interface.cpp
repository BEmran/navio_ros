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
    gyroCalibrate(ins, ahrs);

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
 getIMU: Read IMU and update AHRS
*****************************************************************************************/
void getIMU(InertialSensor *ins, AHRS *ahrs, imuStruct* imu, float dt) {

    //-------- Read raw measurements from the MPU and update AHRS --------------
    ins->update();
    ins->read_accelerometer(&imu->ax, &imu->ay, &imu->az);
    ins->read_gyroscope(&imu->gx, &imu->gy, &imu->gz);
    ins->read_magnetometer(&imu->mx, &imu->my, &imu->mz);

    //----------------- Rotating gyro axis by rotating +90 around z-axis -------------------
    float tmpgx = imu->gx, tmpax = imu->ax;
    imu->gx = imu->gy; 	          		// gx = gy
    imu->gy = -tmpgx;				// gy = -1 * gx
    imu->ax = imu->ay;				// ax = ay
    imu->ay = -tmpax;				// ay = -1 * ax

    //------------ Scale Accelerometer measurement by dividing by 9.81---------------
    imu->ax /= _G_SI;
    imu->ay /= _G_SI;
    imu->az /= _G_SI;

    //--------------------- Perfourm AHRS for Accelerometer + Gyro -----------------------
    ahrs->updateIMU(imu->ax, imu->ay, imu->az, imu->gx, imu->gy, imu->gz, dt);

    //------------ Perfourm AHRS for Accelerometer + Gyro + Magnetometer ---------
    //ahrs->update(imu->ax, imu->ay, imu->az,imu->gx,imu->gy, imu->gz,
    //imu->mx, imu->my, -imu->mz, dt);

    //------------------------------------ Read Euler angles ---------------------------------------
    ahrs->getEuler(&imu->r, &imu->p, &imu->w);

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
