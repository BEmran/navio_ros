/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "../include/testbed_navio/navio_basic.h"
float cmd_ang[3] = {0.0,0.0,0.0};
void control(dataStruct* data, float dt);
void angCmdCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

/*****************************************************************************************
main: Run main function
 ****************************************************************************************/
int main(int argc, char** argv) {
    //----------------------------------------- Welcome msg -------------------------------------
    printf("Start Program\n");
    signal(SIGINT, ctrlCHandler);

    //------------------------------------- Define main variables --------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;

    //----------------------------------------- Start threads ---------------------------------------
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);

    //------------------------------------------  Main loop ------------------------------------------
    while (!_CloseRequested) {
        printf("main\n");
        sleep(1);
    }

    //---------------------------------------- Exit procedure -------------------------------------
    pthread_cancel(_Thread_Sensors);
    pthread_cancel(_Thread_Control);
    pthread_cancel(_Thread_RosNode);
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

    //----------------------------------------  Initialize IMU ----------------------------------------
    char imu_name[] = "mpu";
    my_data->ins = imuSetup(&my_data->ahrs, imu_name);
    if (my_data->ins == NULL) {
        printf("Cannot initialize imu sensor\n");
        pthread_exit(NULL);
    }
    printf("Initialized imu sensor\n");

    //------------------------------------------  Main loop ------------------------------------------
    SamplingTime st(_SENSOR_FREQ);
    float dt, dtsumm = 0;
    while (!_CloseRequested) {
        dt = st.tsCalculat();

        //-------------------------------------- Read Sensor ---------------------------------------
        getIMU(my_data->ins, &my_data->ahrs, &my_data->imu, dt);
        dtsumm += dt;
        if (dtsumm > 1) {
            dtsumm = 0;
            printf("Sensors thread with ROLL: %+03.2f PITCH: %+03.2f YAW: %+03.2f %d Hz\n"
                   , my_data->imu.r, my_data->imu.p, my_data->imu.w * -1, int(1 / dt));
        }
    }

    //---------------------------------------- Exit procedure -------------------------------------
    printf("Exit sensor thread\n");
    pthread_exit(NULL);
}

// Control thread
/*****************************************************************************************
 controlThread: Perfourmcontrol loop and send PWM output
*****************************************************************************************/
void *controlThread(void *data) {
    printf("Start Control thread\n");

    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;
    SamplingTime st(400);
    float dt, dtsumm = 0;
    //--------------------------------------- Initialize PWM ------------------------------------------
    my_data->pwm.init(_MOTOR1);
    my_data->pwm.init(_MOTOR2);
    my_data->pwm.init(_MOTOR3);
    my_data->pwm.init(_MOTOR4);
    my_data->pwm.set_period(_MOTOR1, _FREQ);
    my_data->pwm.set_period(_MOTOR2, _FREQ);
    my_data->pwm.set_period(_MOTOR3, _FREQ);
    my_data->pwm.set_period(_MOTOR4, _FREQ);
    my_data->pwm.set_duty_cycle(_MOTOR1, _SERVO_MIN);
    my_data->pwm.set_duty_cycle(_MOTOR2, _SERVO_MIN);
    my_data->pwm.set_duty_cycle(_MOTOR3, _SERVO_MIN);
    my_data->pwm.set_duty_cycle(_MOTOR4, _SERVO_MIN);
    my_data->pwm.enable(_MOTOR1);
    my_data->pwm.enable(_MOTOR2);
    my_data->pwm.enable(_MOTOR3);
    my_data->pwm.enable(_MOTOR4);

    my_data->du[0] = 0.0;
    my_data->du[1] = 0.0;
    my_data->du[2] = 0.0;
    my_data->du[3] = 0.0;
    //------------------------------------- Wait for user input --------------------------------------
    int x = 0;
    while (x == 0) {
        printf("Enter 1 to start control\n");
        cin >> x;
        sleep(1);
    }

    //------------------------------------------  Main loop -------------------------------------------
    while (!_CloseRequested) {
        dt = st.tsCalculat();
        control(my_data,dt);
        du2motor(&my_data->pwm,my_data->du[0],my_data->du[1],my_data->du[2],my_data->du[3]);
        dtsumm += dt;
        if (dtsumm > 1) {
            dtsumm = 0;
            printf("Control thread with %d Hz\n", int(1 / dt));
        }
    }
    //---------------------------------------- Exit procedure ---------------------------------------
    printf("Exit control thread\n");
    pthread_exit(NULL);
}

/*****************************************************************************************
 rosNodeThread: ROS Node thread
 *****************************************************************************************/
void *rosNodeThread(void *data) {
    printf("Start ROS Node thread\n");
    struct dataStruct *my_data;
    my_data = (struct dataStruct *) data;

    //--------------------------------------- Initialize ROS ------------------------------------------
    ros::init(my_data->argc,my_data->argv,"navio_basic");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise <sensor_msgs::Imu>("testbed/sensors/imu", 1000);
    ros::Publisher du_pub = n.advertise <geometry_msgs::TwistStamped>("testbed/motors/du", 1000);
    ros::Subscriber encoder_sub = n.subscribe("testbed/sensors/encoders", 1000, &encoderesCallback);
    ros::Subscriber ang_cmd_sub = n.subscribe("testbed/cmd/ang", 100, &angCmdCallback);
    ros::Rate loop_rate(_ROS_FREQ);
    sensor_msgs::Imu imu_msg;
    geometry_msgs::TwistStamped du_msg;

    //------------------------------------------  Main loop -------------------------------------------
    while (ros::ok() && !_CloseRequested)
    {
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.seq++;
        imu_msg.angular_velocity.x = my_data->imu.gx;
        imu_msg.angular_velocity.y = my_data->imu.gy;
        imu_msg.angular_velocity.z = my_data->imu.gz;
        imu_msg.linear_acceleration.x = my_data->imu.ax;
        imu_msg.linear_acceleration.y = my_data->imu.ay;
        imu_msg.linear_acceleration.z = my_data->imu.az;
        imu_msg.orientation.x = my_data->ahrs.getX();
        imu_msg.orientation.y = my_data->ahrs.getY();
        imu_msg.orientation.z = my_data->ahrs.getZ();
        imu_msg.orientation.w = my_data->ahrs.getW();
        imu_pub.publish(imu_msg);

        du_msg.header.stamp = ros::Time::now();
        du_msg.header.seq++;
        du_msg.twist.angular.x = my_data->du[0];
        du_msg.twist.angular.y = my_data->du[1];
        du_msg.twist.angular.z = my_data->du[2];
        du_msg.twist.linear.x = my_data->du[3];
        du_pub.publish(du_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    //---------------------------------------- Exit procedure ---------------------------------------
    ctrlCHandler(0);
    printf("Exit ROS Node thread\n");
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
    int i_max = 1000;
    for (int i = 0; i < i_max; i++) {
        ins->update();
        ins->read_gyroscope(&gx, &gy, &gz);
        offset[0] += -gx;
        offset[1] += -gy;
        offset[2] += -gz;
        usleep(10000);
    }
    offset[0] /= i_max;
    offset[1] /= i_max;
    offset[2] /= i_max;

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
    imu->ax /= _G_SI;
    imu->ay /= _G_SI;
    imu->az /= _G_SI;
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

/*****************************************************************************************
 du2motor: map du to PWM and send signal to motors
 *****************************************************************************************/
void du2motor(PWM* pwm, float du0, float du1, float du2, float du3) {

    //---------------------------------- apply saturation for du -----------------------------------
    float dr = sat(du0, _MAX_ROLL   , -_MAX_ROLL    ) / 2.0;
    float dp = sat(du1, _MAX_PITCH  , -_MAX_PITCH   ) / 2.0;
    float dw = sat(du2, _MAX_YAW    , -_MAX_YAW     ) / 4.0;
    float dz = sat(du3, _MAX_Thrust , 0             ) / 1.0;

    //----------------------------------------- du to PWM ------------------------------------------
    float uPWM[4];
    uPWM[0] = dz - dp - dw + _SERVO_MIN;
    uPWM[1] = dz - dr + dw + _SERVO_MIN;
    uPWM[2] = dz + dp - dw + _SERVO_MIN;
    uPWM[3] = dz + dr + dw + _SERVO_MIN;

    //---------------------------------- send PWM duty cycle ------------------------------------
    setPWMDuty(pwm, uPWM);
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
encoderesCallback: Read encoders and map it to gloabal variable
******************************************************************************************/
void encoderesCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    encoderes.header = msg->header;
    encoderes.vector = msg->vector;
    // ROS_INFO("I heard: [%f]", encoderes.vector.x);
}

/*****************************************************************************************
angCmdCallback: Read command signal for angle
******************************************************************************************/
void angCmdCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    cmd_ang[0] = msg->vector.x;
    cmd_ang[1] = msg->vector.y;
    cmd_ang[2] = msg->vector.z;
}

/*****************************************************************************************
control: Perfourm control loop
******************************************************************************************/
void control(dataStruct* data, float dt){

    static float ei = 0.0, ang0 = 0.0;
    float ki = 21.2959, k1 = 6.6207, k2 = 1.5945;

    // defferentiate;
    float angdot = (encoderes.vector.x - ang0)/dt;
    ang0 = encoderes.vector.x;
    // traking error
    float e = cmd_ang[0] - encoderes.vector.x;
    // control signal
    float ur = - e * k1 - angdot * k2 + ei * ki;
    ei += e * dt;
    // saturation
    if (ur > 0.3)
        ur = 0.3;
    else if (ur < -0.3)
        ur = -0.3;
    // Send control signal
    data->du[0] = ur;
    data->du[3] = 0.5;
}

