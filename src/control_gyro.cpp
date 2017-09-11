/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "testbed_navio/basic.h"
void control(dataStruct* data, float dt);

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
    initializePWM(&my_data->pwm);
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
    ros::Subscriber encoder_sub = n.subscribe("testbed/sensors/encoders", 1000, encoderesCallback);
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
    uPWM[0] = dz - dp - dw;
    uPWM[1] = dz - dr + dw;
    uPWM[2] = dz + dp - dw;
    uPWM[3] = dz + dr + dw;

    uPWM[0] = uPWM[0] + _SERVO_MIN;
    uPWM[1] = uPWM[1] + _SERVO_MIN;
    uPWM[2] = uPWM[2] + _SERVO_MIN;
    uPWM[3] = uPWM[3] + _SERVO_MIN;
    //---------------------------------- send PWM duty cycle ------------------------------------
    setPWMDuty(pwm, uPWM);
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
control: Perfourm control loop
******************************************************************************************/
void control(dataStruct* data, float dt){

    static float ei = 0.0, ang0 = 0.0;
    float kd[3]={1.5,1.5,1.5};

    // control signal
    float ur = -data->imu.gx * kd[0]/10.0;
    //data->du = -data->imu.gy * kd[1];
    //data->du = -data->imu.gz * kd[2];

    // saturation
    sat(ur,0.3,-0.3);
    // Send control signal
    data->du[0] = ur;
    data->du[3] = 0.5;
}
