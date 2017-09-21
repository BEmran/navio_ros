/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "testbed_navio/basic.h"
float du_cmd[4]={0.0,0.0,0.0,0.0};
void duCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
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
        my_data->du[0] = du_cmd[0];
        my_data->du[1] = du_cmd[1];
        my_data->du[2] = du_cmd[2];
        my_data->du[3] = du_cmd[3];
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
    ros::Subscriber du_sub = n.subscribe("testbed/cmd/du", 1000, duCmdCallback);
    ros::Subscriber encoder_sub = n.subscribe("testbed/sensors/encoders", 1000, encodersCallback);
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
duCmdCallback: Read cmanded du values and map it to gloabal variable
******************************************************************************************/
void duCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    du_cmd[0] = msg->twist.angular.x;
    du_cmd[1] = msg->twist.angular.y;
    du_cmd[2] = msg->twist.angular.z;
    du_cmd[3] = msg->twist.linear.x;
}
