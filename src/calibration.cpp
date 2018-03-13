/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "testbed_navio/basic.h"
/*****************************************************************************************
main: Run main function
 ****************************************************************************************/
int main(int argc, char** argv) {
    //----------------------------------------- Welcome msg -------------------------------------
    printf("Start Program...\n");
    signal(SIGINT, ctrlCHandler);

    //------------------------------------- Define main variables --------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;
    data.is_sensor_ready = false;
    data.is_tcp_ready = false;

    struct tcpStruct tcp;
    tcp.portNum = 1500;
    //----------------------------------------- Start threads ---------------------------------------


    //------------------------------------------ initialize tcp ----------------------------------------
    data.is_tcp_ready = initTcp(&tcp);
    SamplingTime st(1000);
    float dt, dtsumm = 0;
    bool enablePrint;
    //-------------------------------------------- Min loop ------------------------------------------
    while(!_CloseRequested && data.is_tcp_ready){
        dt = st.tsCalculat();

        getTcpData(&tcp, &data, enablePrint);
        enablePrint = false;
        dtsumm += dt;
        if (dtsumm > 2) {
            dtsumm = 0;
            enablePrint = true;
            printf("main thread with %d Hz\n", int(1 / dt));
        }
    }

    //----------------------------------- Terminate tcp connection -----------------------------
    printf("\n\n=> Connection terminated");
    close(tcp.server);
    close(tcp.client);

    //----------------------------------------- Exit procedure -------------------------------------

    pthread_cancel(_Thread_Sensors);
    printf("Close program\n");

    return 0;
}


/*****************************************************************************************
 sensors: read navio sensors (IMU +...) and perfourm AHRS
 *****************************************************************************************/
void sensors(dataStruct *my_data) {
    printf("Start Sensors \n");

    //----------------------------------------  Initialize IMU ----------------------------------------
    char imu_name[] = "mpu";
    //char imu_name[] = "lsm";
    my_data->ins = imuSetup(&my_data->ahrs, imu_name, &my_data->imu);
    if (my_data->ins == NULL) {
        printf("Cannot initialize imu sensor\n");
        pthread_exit(NULL);
    }
    printf("Initialized imu sensor\n");
    my_data->is_sensor_ready = true;

    //------------------------------------------  Main loop ------------------------------------------
    SamplingTime st(_SENSOR_FREQ);
    float dt, dtsumm = 0;
    while (!_CloseRequested) {
        dt = st.tsCalculat();

        //-------------------------------------- Read Sensor ---------------------------------------
        getIMU(my_data->ins, &my_data->imu);

        //------------------------------------- Perfourm filter ---------------------------------------
        //doAHRS(&my_data->ahrs, &my_data->imu, dt);
        doComplementaryFilter(&my_data->comp_filter_, &my_data->imu, dt);

        //-------------------------------------- Display data ---------------------------------------
        dtsumm += dt;
        if (dtsumm > 2) {
            dtsumm = 0;
            printf("Sensors thread with ROLL: %+03.2f PITCH: %+03.2f YAW: %+03.2f %d Hz\n"
                   , my_data->imu.rpy[0], my_data->imu.rpy[1], my_data->imu.rpy[2] * -1, int(1 / dt));
        }
    }

    //---------------------------------------- Exit procedure -------------------------------------
    printf("Exit sensor thread\n");
    pthread_exit(NULL);
}
