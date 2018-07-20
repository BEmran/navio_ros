/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on March 14, 2018
 */
#include "testbed_navio/testbed_demo.h"
/******************************************************************************
main: Run main function
******************************************************************************/
int main(int argc, char** argv) {

    // Welcome msg ------------------------------------------------------------
    printf("Start Program...\n");
    // conncet ctrl+c Handler
    signal(SIGINT, ctrlCHandler);

    // Define main variables --------------------------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;
    data.is_mainfun_ready = false;
    data.is_control_ready = false;
    data.is_rosnode_ready = false;
    data.is_sensors_ready = false;
    TimeSampling st(_MAINFUN_FREQ);
    for (int i = 0; i < 25; ++i) {
        data.record[i] = 0.0;       // initialize record data with zeros
    }

    // Start threads ----------------------------------------------------------
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);
     pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);

    // Create new record file -------------------------------------------------
    char file_name[64];
    int fileNumber = 0;
    // find avaliable file name & number
    do {
        sprintf(file_name, "/home/pi/testbed_data_%.2d.csv", fileNumber++);
    } while (access(file_name, F_OK) == 0);
    // open avaliable file
    data.file = fopen(file_name, "w");
    // check file
    if (data.file == NULL) {
        printf("Error creating file!\n");
        exit(1);
    }

    // Display Information for user
    printf("A file successfully created to record testbed data\n");
    printf("Record file path and name: %s\n",file_name);

    // Record starting time of test -------------------------------------------
    time_t rawtime;
    time (&rawtime);
    struct tm * timeinfo = localtime (&rawtime);
    fprintf(data.file,"Current local time and date: %s", asctime(timeinfo));

    // Print data header
    fprintf(data.file, "time,"
                       "ax,ay,az,"
                       "gx,gy,gz,"
                       "mx,my,mz,"
                       "enc0,enc1,enc2,"
                       "roll,pitch,yaw,"
                       "ur,up,uw,uz,"
                       "d0,d1,d2,d3,d4\n");

    // Wait for user to be ready ----------------------------------------------
    while(!data.is_rosnode_ready || !data.is_control_ready || !data.is_sensors_ready);
    int x = 0;
    while (x == 0) {
        printf("Enter 1 to start control\n");
        cin >> x;
        sleep(1);
    }
    data.is_mainfun_ready = true;

    // Main loop --------------------------------------------------------------
    float dt, dtsumm = 0;
    while(!_CloseRequested){
        dt = st.updateTs();
        dtsumm += dt;
        //printf("dt = %f\n",dt);
        if (dtsumm > 1) {
            dtsumm = 0;
            printf("Mainfun thread: running fine with %4d Hz\n", int(1 / dt));
        }
    }

    // Exit procedure ---------------------------------------------------------
    ctrlCHandler(0);
    printf("Close program\n");
    return 0;
}

/*****************************************************************************************
control: Perfourm control loop
******************************************************************************************/
void control(dataStruct* data, float dt){
    for (int i = 0; i < 4; i++){
        data->du[i] = data->rosnode->_cmd_du[i];
    }
}
