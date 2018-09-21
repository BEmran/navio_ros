/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "testbed_navio/testbed_full_old.h"
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
    data.is_sensors_ready = false;
    data.is_maintcp_ready = false;
    data.is_control_ready = false;
    data.pwm_offset[0] = 0;
    data.pwm_offset[1] = 0;
    data.pwm_offset[2] = 0;
    data.pwm_offset[3] = 0;
    float w0[3] = {50.0,50.0,50.0};
    data.wSys = DynSys(3, *wdotDyn,w0);
    data.w = data.wSys.getY();
    struct tcpStruct tcp;
    tcp.portNum = 1500;
    //----------------------------------------- Start threads ---------------------------------------
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);
    //------------------------------------------ initialize tcp ----------------------------------------
    data.is_maintcp_ready = initTcp(&tcp);
    SamplingTime st(_MAINTCP_FREQ);
    //---------------------------- Close record file -----------------------------------
    char file_name[32];
    char dir_name[] = "/home/pi";
    int fileNumber = 0;
    //---------------------------- Create new record file -------------------------------
    do {
        sprintf(file_name, "%s/test_%.2d.csv", dir_name, fileNumber++);
    } while (access(file_name, F_OK) == 0);
    data->*file = fopen(image_name, "w");
    //------------------------------ Check the file name --------------------------------
    if (data->file == NULL) {
        printf("Error creating file!\n");
        exit(1);
    }
    //-------------------------- Display Information for user ---------------------------
    printf("a file successfully created to record testbed data\n");
    printf("record file path and name: %s",file_name);
    //------------------------ Record starting time of test -----------------------------
    time_t rawtime;
    time (&rawtime);
    struct tm * timeinfo = localtime (&rawtime);
    sprintf(file_name,"Current local time and date: %s", asctime(timeinfo));
    //-------------------------------------------- Min loop ------------------------------------------
    float dt, dtsumm = 0;
    bool enablePrint = false;
    while(!_CloseRequested && data.is_maintcp_ready){
        dt = st.tsCalculat();
        getTcpData(&tcp, &data, dt, enablePrint);     
        dtsumm += dt;
        if (dtsumm > 2) {
            dtsumm = 0;
            printf("maintcp thread with %d Hz\n", int(1 / dt));
        }
    }
    //------------------------------------- Close record file -----------------------------------
    fclose(file);
    //----------------------------------- Terminate tcp connection -----------------------------
    printf("\n\n=> Connection terminated");
    close(tcp.server);
    close(tcp.client);
    //----------------------------------------- Exit procedure ---------------------------------
    pthread_cancel(_Thread_Sensors);
    pthread_cancel(_Thread_Control);
    pthread_cancel(_Thread_RosNode);

    printf("Close program\n");
    return 0;
}

/*****************************************************************************************
control: Perfourm control loop
******************************************************************************************/
void control(dataStruct* data, float dt){
    for (int i=0; i <4;i++){
        data->du[i] = data->rosnode->_cmd_du[i];
    }
}
