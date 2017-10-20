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
    printf("Start Program\n");
    signal(SIGINT, ctrlCHandler);

    //------------------------------------- Define main variables --------------------------------
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;
    data.is_sensor_ready = false;

    //----------------------------------------- Start threads ---------------------------------------
    pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) &data);
    pthread_create(&_Thread_Control, NULL, controlThread, (void *) &data);
    pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) &data);


    /* ---------- INITIALIZING VARIABLES ---------- */
    int client, server, portNum = 1500, bufsize = 1024;
    bool isExit = false;
    char buffer[bufsize];

    struct timeval tval;
    struct sockaddr_in server_addr;
    socklen_t size;

    /* ---------- ESTABLISHING SOCKET CONNECTION ----------*/
    client = socket(AF_INET, SOCK_STREAM, 0);
    if (client < 0)
    {
        cout << "\nError establishing socket..." << endl;
        exit(1);
    }

    cout << "\n=> Socket server has been created..." << endl;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(portNum);

    /* ---------- BINDING THE SOCKET ---------- */
    if ((bind(client, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0)
    {
        cout << "=> Error binding connection, the socket has already been established..." << endl;
        return -1;
    }
    size = sizeof(server_addr);
    cout << "=> Looking for clients..." << endl;
    listen(client, 1); // Listening call

    /* ------------- ACCEPTING CLIENTS  ------------- */
    int clientCount = 1;
    server = accept(client,(struct sockaddr *)&server_addr,&size);
    if (server < 0)  // first check if it is valid or not
        cout << "=> Error on accepting..." << endl;
    float sum = 0;
    strcpy(buffer, "=> Server connected...\n");
    send(server, buffer, bufsize, 0);
    cout << "=> Connected with the client #" << clientCount << ", you are good to go..." << endl;
    cout << "\n=> Enter # to end the connection\n" << endl;
    //------------------------------------------  Main loop ------------------------------------------
    char * pEnd;
    unsigned long int time, time0;
    while (_CloseRequested){
        recv(server, buffer, bufsize, 0);

        time = strtoul (buffer,&pEnd,10);
        time0 = strtoul (pEnd,&pEnd,10);
        float r = (float) strtod (pEnd,&pEnd);
        float p = (float) strtod (pEnd,&pEnd);
        float y = (float) strtod (pEnd,NULL);

        data.encoders[0] = r;
        data.encoders[1] = p;
        data.encoders[2] = y;

        sprintf(buffer,"%lu",time);
        send(server, buffer, bufsize, 0);

        if (*buffer == '#')
            break;
       //printf("%lu, %lu, %+5.5f, %+5.5f, %+5.5f\n",time, time0, r, p, y);
    }

    //---------------------------------------- Exit procedure -------------------------------------
    cout << "\n\n=> Connection terminated with IP " << inet_ntoa(server_addr.sin_addr);
    close(server);
    cout << "\nGoodbye..." << endl;
    close(client);

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
