/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 7, 2017, 1:11 PM
 */
#include "../include/testbed_navio/navio_basic.h"

int main(int argc, char** argv) {
    // Welcome msg
    printf("Start Program\n");
    signal(SIGINT, ctrlCHandler);

    // Define main variables
    struct dataStruct data;
    data.argc = argc;
    data.argv = argv;

    // Main loop to read encoders value
    while (!_CloseRequested) {
        printf("main\n");
        sleep(1);
    }

    // Exit procedure
    printf("Close program\n");

    return 0;
}

// ********************************************************** //
// Detect ctrl+c

void ctrlCHandler(int signal) {
    _CloseRequested = true;
    printf("Ctrl+c have been detected\n");
}

