/*
 * File:   main.hpp
 * Author: Bara ESmran
 * Created on September 7, 2017, 1:14 PM
 */

#ifndef NAVIO_BASIC
#define NAVIO_BASIC

// ********************************************************** //
// Header files:
#include <signal.h>       // signal ctrl+c
#include <stdio.h>        // printf
#include <cstdlib>
#include <unistd.h>     // usleep

// ********************************************************** //
// Global variables:
bool _CloseRequested = false;

// ********************************************************** //
// Define structures:
struct dataStruct {
        int argc;
        char** argv;
};

// ********************************************************** //
// Functions prototype:
void ctrlCHandler(int signal);

#endif // NAVIO_BASIC

