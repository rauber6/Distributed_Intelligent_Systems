//******************************************************************************
//  Name:   e-puck.c
//  Author: -
//  Date:   -
//  Rev:    -
//******************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include "include/e-puck-lib.hpp"

int main(int argc, char **argv) 
{
    EpuckCentralized e = EpuckCentralized();
    e.reset();
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {e.run(TIME_STEP);}
    wb_robot_cleanup();


    return 0;
}