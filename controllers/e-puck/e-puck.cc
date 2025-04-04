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
#include <memory>

#include "include/e-puck-lib.hpp"

#define MOD 3 // 0 - centralised single task, 1 - distributed single task, 2 - centralised planning, 3 - distributed planning 

int main(int argc, char **argv) 
{

    std::shared_ptr<Epuck> epuck;

    switch (MOD)
  {
    case 0:
      epuck = std::make_shared<EpuckCentralized>(1, false); // plan_length = 1 and task_timeout_off = false
      break;
    case 1:
      epuck = std::make_shared<EpuckDistributed>();
      break;
    case 2:
      epuck = std::make_shared<EpuckCentralized>(3, false); // plan_length = 3 and task_timeout_off = false
      break;
    case 3:
      epuck = std::make_shared<EpuckDistributedPlan>(); 
      break;
  
    default:
      break;
  }

    epuck->reset();
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {
      epuck->run(TIME_STEP);
    }
    wb_robot_cleanup();

    return 0;
}