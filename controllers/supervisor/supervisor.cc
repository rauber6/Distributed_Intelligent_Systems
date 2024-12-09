//******************************************************************************
//  Name:   supervisor.c
//  Author: -
//  Date: -
//  Rev: -
//******************************************************************************
#include "include/supervisor-lib.hpp"
#include <memory>

#define MOD 1 // 0 - centralised single task, 1 - distributed single task, 2 - centralised planning, 3 - distributed planning 

// MAIN LOOP (does steps)
int main(void) 
{
  
  std::shared_ptr<Supervisor> supervisor;
  

  switch (MOD)
  {
    case 0:
      supervisor = std::make_shared<SupervisorCentralised>();
      break;
    case 1:
      supervisor = std::make_shared<SupervisorDistributed>();
      break;
    case 2:
      supervisor = std::make_shared<SupervisorCentralised>();
      break;
  
    default:
      break;
  }
  // initialization
  wb_robot_init();
  link_event_nodes();
  wb_robot_step(STEP_SIZE);
  
  srand(time(NULL));
  supervisor->reset();

  // start the controller
  printf("Starting main loop...\n");
  while (wb_robot_step(STEP_SIZE) != -1)
  {
    if (!supervisor->step(STEP_SIZE)) break; //break at return = false
  }
  wb_supervisor_simulation_reset_physics();
  wb_robot_cleanup();
  exit(0);
  return 0;

}