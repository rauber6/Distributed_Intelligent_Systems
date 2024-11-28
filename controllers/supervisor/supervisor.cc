//******************************************************************************
//  Name:   supervisor.c
//  Author: -
//  Date: -
//  Rev: -
//******************************************************************************

#include "include/supervisor-lib.hpp"

//Links up all the nodes we are interested in.
//Gets called by webots at robot_live(reset)


// MAIN LOOP (does steps)
int main(void) 
{
  Supervisor supervisor;
  
  // initialization
  wb_robot_init();
  link_event_nodes();
  wb_robot_step(STEP_SIZE);
  
  srand(time(NULL));
  supervisor.reset();

  // start the controller
  printf("Starting main loop...\n");
  while (wb_robot_step(STEP_SIZE) != -1)
  {
    if (!supervisor.step(STEP_SIZE)) break; //break at return = false
  }
  wb_supervisor_simulation_reset_physics();
  wb_robot_cleanup();
  exit(0);
  return 0;

}