//******************************************************************************
//  Name:   e-puck.c
//  Author: -
//  Date:   -
//  Rev:    -
//******************************************************************************

#ifndef E_PUCK_LIB_H
#define E_PUCK_LIB_H


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>
#include <webots/motor.h>

#include <webots/supervisor.h>

#include "../supervisor/include/message.h"
#include "../supervisor/include/taskType.hpp"

#define MAX_SPEED_WEB 6.28 // Maximum speed webots

#define DEBUG 1
#define TIME_STEP 64 // Timestep (ms)
#define RX_PERIOD 2  // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH 0.052        // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628  // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205      // Wheel radius (meters)
#define DELTA_T TIME_STEP / 1000 // Timestep (seconds)
#define MAX_SPEED 500            // Maximum speed
#define PS_OFFSET -77            // Proximity sensor offset, to have the readings in void at around zero.


#define INVALID -999
#define BREAK -999 // for physics plugin

#define NUM_ROBOTS 5 // Change this also in the supervisor!

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_SUM 0 //500 // minimum value of all sensor inputs combined to change to obstacle avoidance mode
#define STATECHANGE_MAX 0 //minimum value for the max reading among the 8 proximity sensors to change state to avoid.

typedef enum
{
    STAY = 1,
    GO_TO_GOAL = 2, // Initial state aliases
    OBSTACLE_AVOID = 3,
    RANDOM_WALK = 4,
    PERFORMING_TASK = 5,
} robot_state_t;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS 8
#define BIAS_SPEED 100

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
extern int Interconn[16];


class Epuck
{
public:
    Epuck();

    void reset();
    void update_state(int _sum_distances, int _max_distance);
    void update_self_motion(int msl, int msr);
    void compute_avoid_obstacle(int *msl, int *msr, int distances[]);
    void compute_go_to_goal(int *msl, int *msr);
    void run(int ms);
    void receive_updates();
    
    // Abstract methods
    virtual void msgEventDone(message_t msg) = 0;
    virtual void msgEventWon(message_t msg) = 0;
    virtual void msgEventNew(message_t msg) = 0;
    virtual void msgEventCustom(message_t msg) = 0;  // to handle custom messages in sub-classes
    virtual void update_state_custom() = 0;  // to handle custom states in sub-classes
    virtual void run_custom_pre_update() = 0;  // to handle custom run instructions in sub-classes
    virtual void run_custom_post_update() = 0;  // to handle custom run instructions in sub-classes

protected:
    int clock;
    int clock_task;
    char task_in_progress;
    uint16_t robot_id;    // Unique robot ID
    TaskType robot_type;  // robot type
    robot_state_t state;  // State of the robot
    double my_pos[3];     // X, Z, Theta of this robot
    char target_valid;    // boolean; whether we are supposed to go to the target
    double target[99][4]; // x and z coordinates of target position (max 99 targets)
    // int lmsg, rmsg;       // Communication variables
    int indx;             // Event index to be sent to the supervisor
    int target_list_length;
    int collision_counter;

    // Proximity and radio handles
    WbDeviceTag emitter_tag, receiver_tag;
    WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors

    float buff[99]; // Buffer for physics plugin

    double stat_max_velocity;

    WbDeviceTag left_motor;  // handler for left wheel of the robot
    WbDeviceTag right_motor; // handler for the right wheel of the robot

};

class EpuckCentralized : public Epuck{
    public:
        EpuckCentralized();

    private:
        void msgEventDone(message_t msg) override;
        void msgEventWon(message_t msg) override;
        void msgEventNew(message_t msg) override;
        void msgEventCustom(message_t msg) override;
        void update_state_custom() override;
        void run_custom_pre_update() override;
        void run_custom_post_update() override;

};

class EpuckDistributed : public Epuck{
    public:
        EpuckDistributed();
};


double rnd(void);

void limit(int *number, int limit);

double dist(double x0, double y0, double x1, double y1);
#endif
