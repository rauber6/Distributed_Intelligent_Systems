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
#include <iostream>
#include <algorithm>

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
#define EVENT_RANGE (0.1) // also defined in supervisor-lib.hpp

#define DEBUG 0
#define TIME_STEP 64 // Timestep (ms)
#define RX_PERIOD 2  // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH 0.052        // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628  // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205      // Wheel radius (meters)
#define DELTA_T TIME_STEP / 1000 // Timestep (seconds)

#define MAX_SPEED 500            // Maximum speed
#define PS_OFFSET -77            // Proximity sensor offset, to have the readings in void at around zero.
#define BATTERY_LIFE (2*60*1000) // Max number of timesteps robot can spend performing a task (includes going and waiting until its finished)


#define INVALID -999
#define BREAK -999 // for physics plugin

#define NUM_ROBOTS 5 // Change this also in the supervisor!
// #define NUM_TASKS 10 // set this in message.h

#define WB_CHANNEL_BROADCAST -1
#define SUP_REC_BASE_CHANNEL 900

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
    TASK_COMPLETED = 6,
    OUT_OF_BATTERY = 7
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
    virtual ~Epuck() {}


    virtual void reset();
    void update_state(int _sum_distances, int _max_distance);
    void update_self_motion(int msl, int msr);
    void compute_avoid_obstacle(int *msl, int *msr, int distances[]);
    void compute_go_to_goal(int *msl, int *msr);
    void run(int ms);
    void receive_updates();
    bool check_if_event_reached();
    
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
    int clock_prev;         // Previous clock time for tracking active robot time
    int clock_task;         // Time when the robot started waiting on a task
    char task_in_progress;  // Indicates robot waiting on a task to be completed 
    int time_active;        // Tracks robot active time of performing tasks
    uint16_t robot_id;      // Unique robot ID
    TaskType robot_type;    // robot type
    robot_state_t state;    // State of the robot
    double my_pos[3];       // X, Z, Theta of this robot
    char target_valid;      // boolean; whether we are supposed to go to the target
    double target[99][4];   // x and z coordinates of target position (max 99 targets)
    // int lmsg, rmsg;      // Communication variables
    int indx;               // Event index to be sent to the supervisor
    int target_list_length;
    int collision_counter;

    WbDeviceTag left_motor;  // handler for left wheel of the robot
    WbDeviceTag right_motor; // handler for the right wheel of the robot
    
    // Proximity and radio handles
    WbDeviceTag emitter_tag, receiver_tag;
    WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors

    float buff[99]; // Buffer for physics plugin

    double stat_max_velocity;

};

class EpuckCentralized : public Epuck{
    public:
        EpuckCentralized(int plan_length = 0, bool task_timeout_off = false);
        ~EpuckCentralized() override {}
    private:
        int plan_length;            // Maximum plan length for multi-task assignement task
        bool task_timeout_off;      // Allow robot to inform the supervisor it will not be bidding for a task
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
        ~EpuckDistributed() override {};
        void reset() override;

    private:

        int64_t a[1000];
        static constexpr double EMITTER_RANGE = 0.3; //[m]

        int x[NUM_TASKS];  // tracks tasks (-1 not announced, 0 not assigned, >0 assigned with that bid)
        int y_bids[NUM_TASKS];   // tracks local knowledge of the market (best bid on the market for each task)
        int y_winners[NUM_TASKS];  // tracks local knowledge of the market (winner for each task)
        int h[NUM_TASKS];   // track personal valid tasks and thier corresponding bid (if =0 task not valid, otherwise this is bid)
        task_t t[NUM_TASKS];  // keep info about all tasks

        int assigned_task;
        int newly_received_task;
        int start_received_task;
        bool can_start_broadcatsting;
        WbDeviceTag emitter_tag_sup;

        void msgEventDone(message_t msg) override;
        void msgEventWon(message_t msg) override;
        void msgEventNew(message_t msg) override;
        void msgEventCustom(message_t msg) override;
        void update_state_custom() override;
        void run_custom_pre_update() override;
        void run_custom_post_update() override;
        int compare_bids(int bid1, int bid2);
        int is_my_bid_better(int myBid, int otherBid);
        int compute_bid(task_t task);
        bool is_assigned();
};

class EpuckDistributedPlan : public Epuck{
    public:
        EpuckDistributedPlan();
        ~EpuckDistributedPlan() override {};
        void reset() override;

    private:

        int64_t a[10000];
        static constexpr double EMITTER_RANGE = 0.3; //[m]

        static constexpr int plan_length = 3;

        int b_length = 0;
        int p[NUM_TASKS]; // order of tasks based on their location in the plan

        int x[NUM_TASKS];  // tracks tasks (-1 not announced, 0 not assigned, >0 assigned with that bid)
        int y_bids[NUM_TASKS];   // tracks local knowledge of the market (best bid on the market for each task)
        int y_winners[NUM_TASKS];  // tracks local knowledge of the market (winner for each task)
        int h[NUM_TASKS];   // track personal valid tasks and thier corresponding bid (if =0 task not valid, otherwise this is bid)
        task_t t[NUM_TASKS];  // keep info about all tasks
        // bool G[NUM_ROBOTS];  // adjacency vector

        int assigned_task;
        int newly_received_task;
        int start_received_task;
        bool can_start_broadcatsting;
        WbDeviceTag emitter_tag_sup;

        void msgEventDone(message_t msg) override;
        void msgEventWon(message_t msg) override;
        void msgEventNew(message_t msg) override;
        void msgEventCustom(message_t msg) override;
        void update_state_custom() override;
        void run_custom_pre_update() override;
        void run_custom_post_update() override;
        int compare_bids(int bid1, int bid2);
        int is_my_bid_better(int myBid, int otherBid);
        int compute_bid(task_t task, double expected_time);
        double compute_cumulative_bid(int indx);
        bool is_assigned();
        void next_into_plan();
};

double rnd(void);

void limit(int *number, int limit);

double dist(double x0, double y0, double x1, double y1);
#endif
