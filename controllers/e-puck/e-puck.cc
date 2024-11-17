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

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>
#include <webots/motor.h>
  
#include <webots/supervisor.h> 

#include "../supervisor/message.h" 
// #include "../supervisor/taskType.h"

#define MAX_SPEED_WEB      6.28    // Maximum speed webots
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot


#define DEBUG 1
#define TIME_STEP           64      // Timestep (ms)
#define RX_PERIOD           2    // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define DELTA_T             TIME_STEP/1000   // Timestep (seconds)
#define MAX_SPEED         800     // Maximum speed

#define INVALID          -999
#define BREAK            -999 //for physics plugin

#define NUM_ROBOTS 5 // Change this also in the supervisor!


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 500   // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
    PERFORMING_TASK = 5,
} robot_state_t;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS           8
#define BIAS_SPEED           400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};

// The state variables
int clock;
int clock_task;
char task_in_progress = 0;
uint16_t robot_id;          // Unique robot ID
TaskType robot_type;        // robot type
robot_state_t state;        // State of the robot
double my_pos[3];           // X, Z, Theta of this robot
char target_valid;          // boolean; whether we are supposed to go to the target
double target[99][4];       // x and z coordinates of target position (max 99 targets)
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;


// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}

void limit(int *number, int limit) {
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}

void markAsDone(int event_id){
    //find target list length
    int i = 0, target_list_length = 0;
    while(target[i][2] != INVALID){ i++;}
    target_list_length = i;  
        
    // If event is done, delete it from array 
    for(i=0; i<=target_list_length; i++)
    {
        if((int)target[i][2] == event_id) 
        { //look for correct id (in case wrong event was done first)
            for(; i<=target_list_length; i++)
            { //push list to the left from event index
                target[i][0] = target[i+1][0];
                target[i][1] = target[i+1][1];
                target[i][2] = target[i+1][2];
                target[i][3] = target[i+1][3];
            }
        }
        }
    // adjust target list length
    if(target_list_length-1 == 0) target_valid = 0; //used in general state machine 
    target_list_length = target_list_length-1;  
}
// Check if we received a message and extract information
static void receive_updates() 
{
    message_t msg;
    int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const message_t *pmsg = (const message_t *) wb_receiver_get_data(receiver_tag);
        
        // save a copy, cause wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(message_t));
        wb_receiver_next_packet(receiver_tag);

        // double check this message is for me
        // communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if(msg.robot_id != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d "  "doesn't match receiver %d\n", msg.robot_id, robot_id);
            //return;
            exit(1);
        }

        //find target list length
        i = 0;
        while(target[i][2] != INVALID){ i++;}
        target_list_length = i;  
        
        if(target_list_length == 0) target_valid = 0;   

        
        // Event state machine
        if(msg.event_state == MSG_EVENT_GPS_ONLY)
        {
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
            continue;
        }
        else if(msg.event_state == MSG_QUIT)
        {
            // Set speed
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            wb_robot_step(TIME_STEP);
            exit(0);
        }
        else if(msg.event_state == MSG_EVENT_REACHED && !task_in_progress)
        {
            printf("Robot %d reached task %d, first target %d\n", robot_id, msg.event_id, int(target[0][2]));
    
            if((int)target[0][2] != msg.event_id)
            {
                const message_event_status_t my_task = {robot_id, msg.event_id, MSG_EVENT_NOT_IN_PROGRESS};
                wb_emitter_set_channel(emitter_tag, robot_id+1);
                wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t)); 
            } 
            else {
                task_in_progress = 1;
                clock_task = clock;
            }



            // double temp[4];
            // if(int(target[0][2]) != msg.event_id){
            //     temp[0] = target[0][0];
            //     temp[1] = target[0][1];
            //     temp[2] = target[0][2];
            //     temp[3] = target[0][3];

            //     for(i=0; i<target_list_length; i++)
            //     {
            //         if((int)target[i][2] == msg.event_id) 
            //         { //look for correct id (in case wrong event was done first)
            //          target[0][0] = target[i][0];
            //          target[0][1] = target[i][1];
            //          target[0][2] = target[i][2];
            //          target[0][3] = target[i][3];     

            //          target[i][0] = temp[0];
            //          target[i][1] = temp[1];
            //          target[i][2] = temp[2];
            //          target[i][3] = temp[3];       
            //         }
            //     }
            // }
            printf("Robot %d reached task %d, first target %d\n", robot_id, msg.event_id, int(target[0][2]));

        }
        else if(msg.event_state == MSG_EVENT_DONE)
         {
             // If event is done, delete it from array 
             for(i=0; i<=target_list_length; i++)
             {
                 if((int)target[i][2] == msg.event_id) 
                 { //look for correct id (in case wrong event was done first)
                     for(; i<=target_list_length; i++)
                     { //push list to the left from event index
                         target[i][0] = target[i+1][0];
                         target[i][1] = target[i+1][1];
                         target[i][2] = target[i+1][2];
                         target[i][3] = target[i+1][3];
                     }
                 }
             }
             // adjust target list length
             if(target_list_length-1 == 0) target_valid = 0; //used in general state machine 
             target_list_length = target_list_length-1;    
         }
        else if(msg.event_state == MSG_EVENT_WON)
        {
            // insert event at index
            // for(i=target_list_length; i>=msg.event_index; i--)
            // {
            //     target[i+1][0] = target[i][0];
            //     target[i+1][1] = target[i][1];
            //     target[i+1][2] = target[i][2];
            //     target[i+1][3] = target[i][3];
            // }
            // target[msg.event_index][0] = msg.event_x;
            // target[msg.event_index][1] = msg.event_y;
            // target[msg.event_index][2] = msg.event_id;
            // target[msg.event_index][3] = int(msg.event_type);
            target[target_list_length][0] = msg.event_x;
            target[target_list_length][1] = msg.event_y;
            target[target_list_length][2] = msg.event_id;
            target[target_list_length][3] = int(msg.event_type);
            target_valid = 1; //used in general state machine
            target_list_length = target_list_length+1;
        }
        // check if new event is being auctioned
        else if(msg.event_state == MSG_EVENT_NEW)
        {                
                //*********         INSERT YOUR TACTIC BELOW         *********//
                //                                                            //
                // Determine your bid "d" and the index "indx" at             //
                // which you want to insert the event into the target list.   //
                // Available variables:                                       //
                // my_pos[0], my_pos[1]: Current x and y position of e-puck   //
                // msg.event_x, msg.event_z: Event x and y position           //
                // target[n][0], target[n][1]: x and y pos of target n in your//
                //                             target list                    //
                // target_list_length: current length of your target list     //
                //                                                            //
                // You can use dist(ax, ay, bx, by) to determine the distance //
                // between points a and b.                                    //
                ////////////////////////////////////////////////////////////////

            ///*** SIMPLE TACTIC ***///
            indx = target_list_length;
            // double d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
            ///*** END SIMPLE TACTIC ***///
                

            ///*** BETTER TACTIC ***///
            // Place your code here for I17 
            //*indx = target_list_length;
            
            double d = 0;
            // printf("time: %f\n", get_task_time(robot_type, msg.event_type));
            if(target_list_length > 0){
              d = dist(target[indx - 1][0], target[indx - 1][1], msg.event_x, msg.event_y)/0.1 + get_task_time(robot_type, msg.event_type);
            }else{
              d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y)/0.1 + get_task_time(robot_type, msg.event_type); 
      	 }
      	 
            ///*** END BETTER TACTIC ***///
                
                
            ///*** BEST TACTIC ***/// 
		    // Place your code here for I20
            //indx = 0;
            //double d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_z);             
            //if(target_list_length > 0)
            //{  
                // for all the tasks inside the task list (i.e. target[i] where i goes up to target_list_length)  check if putting the current 
                // event (located at (msg.event_x, msg.event_z)) in between two task results in  a smaller distance, and modify the d accordingly.
            //}

            ///*** END BEST TACTIC ***///
                
            // Send my bid to the supervisor
            // printf("Robot %d %c bid %f on event %d %c\n", robot_id, strType(robot_type), d, msg.event_id, strType(msg.event_type));
            const bid_t my_bid = {robot_id, msg.event_id, d, indx};
            wb_emitter_set_channel(emitter_tag, robot_id+1);
            wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));            
        }

    }
    
    
    // Communication with physics plugin (channel 0)            
    i = 0; k = 1;
    while((int)target[i][2] != INVALID){i++;}
    target_list_length = i; 
    if(target_list_length > 0)
    {        
        // Line from my position to first target
        wb_emitter_set_channel(emitter_tag,0);         
        buff[0] = BREAK; // draw new line
        buff[1] = my_pos[0]; 
        buff[2] = my_pos[1];
        buff[3] = target[0][0];
        buff[4] = target[0][1];
        // Lines between targets
        for(i=5;i<5*target_list_length-1;i=i+5)
        {
            buff[i] = BREAK;
            buff[i+1] = buff[i-2]; 
            buff[i+2] = buff[i-1];
            buff[i+3] = target[k][0]; 
            buff[i+4] = target[k][1];
            k++;  
        }
        // send, reset channel        
        if(target[0][2] == INVALID){ buff[0] = my_pos[0]; buff[1] = my_pos[1];}
        wb_emitter_send(emitter_tag, &buff, (5*target_list_length)*sizeof(float));
        wb_emitter_set_channel(emitter_tag,robot_id+1);                     
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RESET and INIT (combined in function reset()) */
void reset(void) 
{
    wb_robot_init();
    int i;

  //get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
    char s[4] = "ps0";
    for(i=0; i<NB_SENSORS;i++) 
    {
        // the device name is specified in the world file
        ds[i]=wb_robot_get_device(s);      
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i],64);
    } 

    clock = 0;
    indx = 0;
    
    // Init target positions to "INVALID"
    for(i=0;i<99;i++){ 
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID; 
        target[i][3] = UNKNOWN;
    }

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name; 
    robot_name = (char*) wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck(%d)", &tmp_id)) {
        robot_id = (uint16_t)tmp_id;        
        robot_type = getType(robot_id);
        } 
    else {fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name); exit(1);}

    printf("Robot %s of type %c has been correctly initialized\n", robot_name, strType(robot_type));

    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id+1);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);
    
    // Seed random generator
    srand(getpid());
    

    // Reset stats
    stat_max_velocity = 0.0;
}


void update_state(int _sum_distances)
{   
    if (_sum_distances > STATECHANGE_DIST && state == GO_TO_GOAL)
    {
        state = OBSTACLE_AVOID;
        // printf("Robot %d obstacle avoid _sum_distances: %d\n", robot_id, _sum_distances);
    }
    else if(target_valid && task_in_progress && state != PERFORMING_TASK)
    {
        state = PERFORMING_TASK;
    }
    else if (target_valid && state != PERFORMING_TASK)
    {
        state = GO_TO_GOAL;
    }
    else if(state != PERFORMING_TASK)
    {
        state = DEFAULT_STATE;
    }
    else if(state == PERFORMING_TASK && (clock - clock_task) >= (get_task_time(robot_type, TaskType(target[0][3]))*1000)){
        state = OBSTACLE_AVOID;
        task_in_progress = 0;
        // printf("Time to finish task: %f \n", (get_task_time(robot_type, TaskType(target[0][3]))*1000));
        // printf("Robot %d completed task %c\n after %d time \n", robot_id, strType(TaskType(target[0][3])), (clock - clock_task));

        const message_event_status_t my_task = {robot_id, uint16_t(int(target[0][2])), MSG_EVENT_DONE};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t));  

        // markAsDone(target[0][2]);
    }
}

// Odometry
void update_self_motion(int msl, int msr) {
    double theta = my_pos[2];
  
    // Compute deltas of the robot
    double dr = (double)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = (double)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl)/2.0;
    double dtheta = (dr - dl)/AXLE_LENGTH;
  
    // Compute deltas in the environment
    double dx = du * cosf(theta);
    double dy = du * sinf(theta);
  
    // Update position
    my_pos[0] += dx;
    my_pos[1] -= dy;
    my_pos[2] -= dtheta;
    
    // Keep orientation within 0, 2pi
    if (my_pos[2] > 2*M_PI) my_pos[2] -= 2.0*M_PI;
    if (my_pos[2] < 0) my_pos[2] += 2.0*M_PI;

    // Keep track of highest velocity for modelling
    double velocity = du * 1000.0 / (double) TIME_STEP;
    if (state == GO_TO_GOAL && velocity > stat_max_velocity)
        stat_max_velocity = velocity;
}


// Compute wheel speed to avoid obstacles
void compute_avoid_obstacle(int *msl, int *msr, int distances[]) 
{
    int d1=0,d2=0;       // motor speed 1 and 2     
    int sensor_nb;       // FOR-loop counters    

    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {   
       d1 += (distances[sensor_nb]-300) * Interconn[sensor_nb];
       d2 += (distances[sensor_nb]-300) * Interconn[sensor_nb + NB_SENSORS];
    }
    d1 /= 80; d2 /= 80;  // Normalizing speeds

    *msr = d1+BIAS_SPEED; 
    *msl = d2+BIAS_SPEED; 
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

// Computes wheel speed to go towards a goal
void compute_go_to_goal(int *msl, int *msr) 
{
    // // Compute vector to goal
    float a = target[0][0] - my_pos[0];
    float b = target[0][1] - my_pos[1];
    // Compute wanted position from event position and current location
    float x =  a*cosf(my_pos[2]) - b*sinf(my_pos[2]); // x in robot coordinates
    float y =  a*sinf(my_pos[2]) + b*cosf(my_pos[2]); // y in robot coordinates

    float Ku = 0.2;   // Forward control coefficient
    float Kw = 10.0;  // Rotational control coefficient
    float range = 1; //sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position
    
    // Compute forward control
    float u = Ku*range*cosf(bearing);
    // Compute rotational control
    float w = Kw*range*sinf(bearing);
    
    // Convert to wheel speeds!
    *msl = 50*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    *msr = 50*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

// RUN e-puck
void run(int ms)
{
    float msl_w, msr_w;
    // Motor speed and sensor variables	
    int msl=0,msr=0;                // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    int sum_distances=0;        // sum of all distance sensor inputs, used as threshold for state change.  	

    // Other variables
    int sensor_nb;

    // Add the weighted sensors values
    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {  
        distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
        sum_distances += distances[sensor_nb];
    }

    // Get info from supervisor
    receive_updates();

    // State may change because of obstacles
    update_state(sum_distances);
    
    if(target_valid){
        
        const message_event_status_t my_task = {robot_id, uint16_t(target[0][2]), MSG_EVENT_IN_PROGRESS};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t)); 
    }

    // printf("Robot %d state %d valid target %d sum distances %d\n", robot_id, int(state), target_valid, sum_distances);
    // printf("State: %c\n", strType(state));
    // Set wheel speeds depending on state
    switch (state) {
        case STAY:
            msl = 0;
            msr = 0;
            break;

        case GO_TO_GOAL:
            compute_go_to_goal(&msl, &msr);
            break;

        case OBSTACLE_AVOID:
            compute_avoid_obstacle(&msl, &msr, distances);
            break;

        case RANDOM_WALK:
            msl = 400;
            msr = 400;
            break;

        case PERFORMING_TASK:
            msl = 0;
            msr = 0;
            break;


        default:
            printf("Invalid state: robot_id %d \n", robot_id);
    }
    // Set the speed
    msl_w = msl*MAX_SPEED_WEB/1000;
    msr_w = msr*MAX_SPEED_WEB/1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    update_self_motion(msl, msr);

    // Update clock
    clock += ms;
}

// MAIN
int main(int argc, char **argv) 
{
    reset();
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {run(TIME_STEP);}
    wb_robot_cleanup();

    return 0;
}
