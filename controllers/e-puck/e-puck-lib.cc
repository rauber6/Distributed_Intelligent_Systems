#include "include/e-puck-lib.hpp"
#include "../supervisor/include/taskType.hpp"
#include <iostream>

int Interconn[16] = {17, 29, 34, 10, 8, -38, -56, -76, -72, -58, -36, 8, 10, 36, 28, 18};

/* e-puck class implementation */

Epuck::Epuck() {
    task_in_progress = 0;
    target_valid = 0;
    target_list_length = 0;
};

// Epuck::~Epuck(){

// };

void Epuck::reset()
{
    wb_robot_init();
    int i;

    // get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    char s[4] = "ps0";
    for (i = 0; i < NB_SENSORS; i++)
    {
        // the device name is specified in the world file
        ds[i] = wb_robot_get_device(s);
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i], 64);
    }

    clock = 0;
    indx = 0;

    // Init target positions to "INVALID"
    for (i = 0; i < 99; i++)
    {
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID;
        target[i][3] = UNKNOWN;
    }

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char *robot_name;
    robot_name = (char *)wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck(%d)", &tmp_id))
    {
        robot_id = (uint16_t)tmp_id;
        robot_type = getType(robot_id);
    }
    else
    {
        fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name);
        exit(1);
    }

    printf("Robot %s of type %c has been correctly initialized\n", robot_name, strType(robot_type));

    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id + 1);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id + 1);

    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}

void Epuck::update_state(int _sum_distances)
{   
    if (_sum_distances > STATECHANGE_DIST && state == GO_TO_GOAL)
    {
        state = OBSTACLE_AVOID;
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

        const message_event_status_t my_task = {robot_id, uint16_t(target[0][2]), MSG_EVENT_DONE};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t));  
    }

    update_state_custom();

}

void Epuck::update_self_motion(int msl, int msr) {
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

void Epuck::compute_avoid_obstacle(int *msl, int *msr, int distances[]) 
{
    int d1=0,d2=0;       // motor speed 1 and 2     
    int sensor_nb;       // FOR-loop counters    

    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {   
       d1 += (distances[sensor_nb]-300) * Interconn[sensor_nb];
       d2 += (distances[sensor_nb]-300) * Interconn[sensor_nb + NB_SENSORS];
    }
    d1 /= 8; d2 /= 8;  // Normalizing speeds

    *msr = d1+BIAS_SPEED; 
    *msl = d2+BIAS_SPEED; 
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

void Epuck::compute_go_to_goal(int *msl, int *msr) 
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

void Epuck::run(int ms)
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

    run_custom_pre_update();

    // State may change because of obstacles
    update_state(sum_distances);

    // Custom instruction
    run_custom_post_update();

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

        case RANDOM_WALK:  // FIXME can it be safely removed? seems not te used
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

// Check if we received a message and extract information
void Epuck::receive_updates() 
{
    message_t msg;
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
        // i = 0;
        // while(target[i][2] != INVALID){ i++;}
        // target_list_length = i;  
        
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

        }
        else if(msg.event_state == MSG_EVENT_DONE)
         {
            msgEventDone(msg);
         }
        else if(msg.event_state == MSG_EVENT_WON)
        {
            msgEventWon(msg);
        }
        // check if new event is being auctioned
        else if(msg.event_state == MSG_EVENT_NEW)
        {     
            msgEventNew(msg);
        }
    }

    msgEventCustom(msg);


    // Communication with physics plugin (channel 0)            
    i = 0; k = 1;
    // while((int)target[i][2] != INVALID){i++;}
    // target_list_length = i; 
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