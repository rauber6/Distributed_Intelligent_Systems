#include "include/e-puck-lib.hpp"
#include "../supervisor/include/taskType.hpp"

EpuckCentralized::EpuckCentralized(int plan_length, bool task_timeout_off) : Epuck(){

    EpuckCentralized::plan_length = plan_length;
    EpuckCentralized::task_timeout_off = task_timeout_off;

    printf("plan length: %d\n", EpuckCentralized::plan_length);
}

void EpuckCentralized::msgEventDone(message_t msg){
    // If event is done, delete it from array 
    for(int i=0; i< target_list_length; i++)
    {
        if((int)target[i][2] == msg.event_id) 
        { 
        //look for correct id (in case wrong event was done first)
            for(; i < target_list_length; i++)
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

void EpuckCentralized::msgEventWon(message_t msg){
    // insert event at index
    if(msg.event_index <= target_list_length){
        for(int i=target_list_length; i>=msg.event_index; i--)
        {
            target[i+1][0] = target[i][0];
            target[i+1][1] = target[i][1];
            target[i+1][2] = target[i][2];
            target[i+1][3] = target[i][3];
        }
        target[msg.event_index][0] = msg.event_x;
        target[msg.event_index][1] = msg.event_y;
        target[msg.event_index][2] = msg.event_id;
        target[msg.event_index][3] = int(msg.event_type);
    }
    else{
        target[target_list_length][0] = msg.event_x;
        target[target_list_length][1] = msg.event_y;
        target[target_list_length][2] = msg.event_id;
        target[target_list_length][3] = int(msg.event_type);
    }
    target_valid = 1; //used in general state machine

    target_list_length = target_list_length+1;
}

void EpuckCentralized::msgEventNew(message_t msg){

    if(target_list_length < plan_length){    

        indx = target_list_length;            
            
        indx = 0;
        double inserted_d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y)+ dist(target[0][0], target[0][1], msg.event_x, msg.event_y);
        double prev_d = dist(target[0][0], target[0][1], my_pos[0], my_pos[1]);           
        double d = inserted_d - prev_d;

        double* time_to = new double[target_list_length]();
        double* distance_to = new double[target_list_length]();

        // Make sure to initialize the arrays if needed
        for (int i = 0; i < target_list_length; ++i) {
            time_to[i] = 0;
            distance_to[i] = 0;
        }

        if(target_list_length > 0)
        {  
            indx = 1;
            // for all the tasks inside the task list (i.e. target[i] where i goes up to target_list_length)  check if putting the current 
            // event (located at (msg.event_x, msg.event_z)) in between two task results in  a smaller distance, and modify the d accordingly.
            for(int i = 1; i < target_list_length; i++){
            inserted_d = dist(target[i - 1][0], target[i - 1][1], msg.event_x, msg.event_y) + dist(target[i][0], target[i][1], msg.event_x, msg.event_y);
            prev_d = dist(target[i - 1][0], target[i - 1][1], target[i][0], target[i][1]);
            if( inserted_d - prev_d < d) {
                d = inserted_d - prev_d;
                indx = i;
            }
            time_to[i] = time_to[i-1] + get_task_time(robot_type, TaskType(target[i][3]));
            distance_to[i] = prev_d;
            }
        }
        d = (distance_to[indx] + d)/0.1 + get_task_time(robot_type, msg.event_type) + time_to[indx];
        // d = (d)/0.1 + get_task_time(robot_type, msg.event_type);

        ///*** END BEST TACTIC ***///
        // Send my bid to the supervisor
        // printf("Robot %d %c bid %f on event %d %c\n", robot_id, strType(robot_type), d, msg.event_id, strType(msg.event_type));
        const bid_t my_bid = {robot_id, msg.event_id, d, indx};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));  
    

        delete[] time_to;
        delete[] distance_to;    
    } 
    else if (task_timeout_off){
        const bid_t my_bid = {robot_id, msg.event_id, -1, -1};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));  
    }  
}

void EpuckCentralized::msgEventCustom(message_t msg){

    if(msg.event_state == MSG_EVENT_REACHED && !task_in_progress){
    
        if((int)target[0][2] != msg.event_id)
        {
            const message_event_status_t my_task = {robot_id, msg.event_id, 0, MSG_EVENT_NOT_IN_PROGRESS};
            wb_emitter_set_channel(emitter_tag, robot_id+1);
            wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t)); 
        } 
        else {
            task_in_progress = 1;
            clock_task = clock;
        }

    }
}

void EpuckCentralized::update_state_custom(){
    
    if(state == TASK_COMPLETED)
    {
        state = DEFAULT_STATE;
        task_in_progress = 0;


        const message_event_status_t my_task = {robot_id, uint16_t(target[0][2]), 0, MSG_EVENT_DONE};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t));  

    }
}

void EpuckCentralized::run_custom_pre_update(){

}

void EpuckCentralized::run_custom_post_update(){
    if(target_valid)
    {
        const message_event_status_t my_task = {robot_id, uint16_t(target[0][2]), 0, MSG_EVENT_IN_PROGRESS};
        wb_emitter_set_channel(emitter_tag, robot_id+1);
        wb_emitter_send(emitter_tag, &my_task, sizeof(message_event_status_t)); 
    }
}