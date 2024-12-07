#include "include/e-puck-lib.hpp"
#include "../supervisor/include/taskType.hpp"

EpuckDistributed::EpuckDistributed() : Epuck(){
    std::fill(std::begin(x), std::end(x), -1);
    std::fill(std::begin(y_bids), std::end(y_bids), 0);
    std::fill(std::begin(y_winners), std::end(y_winners), -1);
    std::fill(std::begin(h), std::end(h), 0);
    assigned_task = -1;
}

void EpuckDistributed::reset(){
    // printf("Epuck Reset in distributed child\n");
    Epuck::reset();

    wb_emitter_set_range(emitter_tag, EMITTER_RANGE);


    emitter_tag_sup = wb_robot_get_device("emitter_inf");
    wb_emitter_set_channel(emitter_tag_sup, SUP_REC_BASE_CHANNEL + robot_id + 1);
    wb_emitter_set_range(emitter_tag_sup, -1);


    printf("DistEpuck Reset completed\n");

}

void EpuckDistributed::msgEventDone(message_t msg){
    
}

void EpuckDistributed::msgEventWon(message_t msg){
    
}

void EpuckDistributed::msgEventNew(message_t msg){
    /*
        update x vector to update task as announced
        update tasks info in t
    */
   
    // printf("R%d: new task with index %d received\n", robot_id, msg.event_index);
    std::cout << "R" << robot_id << ": task ind" <<  msg.event_index << " type" << strType(msg.event_type) << " pos:[" << msg.event_x << ":" <<  msg.event_y << "] received" << std::endl;

    // update task in t
    t[msg.event_index].id = msg.event_id;
    t[msg.event_index].index = msg.event_index;
    t[msg.event_index].type = msg.event_type;
    t[msg.event_index].posX = msg.event_x;
    t[msg.event_index].posY = msg.event_y;

    // set task as available in x
    x[msg.event_index] = 0;

    // reset also your market
    y_bids[msg.event_index] = 0;
    y_winners[msg.event_index] = -1;

    if(msg.event_index == assigned_task){
        assigned_task = -1;
        // if the new task has the same index of the assigned task, we need to
        // set the robot to unassign state
    }

           
}

void EpuckDistributed::msgEventCustom(message_t msg){

    if(msg.event_state == MSG_DISTRIBUTED_MARKET)
    {
        // PHASE 2.1
        uint16_t neighbor = msg.sender_id;
        float* neighbor_market_bids = msg.market_bids;
        int16_t* neighbor_market_winners = msg.market_winners;

        // printf("R%d: new market received from R%d\n", robot_id, neighbor);

        // printf("R%d: new market received from R%d: [", robot_id, neighbor);
        // for(int i=0; i<NUM_TASKS;++i){
        //     std::cout << i << ": ";
        //     printf("R%dB%.0f] - [", neighbor_market_winners[i], neighbor_market_bids[i]);
        // }    
        // std::cout << std::endl;

        for (int i = 0; i < NUM_TASKS; ++i) {
            // update my market
            if(! is_my_bid_better(y_bids[i], neighbor_market_bids[i] )){
                y_bids[i] = neighbor_market_bids[i];
                y_winners[i] = neighbor_market_winners[i];
            }
        }
    }

}

void EpuckDistributed::update_state_custom(){
    if(state == TASK_COMPLETED)
    {
        state = DEFAULT_STATE;
        task_in_progress = 0;

        // notify supervisor
        message_event_status_t my_task = {robot_id, uint16_t(target[0][2]), assigned_task, MSG_EVENT_DONE};
        int res = wb_emitter_send(emitter_tag_sup, &my_task, sizeof(message_event_status_t)); 
        printf("R%d: task ID%d completed, Sending on channel %d(%s)\n", robot_id, uint16_t(target[0][2]), wb_emitter_get_channel(emitter_tag_sup), res?"OK":"FAILED");
        
        x[assigned_task] = -1;
        assigned_task = -1;


    }

}

void EpuckDistributed::run_custom_pre_update(){

    // PHASE 2.2
    // drop assigned task if necessary
    if(is_assigned() && !is_my_bid_better(x[assigned_task], y_bids[assigned_task]) ){
        printf("R%d: checking if I need to drop bid %d\n", robot_id, assigned_task);
        bool condNeighborBetterBid = (! is_my_bid_better(x[assigned_task], y_bids[assigned_task]));
        bool condSameBid = ( x[assigned_task] == y_bids[assigned_task] );
        bool condNeighborBetterID = (y_winners[assigned_task] < robot_id);  // if tie, task assigned to robot with lower ID
        if( condNeighborBetterBid || (condSameBid && condNeighborBetterID)){
            printf("R%d: dropping bid %d because mine is %.2f while market is %.2f - ", robot_id, assigned_task, x[assigned_task], y_bids[assigned_task]);
            printf("conds %d %d %d\n", robot_id, condNeighborBetterBid, condSameBid, condNeighborBetterID);
            // drop bid
            x[assigned_task] = 0;
            assigned_task = -1;
        }
    }

    // PHASE 1
    if(! is_assigned()){
        // robot doesn't have a task
        // printf("R%d: assigning task\n", robot_id);
        std::fill(std::begin(h), std::end(h), 0);
        for(int i=0; i<NUM_TASKS; ++i)
        {
            if(x[i] == -1)
                continue;  // task not yet announced
            
            float b = compute_bid(t[i]);
            h[i] = (is_my_bid_better(b, y_bids[i])) ? b : 0;
        }


        // extract max valid bid
        auto max_bid = std::max_element(std::begin(h), std::end(h));
        int max_index = std::distance(std::begin(h), max_bid);

        if( *max_bid > 0 ){  // f there is at least one valid task
            // auto-assign bid
            x[max_index] = *max_bid;
            y_bids[max_index] = *max_bid;
            y_winners[max_index] = robot_id;
            assigned_task = max_index;

            target[0][0] = t[assigned_task].posX;
            target[0][1] = t[assigned_task].posY;
            target[0][2] = t[assigned_task].id;
            target[0][3] = int(t[assigned_task].type);
            target_valid = 1; //used in general state machine
            target_list_length = target_list_length+1;

            printf("R%d: task assigned %d with bid %.2f\n", robot_id, assigned_task, x[max_index]);
        }
    }
}

void EpuckDistributed::run_custom_post_update(){

    /*
    std::cout << "R" << robot_id << " bid: ";
    for(float i : x){
        std::cout << i << " ";
    }    
    std::cout << std::endl;
    std::cout << "R" << robot_id << " market: ";
    for(float i : y){
        std::cout << i << " ";
    }    
    std::cout << std::endl;
    */
    // re-initialize adjacency matrix for next iteration
    // std::fill(std::begin(G), std::end(G), 0);

    // broadcast new version of my market
    message_t msg;
    msg.robot_id = -1; // indended receiver - broadcast
    msg.sender_id = robot_id;
    msg.event_state = MSG_DISTRIBUTED_MARKET;
    memcpy(msg.market_bids, y_bids, sizeof(y_bids));
    memcpy(msg.market_winners, y_winners, sizeof(y_winners));
    for(int i = 1; i < NUM_ROBOTS+1; ++i)
    {
        // iterate over all channels to share market
        if(i == robot_id+1) continue;
        wb_emitter_set_channel(emitter_tag, i);
        wb_emitter_send(emitter_tag, &msg, sizeof(message_t));
    }



    //check if task has been reached
    if( !task_in_progress && assigned_task != -1 && check_if_event_reached() == true)
    {
        // printf("R%d: reached task %d\n", robot_id, assigned_task);
        task_in_progress = 1;
        clock_task = clock;
    }

}

float EpuckDistributed::compare_bids(float bid1, float bid2){
    // return best bid
    return std::max(bid1, bid2);
}

bool EpuckDistributed::is_my_bid_better(float myBid, float otherBid){

    return (myBid == compare_bids(myBid, otherBid)) ? true : false;
}

float EpuckDistributed::compute_bid(task_t task){
    // printf("R%d: my pos %.2f %.2f - task pos: %.2f %.2f\n", robot_id, my_pos[0], my_pos[1], task.posX, task.posY);
    float d = dist(task.posX, task.posY, my_pos[0], my_pos[1]);
    float t = get_task_time(robot_type, task.type);

    float bid = 1 / (d/0.5 + t) * 1000; // 0.5 is epuck max velocity
    // printf("R%d: bid %.2f\n", robot_id, bid);

    return bid; 
}

bool EpuckDistributed::is_assigned(){
    if(assigned_task == -1)
        return false;
    else
        return true;
}