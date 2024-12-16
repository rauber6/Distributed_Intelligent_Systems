#include "include/e-puck-lib.hpp"
#include "../supervisor/include/taskType.hpp"

EpuckDistributed::EpuckDistributed() : Epuck(){
    std::fill(std::begin(x), std::end(x), -1);
    std::fill(std::begin(y_bids), std::end(y_bids), -1);
    std::fill(std::begin(y_winners), std::end(y_winners), -1);
    std::fill(std::begin(h), std::end(h), 0);
    assigned_task = -1;
    newly_received_task = -1;

    start_received_task = 0;
    can_start_broadcatsting = false;
}

void EpuckDistributed::reset(){
    Epuck::reset();
    wb_emitter_set_range(emitter_tag, EMITTER_RANGE);

    // activate emitter for SV
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

    // update task in t
    t[msg.event_index].id = msg.event_id;
    t[msg.event_index].index = msg.event_index;
    t[msg.event_index].type = msg.event_type;
    t[msg.event_index].posX = msg.event_x;
    t[msg.event_index].posY = msg.event_y;

    newly_received_task = msg.event_index; //store in this timestamp we are updating this task
    
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

    start_received_task++;
    if (start_received_task == NUM_TASKS)
        can_start_broadcatsting = true;

    #if DEBUG
        printf("\033[32m");
        printf("[%d]R%d: new task (id%d) with index %d received\n", clock, robot_id, msg.event_id, msg.event_index);
        // === print x ===
        printf("[%d]\tR%d: x: [",clock, robot_id);
        for(int i=0; i<NUM_TASKS;++i){
            printf("%d: ", i);
            printf("%d] - [", x[i]);
        }    
        printf("\n");
        // ====================
        // === print market ===
        printf("[%d]\tR%d: my market: [",clock, robot_id);
        for(int i=0; i<NUM_TASKS;++i){
            printf("%d: ", i);
            printf("R%dB%d] - [", y_winners[i], y_bids[i]);
        }    
        printf("\n");
        // ====================
        printf("\033[0m");
    #endif

          
}

void EpuckDistributed::msgEventCustom(message_t msg){

    if(msg.event_state == MSG_DISTRIBUTED_MARKET)
    {
        // PHASE 2.1
        int* neighbor_market_bids = msg.market_bids;
        int* neighbor_market_winners = msg.market_winners;

        // === print market ===
        #if DEBUG
            printf("\033[32m");
            printf("[%d]R%d: neighbor %d is sharing its market \n", clock, robot_id, msg.sender_id);
            printf("[%d]\tR%d: neigbors R%d market: [",clock, robot_id, msg.sender_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("R%dB%d] - [", neighbor_market_winners[i], neighbor_market_bids[i]);
            }    
            printf("\n");
            printf("\033[0m");
        #endif
        // ====================

        for (int i = 0; i < NUM_TASKS; ++i) {
            if(i == newly_received_task){
                // here it means we may update task with neighbor knowledge, but this knowledge is outdated
                #if DEBUG
                    printf("\033[32m");
                    printf("[%d]R%d: neighbor %d is telling me sth about %d but I don't update because I just got it from SV \n", clock, robot_id, msg.sender_id, i);
                    printf("\033[0m");
                #endif
                continue;
            }

            if(neighbor_market_bids[i] < 0)  // ==-2
            {
                // the current tash is being performed or has been completed, update my market accordingly
                if(y_bids[i] != -1){
                    y_bids[i] = neighbor_market_bids[i];  // never override a state if it is currently invalid (i.e. -1)
                }
                if(x[i] != -1){
                    x[i] = neighbor_market_bids[i];;  // never override a state if it is currently invalid (i.e. -1)
                }
            }
            // update my market if marker has better bid
            else if( is_my_bid_better(y_bids[i], neighbor_market_bids[i]) == 0){
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
        
        #if DEBUG
            int tmp = assigned_task;
        #endif

        // reset info about the completed task
        x[assigned_task] = -1;
        y_bids[assigned_task] = -1;
        y_winners[assigned_task] = -1;
        assigned_task = -1;

        #if DEBUG
            printf("\033[33m"); 
            printf("[%d]R%d: task %d(id%d) completed, Sending on channel %d(%s)\n", clock, robot_id, tmp, uint16_t(target[0][2]), wb_emitter_get_channel(emitter_tag_sup), res?"OK":"FAILED");
            // === print x ===
            printf("[%d]\tR%d: x: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("%d] - [", x[i]);
            }    
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my market: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("R%dB%d] - [", y_winners[i], y_bids[i]);
            }    
            printf("\n");
            // ====================
            printf("\033[0m");
        #endif
    }

}

void EpuckDistributed::run_custom_pre_update(){

    // don't perform this step of algorithm when performing tasks
    if(state == PERFORMING_TASK)
        return;

    // PHASE 2.2
    // drop assigned task if necessary
    if(is_assigned() && (y_bids[assigned_task] < 0)){ 
        // someone else is already executing my task, drop it
        #if DEBUG
        printf("[%d]R%d: dropping task %d because someone made it invalid (%d)\n", clock, robot_id, assigned_task, y_bids[assigned_task]); 
        #endif
        x[assigned_task] = y_bids[assigned_task];
        assigned_task = -1;
    }
    else if(is_assigned() && (is_my_bid_better(x[assigned_task], y_bids[assigned_task])==0) && (y_winners[assigned_task] != robot_id) ){
        bool condNeighborBetterBid = ( is_my_bid_better(x[assigned_task], y_bids[assigned_task]) == 0);
        bool condSameBid = ( x[assigned_task] == y_bids[assigned_task] );
        bool condNeighborBetterID = (y_winners[assigned_task] < robot_id);  // if tie, task assigned to robot with lower ID
        if( condNeighborBetterBid || (condSameBid && condNeighborBetterID)){
            #if DEBUG
                printf("[%d]R%d: dropping task %d because my bid is %d while market is %d - ",clock, robot_id, assigned_task, x[assigned_task], y_bids[assigned_task]);
                printf("conds %d %d %d\n", condNeighborBetterBid, condSameBid, condNeighborBetterID);
            #endif
            // drop bid
            x[assigned_task] = 0;
            assigned_task = -1;
        }
    }

    // PHASE 1
    if(! is_assigned()){
        // robot doesn't have a task
        #if DEBUG
            printf("\033[36m");
            printf("[%d]R%d: assigning task\n", clock, robot_id);
            // === print x ===
            printf("[%d]\tR%d: x: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("%d] - [", x[i]);
            }    
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my market: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("R%dB%d] - [", y_winners[i], y_bids[i]);
            }    
            printf("\n");
            // ====================
            printf("\033[0m");
        #endif


        std::fill(std::begin(h), std::end(h), 0);
        for(int i=0; i<NUM_TASKS; ++i)
        {
            if(x[i] < 0)
                continue;  // task not valid
            
            int b = compute_bid(t[i]);
            int res = is_my_bid_better(b, y_bids[i]);
            if(res == 1){
                h[i] = b; // my bid is better than market
            } else if (res == 0){
                h[i] = 0; // bid on market is better than mine
            } else{
                #if DEBUG
                    printf("[%d]R%d: Error with bids %d - x=%d, y_b=%d, y_w=%d\n", clock, robot_id, i, x[i], y_bids[i], y_winners[i]);
                #endif
                h[i] = 0;
            }
            if( (x[i] < 0) || (h[i] < 0) ){  // sanity check
                printf("\033[31mError bid < 0\033[0m\n");
                exit(1);
            }
        }


        // extract max valid bid
        auto max_bid = std::max_element(std::begin(h), std::end(h));
        int max_index = std::distance(std::begin(h), max_bid);

        #if DEBUG
            printf("\033[36m");
        #endif
        if( *max_bid > 0 ){  // if there is at least one valid task
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

            #if DEBUG
                printf("[%d]R%d: task assigned %d(id%d) with bid %d - market says [R%d:B%d]\n", clock, robot_id, assigned_task, int(target[0][2]), x[assigned_task], y_winners[assigned_task], y_bids[assigned_task] );
            #endif
        } else{
            #if DEBUG
                printf("[%d]R%d: didn't find a task\n", clock, robot_id);
            #endif
        }
        #if DEBUG
            // === print x ===
            printf("[%d]\tR%d: x: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("%d] - [", x[i]);
            }    
            printf("\n");
            // ====================
            // === print h ===
            printf("[%d]\tR%d: h: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("%d] - [", h[i]);
            }    
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my market: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("R%dB%d] - [", y_winners[i], y_bids[i]);
            }    
            printf("\n");
            // ====================
            printf("\033[0m");
        #endif
    }
}

void EpuckDistributed::run_custom_post_update(){
    
    //check if task has been reached
    if( !task_in_progress && assigned_task != -1 && check_if_event_reached() == true)
    {
        task_in_progress = 1;
        clock_task = clock;
        x[assigned_task] = -2;
        y_bids[assigned_task] = -2;
        // y_winners[assigned_task] = -2;
        state = PERFORMING_TASK;

        #if DEBUG
            printf("\033[35m");
            printf("[%d]R%d: reached task %d\n", clock, robot_id, assigned_task);
            // === print x ===
            printf("[%d]\tR%d: x: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("%d] - [", x[i]);
            }    
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my market: [",clock, robot_id);
            for(int i=0; i<NUM_TASKS;++i){
                printf("%d: ", i);
                printf("R%dB%d] - [", y_winners[i], y_bids[i]);
            }    
            printf("\n");
            // ====================
            printf("\033[0m");
        #endif
    }


    // broadcast new version of my market
    if(can_start_broadcatsting)
    {
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
    }

    // reset newly_received_task
    newly_received_task = -1;

}

int EpuckDistributed::compare_bids(int bid1, int bid2){
    // return best bid
    return std::max(bid1, bid2);
}

int EpuckDistributed::is_my_bid_better(int myBid, int otherBid){

    /*
        return 1: my bid is better
        return 0: other bid is better
        return -1: invalid bids
    */

    if((myBid < 0) || (otherBid < 0))
        return -1;

    return (myBid == compare_bids(myBid, otherBid)) ? 1 : 0;
}

int EpuckDistributed::compute_bid(task_t task){
    float d = dist(task.posX, task.posY, my_pos[0], my_pos[1]);
    float t = get_task_time(robot_type, task.type);

    int bid = std::floor( 1 / (d/0.5 + t) * 1000 ); // 0.5 is epuck max velocity

    if(bid < 0){
        printf("\033[31mError bid < 0\033[0m\n");
        exit(1);
    }

    return bid; 
}

bool EpuckDistributed::is_assigned(){
    if(assigned_task == -1)
        return false;
    else
        return true;
}