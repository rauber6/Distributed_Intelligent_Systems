#include "include/e-puck-lib.hpp"
#include "../supervisor/include/taskType.hpp"

EpuckDistributedPlan::EpuckDistributedPlan() : Epuck()
{
    std::fill(std::begin(x), std::end(x), -1);
    std::fill(std::begin(y_bids), std::end(y_bids), -1);
    std::fill(std::begin(y_winners), std::end(y_winners), -1);
    std::fill(std::begin(h), std::end(h), 0);
    assigned_task = -1;
    newly_received_task = -1;

    start_received_task = 0;
    can_start_broadcatsting = false;
}

void EpuckDistributedPlan::reset()
{
    // printf("Epuck Reset in distributed child\n");
    Epuck::reset();
    wb_emitter_set_range(emitter_tag, EMITTER_RANGE);

    // activate emitter sor SV
    emitter_tag_sup = wb_robot_get_device("emitter_inf");
    wb_emitter_set_channel(emitter_tag_sup, SUP_REC_BASE_CHANNEL + robot_id + 1);
    wb_emitter_set_range(emitter_tag_sup, -1);

    printf("DistEpuck Reset completed\n");
}

void EpuckDistributedPlan::msgEventDone(message_t msg)
{
}

void EpuckDistributedPlan::msgEventWon(message_t msg)
{
}

void EpuckDistributedPlan::msgEventNew(message_t msg)
{
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

    newly_received_task = msg.event_index; // store in this timestamp we are updating this task

    // set task as available in x
    x[msg.event_index] = 0;

    // reset also your market
    y_bids[msg.event_index] = 0;
    y_winners[msg.event_index] = -1;

    // check if completed task is in the plan
    for ( int i = 0; i < b_length; ++i){
        if (msg.event_index == p[i])
        {
            assigned_task = -1;
            // if the new task has the same index of the assigned task, we need to
            // set the robot to unassign state

            // printf("Robot id%d b_length %d\n.", robot_id, b_length);
            if (b_length > 1)
            {
                for (int j = i; j < b_length - 1; ++j)
                {
                    p[j] = p[j + 1];
                    b[j] = b[j + 1];
                }
                assigned_task = p[0];
                target[0][0] = t[assigned_task].posX;
                target[0][1] = t[assigned_task].posY;
                target[0][2] = t[assigned_task].id; // FIXME assigning a uint16_t to double
                target[0][3] = int(t[assigned_task].type);
                state = GO_TO_GOAL;
                target_valid = 1;
                clock_goal = clock;
                target_list_length += 1;
                // printf("Robot id%d new assigned task %d at time %d and is in state %d.\n", robot_id, assigned_task, clock_goal, state);

            }
            b_length --;
            target_list_length -= 1;
            break;
        }
    }

    start_received_task++;
    if (start_received_task == NUM_TASKS)
        can_start_broadcatsting = true;

#if DEBUG
    printf("\033[32m");
    printf("[%d]R%d: new task (id%d) with index %d received\n", clock, robot_id, msg.event_id, msg.event_index);
    // === print x ===
    printf("[%d]\tR%d: x: [", clock, robot_id);
    for (int i = 0; i < NUM_TASKS; ++i)
    {
        printf("%d: ", i);
        printf("%d] - [", x[i]);
    }
    printf("\n");
    // ====================
    // === print market ===
    printf("[%d]\tR%d: my market: [", clock, robot_id);
    for (int i = 0; i < NUM_TASKS; ++i)
    {
        printf("%d: ", i);
        printf("R%dB%d] - [", y_winners[i], y_bids[i]);
    }
    printf("\n");
                // === print market ===
            printf("[%d]\tR%d: my p: [", clock, robot_id);
            for (int i = 0; i < b_length; ++i)
            {
                printf("%d: ", i);
                printf("%d] - [", p[i]);
            }
            printf("\n");
    // ====================
    printf("\033[0m");
#endif
}

void EpuckDistributedPlan::msgEventCustom(message_t msg)
{

    if (msg.event_state == MSG_DISTRIBUTED_MARKET)
    {
        // PHASE 2.1
        int *neighbor_market_bids = msg.market_bids;
        int *neighbor_market_winners = msg.market_winners;

// === print market ===
// FIXME to be removed
#if DEBUG
        printf("\033[32m");
        printf("[%d]R%d: neighbor %d is sharing its market \n", clock, robot_id, msg.sender_id);
        printf("[%d]\tR%d: neigbors R%d market: [", clock, robot_id, msg.sender_id);
        for (int i = 0; i < NUM_TASKS; ++i)
        {
            printf("%d: ", i);
            printf("R%dB%d] - [", neighbor_market_winners[i], neighbor_market_bids[i]);
        }
        printf("\n");
        printf("\033[0m");
#endif
        // ====================

        for (int i = 0; i < NUM_TASKS; ++i)
        {
            if (i == newly_received_task)
            {
// here it means we may update task with neighbor knoledge, but this knowledge is outdated
#if DEBUG
                printf("\033[32m");
                printf("[%d]R%d: neighbor %d is telling me sth about %d but I don't update because I just got it from SV \n", clock, robot_id, msg.sender_id, i);
                printf("\033[0m");
#endif
                continue;
            }
            // TODO what if market says -1?
            // I think it doesnt matter because robot will get soon new task from SV
            if (neighbor_market_bids[i] < 0) // ==-2
            {
                // the current tash is being performed or has been completed, update my market accordingly
                // x[i] = neighbor_market_bids[i]; // make task invalid
                // y_bids[i] = neighbor_market_bids[i];
                // y_winners[i] = -2;

                if (y_bids[i] != -1)
                {
                    y_bids[i] = neighbor_market_bids[i]; // never override a state if it is currently invalid (i.e. -1)
                }
                if (x[i] != -1)
                {
                    x[i] = neighbor_market_bids[i];
                    ; // never override a state if it is currently invalid (i.e. -1)
                }
            }
            // update my market if marker has better bid
            else if (is_my_bid_better(y_bids[i], neighbor_market_bids[i]) == 0)
            {
                y_bids[i] = neighbor_market_bids[i];
                y_winners[i] = neighbor_market_winners[i];
            }
        }
    }
}

void EpuckDistributedPlan::update_state_custom()
{
    if (state == TASK_COMPLETED)
    {
        // state = DEFAULT_STATE;
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
        
        // printf("Robot id%d b_length %d\n.", robot_id, b_length);
        if (b_length > 1)
        {
            for (int i = 0; i < b_length - 1; ++i)
            {
                p[i] = p[i + 1];
                b[i] = b[i + 1];
            }
            assigned_task = p[0];
            target[0][0] = t[assigned_task].posX;
            target[0][1] = t[assigned_task].posY;
            target[0][2] = t[assigned_task].id; // FIXME assigning a uint16_t to double
            target[0][3] = int(t[assigned_task].type);
            state = GO_TO_GOAL;
            target_valid = 1;
            clock_goal = clock;
            target_list_length += 1;
            // printf("Robot id%d new assigned task %d at time %d and is in state %d.\n", robot_id, assigned_task, clock_goal, state);

        }
        else
        {
            target_valid = 0;
            state = DEFAULT_STATE;
            assigned_task = -1;
        }
        b_length--;
        target_list_length -= 1;

#if DEBUG
        printf("\033[33m");
        printf("[%d]R%d: task %d(id%d) completed, Sending on channel %d(%s)\n", clock, robot_id, tmp, uint16_t(target[0][2]), wb_emitter_get_channel(emitter_tag_sup), res ? "OK" : "FAILED");
        // === print x ===
        printf("[%d]\tR%d: x: [", clock, robot_id);
        for (int i = 0; i < NUM_TASKS; ++i)
        {
            printf("%d: ", i);
            printf("%d] - [", x[i]);
        }
        printf("\n");
        // ====================
        // === print market ===
        printf("[%d]\tR%d: my market: [", clock, robot_id);
        for (int i = 0; i < NUM_TASKS; ++i)
        {
            printf("%d: ", i);
            printf("R%dB%d] - [", y_winners[i], y_bids[i]);
        }
        printf("\n");
        // ====================
        printf("\033[0m");
#endif
    }
}

void EpuckDistributedPlan::run_custom_pre_update()
{

    // don't perform this step of algorithm when performing tasks
    if (state == PERFORMING_TASK)
        return;

    // PHASE 2.2
    // drop assigned task if necessary
    if (is_assigned() && (b_length > 0))
    { // ==-2
// someone else is already executing my task, drop it

        // x[assigned_task] = y_bids[assigned_task];
        // assigned_task = -1;
        // target_list_length -= 1;
        make_plan_valid();
    }
    else if (is_assigned() && (is_my_bid_better(x[assigned_task], y_bids[assigned_task]) == 0) && (y_winners[assigned_task] != robot_id))
    {
        bool condNeighborBetterBid = (is_my_bid_better(x[assigned_task], y_bids[assigned_task]) == 0);
        bool condSameBid = (x[assigned_task] == y_bids[assigned_task]);
        bool condNeighborBetterID = (y_winners[assigned_task] < robot_id); // if tie, task assigned to robot with lower ID
        if (condNeighborBetterBid || (condSameBid && condNeighborBetterID))
        {
#if DEBUG
            printf("[%d]R%d: dropping task %d because my bid is %d while market is %d - ", clock, robot_id, assigned_task, x[assigned_task], y_bids[assigned_task]);
            printf("conds %d %d %d\n", condNeighborBetterBid, condSameBid, condNeighborBetterID);
#endif
            // drop bid
            x[assigned_task] = 0;
            assigned_task = -1;
            target_list_length -= 1;
        }
    }

    // PHASE 1

    while (b_length < plan_length)
    {
        // printf("robot id%d db_length: %d\n", robot_id, b_length);
        // robot's plan isn't complete
        // add the most promising task to the bundle

        double expected_time = compute_cumulative_bid(b_length);

        std::fill(std::begin(h), std::end(h), 0);
        for (int i = 0; i < NUM_TASKS; ++i)
        {
            if (x[i] != 0)
                continue; // task not valid

            int bid = compute_bid(t[i], expected_time);
            int res = is_my_bid_better(bid, y_bids[i]);
            if (res == 1)
            {
                h[i] = bid; // my bid is better than market
            }
            else if (res == 0)
            {
                h[i] = 0; // bid on market is better than mine
            }
            else
            {
#if DEBUG
                printf("[%d]R%d: Error with bids %d - x=%d, y_b=%d, y_w=%d\n", clock, robot_id, i, x[i], y_bids[i], y_winners[i]);
#endif
                h[i] = 0;
            }
            if ((x[i] < 0) || (h[i] < 0))
            { // sanity check
                printf("\033[31mError bid < 0\033[0m\n");
                exit(1);
            }
            // printf("Robot id%d, bid: %d, h[i]:%d\n", robot_id, b, h[i]);
        }
        // extract max valid bid
        auto max_bid = std::max_element(std::begin(h), std::end(h));
        int max_index = std::distance(std::begin(h), max_bid);

#if DEBUG
        printf("\033[36m");
#endif
        if (*max_bid > 0)
        { // f there is at least one valid task
            // auto-assign bid
            p[b_length] = max_index;
            b[b_length] = max_index;
            x[max_index] = *max_bid;
            y_bids[max_index] = *max_bid;
            y_winners[max_index] = robot_id;
            b_length++;
        }
        else{
            break;
        }
    }

#if DEBUG
    printf("\033[36m");
#endif
    if (!is_assigned() && (b_length >0))
    { // f there is at least one valid task
        // assing the first task in the plan
        assigned_task = p[0];

        target[0][0] = t[assigned_task].posX;
        target[0][1] = t[assigned_task].posY;
        target[0][2] = t[assigned_task].id; // FIXME assigning a uint16_t to double
        target[0][3] = int(t[assigned_task].type);
        target_valid = 1; // used in general state machine
        target_list_length = target_list_length + 1;

        // printf("Robot id%d, target_list_length %d \n", robot_id, target_list_length);

        
        #if DEBUG
        printf("[%d]R%d: task assigned %d(id%d) with bid %d - market says [R%d:B%d]\n", clock, robot_id, assigned_task, int(target[0][2]), x[assigned_task], y_winners[assigned_task], y_bids[assigned_task]);
            // === print x ===
            printf("[%d]\tR%d: x: [", clock, robot_id);
            for (int i = 0; i < NUM_TASKS; ++i)
            {
                printf("%d: ", i);
                printf("%d] - [", x[i]);
            }
            printf("\n");
            // ====================
            // === print h ===
            printf("[%d]\tR%d: h: [", clock, robot_id);
            for (int i = 0; i < NUM_TASKS; ++i)
            {
                printf("%d: ", i);
                printf("%d] - [", h[i]);
            }
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my market: [", clock, robot_id);
            for (int i = 0; i < NUM_TASKS; ++i)
            {
                printf("%d: ", i);
                printf("R%dB%d] - [", y_winners[i], y_bids[i]);
            }
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my b: [", clock, robot_id);
            for (int i = 0; i < b_length; ++i)
            {
                printf("%d: ", i);
                printf("%d] - [", b[i]);
            }
            printf("\n");
            // ====================
            // === print market ===
            printf("[%d]\tR%d: my p: [", clock, robot_id);
            for (int i = 0; i < b_length; ++i)
            {
                printf("%d: ", i);
                printf("%d] - [", p[i]);
            }
            printf("\n");
            // ====================
            printf("\033[0m");
        #endif
    }

}

void EpuckDistributedPlan::run_custom_post_update()
{

    // check if task has been reached
    if (!task_in_progress && assigned_task != -1 && check_if_event_reached() == true)
    {
        task_in_progress = 1;
        clock_task = clock;
        time_active += clock - clock_goal;
        x[assigned_task] = -2;
        y_bids[assigned_task] = -2;
        // y_winners[assigned_task] = -2;
        state = PERFORMING_TASK;

#if DEBUG
        printf("\033[35m");
        printf("[%d]R%d: reached task %d\n", clock, robot_id, assigned_task);
        // === print x ===
        printf("[%d]\tR%d: x: [", clock, robot_id);
        for (int i = 0; i < NUM_TASKS; ++i)
        {
            printf("%d: ", i);
            printf("%d] - [", x[i]);
        }
        printf("\n");
        // ====================
        // === print market ===
        printf("[%d]\tR%d: my market: [", clock, robot_id);
        for (int i = 0; i < NUM_TASKS; ++i)
        {
            printf("%d: ", i);
            printf("R%dB%d] - [", y_winners[i], y_bids[i]);
        }
        printf("\n");
        // === print market ===
            printf("[%d]\tR%d: my p: [", clock, robot_id);
            for (int i = 0; i < b_length; ++i)
            {
                printf("%d: ", i);
                printf("%d] - [", p[i]);
            }
            printf("\n");
        // ====================
        printf("\033[0m");
#endif
    }

    // broadcast new version of my market
    if (can_start_broadcatsting)
    {
        message_t msg;
        msg.robot_id = -1; // indended receiver - broadcast
        msg.sender_id = robot_id;
        msg.event_state = MSG_DISTRIBUTED_MARKET;
        memcpy(msg.market_bids, y_bids, sizeof(y_bids));
        memcpy(msg.market_winners, y_winners, sizeof(y_winners));
        for (int i = 1; i < NUM_ROBOTS + 1; ++i)
        {
            // iterate over all channels to share market
            if (i == robot_id + 1)
                continue;
            wb_emitter_set_channel(emitter_tag, i);
            wb_emitter_send(emitter_tag, &msg, sizeof(message_t));
        }
    }

    // reset newly_received_task
    newly_received_task = -1;
}

int EpuckDistributedPlan::compare_bids(int bid1, int bid2)
{
    // return best bid
    return std::max(bid1, bid2);
}

int EpuckDistributedPlan::is_my_bid_better(int myBid, int otherBid)
{

    /*
        return 1: my bid is better
        return 0: other bid is better
        return -1: invalid bids
    */

    if ((myBid < 0) || (otherBid < 0)) // when does this become important ?
        return -1;

    return (myBid == compare_bids(myBid, otherBid)) ? 1 : 0;
}

int EpuckDistributedPlan::compute_bid(task_t task, double expected_time)
{
    // printf("R%d: my pos %.2f %.2f - task pos: %.2f %.2f\n", robot_id, my_pos[0], my_pos[1], task.posX, task.posY);
    float d = dist(task.posX, task.posY, my_pos[0], my_pos[1]);
    float t = get_task_time(robot_type, task.type);

    int bid = std::floor(1 / (d / 0.1 + t + expected_time) * 1000); // 0.5 is epuck max velocity
    // printf("R%d: bid %.2f\n", robot_id, bid);

    if (bid < 0)
    {
        printf("\033[31mError bid < 0\033[0m\n");
        exit(1);
    }

    return bid;
}

double EpuckDistributedPlan::compute_cumulative_bid(int indx)
{
    // printf("Compute cumulative bid\n");
    if (indx == 0)
        return 0.0;

    double d = dist(my_pos[0], my_pos[1], t[p[0]].posX, t[p[0]].posY);

    double *time_to = new double[b_length]();
    double *distance_to = new double[b_length]();

    // Make sure to initialize the arrays if needed
    for (int i = 0; i < b_length; ++i)
    {
        time_to[i] = 0;
        distance_to[i] = 0;
    }

    // for all the tasks inside the task list (i.e. target[i] where i goes up to target_list_length)  check if putting the current
    // event (located at (msg.event_x, msg.event_z)) in between two task results in  a smaller distance, and modify the d accordingly.
    
    for (int i = 1; i < indx; i++)
    {
        time_to[i] = time_to[i - 1] + get_task_time(robot_type, t[p[i]].type);
        distance_to[i] = distance_to[i - 1] + dist(t[p[i - 1]].posX, t[p[i - 1]].posY, t[p[i]].posX, t[p[i]].posY);
    }
    
    float dist = (distance_to[indx] + d);
    float time = get_task_time(robot_type, t[p[0]].type) + time_to[indx];

    // bid = std::floor(1 / (dist / 0.5 + time) * 1000); // 0.5 is epuck max velocity

    // printf("    Robot id%d bid %d\n", robot_id, bid);
    // printf("        dist: %f, d: %f, t: %f\n", dist, d, time);
    // printf("            task time %f, time to: %f", get_task_time(robot_type, task.type), time_to[indx]);

    delete[] time_to;
    delete[] distance_to;

    return dist / 0.1 + time;
}

bool EpuckDistributedPlan::is_assigned()
{
    if (assigned_task == -1)
        return false;
    else
        return true;
}

void EpuckDistributedPlan::make_plan_valid()
{   printf("make plan valid...........................................\n");
    int *invalid = new int[b_length]();
    int invalid_no = 0;
    for(int i = 0; i < b_length; ++i)
    {
        if(y_bids[p[i]] < 0)
        {
            invalid[i] = 1;
            invalid_no ++;
        }
    }
    int og_length = b_length;
    if(invalid_no == 0) return;
    for(int i = 0; i < og_length; ++i){
        if(invalid[i] == 1)
        {

#if DEBUG
        printf("[%d]R%d: dropping task %d because someone made it invalid (%d)\n", clock, robot_id, assigned_task, y_bids[assigned_task]);
#endif
            x[p[i]] = y_bids[p[i]];

            for(int j = i; j < b_length - 1; ++j){
                p[j] = p[j+1];
                b[j] = b[j+1];
            }
            b_length--;
        }
    }
    if(og_length != b_length)
        printf("Robot id%d original plan length %d, current plan length %d\n", robot_id, og_length, b_length);

    if(b_length > 0) assigned_task = p[0];
    else assigned_task = -1;

    delete[] invalid;

}