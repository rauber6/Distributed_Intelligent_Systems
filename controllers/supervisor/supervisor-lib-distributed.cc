#include "include/supervisor-lib.hpp"

SupervisorDistributed::SupervisorDistributed() : Supervisor() {

}

void SupervisorDistributed::addEvent(int8_t index){
    // create new event with a specific event index
    // printf("addEvent child with index %d\n", index);
    Event* pNewEvent;
    if(index == -1){
        pNewEvent = new Event(next_event_id_++, (int8_t) num_active_events_);
        active_events_.push_back(*pNewEvent);  // create new instance
    } else {
        pNewEvent = new Event(next_event_id_++, index);
        active_events_[index] = *pNewEvent;  // replace existing instance
    }
    events_.push_back( (unique_ptr<Event>) pNewEvent); // add to list
    assert(num_active_events_ < NUM_EVENTS); // check max. active events not reached
    t_next_event_ = clock_ + expovariate(EVENT_GENERATION_DELAY);

    num_active_events_++;

}

void SupervisorDistributed::addEvent(){
    addEvent(-1);
}

void SupervisorDistributed::buildMessage(int16_t robot_id, const Event* event,
                                   message_event_state_t event_state, message_t* msg) {
    // Call the parent class implementation
    Supervisor::buildMessage(robot_id, event, event_state, msg);
    if (event)
        msg->event_index = event->event_index;

}

void SupervisorDistributed::reset(){
    Supervisor::reset();

    //set up receiver for comms with epucks
    for(int i = 0; i < NUM_ROBOTS; ++i){
        wb_receiver_set_channel(receivers_[i], SUP_REC_BASE_CHANNEL + i + 1);
        printf("SV: receiver set to channel %d\n", wb_receiver_get_channel(receivers_[i]));
    }

    message_t msg;

    // for debugging - shut down robots
    for (int i = NUM_ROBOTS; i < 5; i++)
    {
        msg.robot_id = i;
        msg.event_state = MSG_QUIT;
        wb_emitter_set_channel(emitter_, i + 1);
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
    }

    // send robots their position
    for(int i=0; i < NUM_ROBOTS; ++i){
        buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
        wb_emitter_set_channel(emitter_, i+1);
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
    }

    // send out tasks
    for(auto e : active_events_){
        // broadcast new event
        message_t msg;
        buildMessage(-1, &e, MSG_EVENT_NEW, &msg);  // robot_id set to -1 because this is broadcasted
        for(int i=0; i < NUM_ROBOTS; ++i){
            wb_emitter_set_channel(emitter_, i+1);
            wb_emitter_send(emitter_, &msg, sizeof(message_t));
        }
        printf("SUP: New event sent %d\n", msg.event_index);
    }
    


}

bool SupervisorDistributed::step(uint64_t step_size)
{
    clock_ += step_size;


    statTotalCollisions();
    // Send and receive messages
    message_event_status_t *pmsg;
    for (int i = 0; i < NUM_ROBOTS; i++)
    {
        // Check if we're receiving data

        if (wb_receiver_get_queue_length(receivers_[i]) > 0)
        {
            assert(wb_receiver_get_queue_length(receivers_[i]) > 0);

            if (wb_receiver_get_data_size(receivers_[i]) == sizeof(message_event_status_t))
            {
                pmsg = (message_event_status_t* ) wb_receiver_get_data(receivers_[i]);
                int event_id = pmsg->event_id;
                int event_index = pmsg->event_index;
                assert(pmsg->robot_id == i);
                Event* event = events_.at(event_id).get();
                if (event_index < 0)
                {
                    printf("\033[31mSV: event_index (%d) < 0\033[0m\n", event_index);
                    exit(1);
                }
                

                if (pmsg->event_state == MSG_EVENT_DONE) {
                    // update that task - generate a new task with the same ID and notify all robots of this new tasl
                    num_events_handled_++;
                    event->markDone(clock_);
                    num_active_events_--;
                    printf("SV: task %d (id%d) completed by R%d\n",event_index, event_id, pmsg->robot_id );                    
                    
                    // send new task
                    addEvent(event_index);
                    Event e = active_events_[event_index]; // use index of completed task to retrieve the newly-generated one                    
                    message_t msg;
                    buildMessage(-1, &e, MSG_EVENT_NEW, &msg);  // robot_id set to -1 because this is broadcasted
                    // send new task
                    for(int i=0; i < NUM_ROBOTS; ++i){
                        wb_emitter_set_channel(emitter_, i+1);
                        wb_emitter_send(emitter_, &msg, sizeof(message_t));
                    }

                    printf("SV: task %d (id%d) broadcasted\n", e.event_index, e.id_);
                }
            }

            wb_receiver_next_packet(receivers_[i]);
        }
    }

    // outbound
    message_t msg;
    bool is_gps_tick = false;

    if (clock_ >= t_next_gps_tick_)
    {
        is_gps_tick = true;
        t_next_gps_tick_ = clock_ + GPS_INTERVAL;
    }

    for (int i = 0; i < NUM_ROBOTS; i++)
    {
        // Send updates to the robot
        while (wb_emitter_get_channel(emitter_) != i + 1)
            wb_emitter_set_channel(emitter_, i + 1);

        if (is_gps_tick)
        {
            buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
            while (wb_emitter_get_channel(emitter_) != i + 1)
                wb_emitter_set_channel(emitter_, i + 1);
            wb_emitter_send(emitter_, &msg, sizeof(message_t));
        }
    }

    // Keep track of distance travelled by all robots
    statTotalDistance();

    // Time to end the experiment?
    if (num_events_handled_ >= TOTAL_EVENTS_TO_HANDLE || (MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME))
    {
        for (int i = 0; i < NUM_ROBOTS; i++)
        {
            buildMessage(i, NULL, MSG_QUIT, &msg);
            wb_emitter_set_channel(emitter_, i + 1);
            wb_emitter_send(emitter_, &msg, sizeof(message_t));
        }
        double clock_s = ((double)clock_) / 1000.0;
        double ehr = ((double)num_events_handled_) / clock_s;
        double perf = ((double)num_events_handled_) / stat_total_distance_;

        printf("Handled %d events in %d seconds, events handled per second = %.2f\n",
               num_events_handled_, (int)clock_ / 1000, ehr);
        printf("Performance: %f\n", perf);
        printf("Total Collision: %d \n", stat_total_collisions_);

        return false;
    }
    else
    {
        return true;
    } // continue
}
