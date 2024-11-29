#include "include/supervisor-lib.hpp"

SupervisorDistributed::SupervisorDistributed() : Supervisor() {

}

void SupervisorDistributed::addEvent(int8_t index){
    // create new event with a specific event index
    // printf("addEvent child with index %d\n", index);
    Event* pNewEvent;
    if(index == -1)
        pNewEvent = new Event(next_event_id_++, num_active_events_);
    else
        pNewEvent = new Event(next_event_id_++, index);
    events_.push_back( (unique_ptr<Event>) pNewEvent); // add to list
    assert(num_active_events_ < NUM_EVENTS); // check max. active events not reached
    t_next_event_ = clock_ + expovariate(EVENT_GENERATION_DELAY);

    // add it to active_events_
    active_events_[num_active_events_] = pNewEvent;
    num_active_events_++;

}

void SupervisorDistributed::addEvent(){
    addEvent(-1);
}

void SupervisorDistributed::buildMessage(int16_t robot_id, const Event* event,
                                   message_event_state_t event_state, message_t* msg) {
    // Call the parent class implementation
    // Supervisor::buildMessage(robot_id, event, event_state, msg);
    msg->robot_id = robot_id;
    msg->event_state = event_state;
    if (event)
        msg->event_index = event->event_index;

}

void SupervisorDistributed::reset(){
    // printf("supervisor reset in child\n");
    Supervisor::reset();

    for(auto e : active_events_){
        // broadcast new event
        message_t msg;
        wb_emitter_set_channel(emitter_, WB_CHANNEL_BROADCAST);
        buildMessage(-1, e, MSG_EVENT_NEW, &msg);  // robot_id set to -1 because this is broadcasted
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
        // printf("SUP: New event sent %d\n", msg.event_index);
    }

}

bool SupervisorDistributed::step(uint64_t step_size)
{
    /*
    clock_ += step_size;

    // Events that will be announced next or that have just been assigned/done
    event_queue_t event_queue;

    markEventsReached(event_queue);

    // ** Add a random new event, if the time has come
    assert(t_next_event_ > 0);
    if (clock_ >= t_next_event_ && num_active_events_ < NUM_EVENTS)
    {
        addEvent();
    }

    // Send and receive messages
    message_event_status_t *pmsg;
    for (int i = 0; i < NUM_ROBOTS; i++)
    {
        // Check if we're receiving data
        if (wb_receiver_get_queue_length(receivers_[i]) > 0)
        {
            assert(wb_receiver_get_queue_length(receivers_[i]) > 0);

            if (wb_receiver_get_data_size(receivers_[i]) == sizeof(message_event_state_t))
            {
                pmsg = (message_event_state_t* ) wb_receiver_get_data(receivers_[i]);
                assert(pmsg->robot_id == i);

                if (pmsg->event_state == MSG_EVENT_DONE) {
                    // update that task - generate a new task with the same ID and notify all robots of this new tasl
                    
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
            //        printf("sending message %d , %d \n",msg.event_id,msg.robot_id);
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
        return false;
    }
    else
    {
        return true;
    } // continue
    */
}