#include "include/supervisor-center-lib.hpp"

  // Marks one event as done, if one of the robots is within the range
void SupervisorCentralised::markEventsDone(event_queue_t& event_queue) {
    for (auto& event : events_) {
      if (!event->is_assigned() || event->is_done())
        continue;
      
      const double *robot_pos = getRobotPos(event->assigned_to_);
      Point2d robot_pos_pt(robot_pos[0], robot_pos[1]);
      double dist = event->pos_.Distance(robot_pos_pt);

      if (dist <= EVENT_RANGE) {
        printf("D robot %d reached event %d\n", event->assigned_to_,
          event->id_);
        num_events_handled_++;
        event->markDone(clock_);
        num_active_events_--;
        event_queue.emplace_back(event.get(), MSG_EVENT_DONE);
      }
    }
  }

void SupervisorCentralised::markEventsReached(event_queue_t& event_queue) {
    for (auto& event : events_) {
      if (!event->is_assigned() || event->is_done())
        continue;
      
      const double *robot_pos = getRobotPos(event->assigned_to_);
      Point2d robot_pos_pt(robot_pos[0], robot_pos[1]);
      double dist = event->pos_.Distance(robot_pos_pt);

      if (dist <= EVENT_RANGE && !event->reached_  && event->in_progress_) { // 
        printf("D robot %d reached event %d\n", event->assigned_to_,
          event->id_);
        event->reached_ = 1;
        event_queue.emplace_back(event.get(), MSG_EVENT_REACHED);
      }
    }
  }

void SupervisorCentralised::handleAuctionEvents(event_queue_t& event_queue) {
    // For each unassigned event
    for (auto& event : events_) {
      if (event->is_assigned()) continue;

      // Send announce, if new
      // IMPL DETAIL: Only allow one auction at a time.
      if (!event->was_announced() && !auction) {
        event->t_announced_ = clock_;
        event_queue.emplace_back(event.get(), MSG_EVENT_NEW); 
        auction = event.get();
        printf("A event %d announced\n", event->id_);

      // End early or restart, if timed out
      } else if (clock_ - event->t_announced_ > EVENT_TIMEOUT) {
        // End early if we have any bids at all
        if (event->has_bids()) {
          // IMPLEMENTATION DETAIL: If about to time out, assign to
          // the highest bidder or restart the auction if there is none.
          event->assigned_to_ = event->best_bidder_;
          event_queue.emplace_back(event.get(), MSG_EVENT_WON); // FIXME?
          auction = NULL;
          printf("W robot %d won event %d\n", event->assigned_to_, event->id_);

        // Restart (incl. announce) if no bids
        } else {
          // (reannounced in next iteration)
          event->restartAuction();
          if (auction == event.get())
            auction = NULL;
        }
      }
    }
  }

  void SupervisorCentralised::reset(){
    Supervisor::reset();
    auction = NULL;
  }

    //Do a step
bool SupervisorCentralised::step(uint64_t step_size) {

    clock_ += step_size;

    // Events that will be announced next or that have just been assigned/done
    event_queue_t event_queue;

    markEventsReached(event_queue);

    // ** Add a random new event, if the time has come
    assert(t_next_event_ > 0);
    if (clock_ >= t_next_event_ && num_active_events_ < NUM_EVENTS) {
      addEvent();
    }

    handleAuctionEvents(event_queue);
    
    // Send and receive messages
    bid_t* pbid; // inbound
    message_event_status_t* pmsg;
    for (int i=0;i<NUM_ROBOTS;i++) {
      // Check if we're receiving data
      if (wb_receiver_get_queue_length(receivers_[i]) > 0) {
        assert(wb_receiver_get_queue_length(receivers_[i]) > 0);
        
        if(wb_receiver_get_data_size(receivers_[i]) == sizeof(bid_t)){ 
          pbid = (bid_t*) wb_receiver_get_data(receivers_[i]); 
          assert(pbid->robot_id == i);

          Event* event = events_.at(pbid->event_id).get();
          event->updateAuction(pbid->robot_id, pbid->value, pbid->event_index);
          // TODO: Refactor this (same code above in handleAuctionEvents)
          if (event->is_assigned()) {
            event_queue.emplace_back(event, MSG_EVENT_WON);
            auction = NULL;
            printf("W robot %d won event %d\n", event->assigned_to_, event->id_);
          }
        }
        else if(wb_receiver_get_data_size(receivers_[i]) == sizeof(message_event_status_t)){
          pmsg = (message_event_status_t*) wb_receiver_get_data(receivers_[i]); 
          assert(pmsg->robot_id == i);
          // std::cout << "VAL " << pmsg->event_id << std::endl;
          // std::cout << "VAL " << i << std::endl;
          // std::cout << "VAL " << events_.at(pmsg->event_id).get()->id_ << std::endl;
          Event* event = events_.at(pmsg->event_id).get();
          // std::cout << "val" << event->id_ << std::endl;
          // std::cout << "2" << std::endl;
          assert(event->in_progress_);
          // return 1;
          message_event_state_t state = pmsg->event_state;
          if(state == MSG_EVENT_DONE){
            num_events_handled_++;
            event->markDone(clock_);
            num_active_events_--;
            event_queue.emplace_back(event, MSG_EVENT_DONE);
            printf("C robot %d completed event %d\n", event->assigned_to_, event->id_);
          }
          else if (state == MSG_EVENT_NOT_IN_PROGRESS){
            event->reached_ = 0;
            event->in_progress_ = 0;
          }
          else if (state == MSG_EVENT_IN_PROGRESS){
            event->in_progress_ = 1;
          }
        }

        wb_receiver_next_packet(receivers_[i]);
      }
    }

    // outbound
    message_t msg;
    bool is_gps_tick = false;

    if (clock_ >= t_next_gps_tick_) {
      is_gps_tick = true;
      t_next_gps_tick_ = clock_ + GPS_INTERVAL;
    }

    for (int i=0;i<NUM_ROBOTS;i++) {
      // Send updates to the robot
      while (wb_emitter_get_channel(emitter_) != i+1)
      wb_emitter_set_channel(emitter_, i+1);
      
      if (is_gps_tick) {
        buildMessage(i, NULL, MSG_EVENT_GPS_ONLY, &msg);
//        printf("sending message %d , %d \n",msg.event_id,msg.robot_id);
        while (wb_emitter_get_channel(emitter_) != i+1)
            wb_emitter_set_channel(emitter_, i+1);        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }

      for (const auto& e_es_tuple : event_queue) {
        const Event* event = e_es_tuple.first;
        const message_event_state_t event_state = e_es_tuple.second;
        if (event->is_assigned() && event->assigned_to_ != i) continue;

        buildMessage(i, event, event_state, &msg);
        while (wb_emitter_get_channel(emitter_) != i+1)
              wb_emitter_set_channel(emitter_, i+1);        
//        printf("> Sent message to robot %d // event_state=%d\n", i, event_state);
//        printf("sending message event %d , robot %d , emitter %d, channel %d\n",msg.event_id,msg.robot_id,emitter_,      wb_emitter_get_channel(emitter_));
        
        wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }
    }

    // Keep track of distance travelled by all robots
    statTotalDistance();

    // Time to end the experiment?
    if (num_events_handled_ >= TOTAL_EVENTS_TO_HANDLE ||(MAX_RUNTIME > 0 && clock_ >= MAX_RUNTIME)) {
      for(int i=0;i<NUM_ROBOTS;i++){
          buildMessage(i, NULL, MSG_QUIT, &msg);
          wb_emitter_set_channel(emitter_, i+1);
          wb_emitter_send(emitter_, &msg, sizeof(message_t));
      }
      double clock_s = ((double) clock_) / 1000.0;
      double ehr = ((double) num_events_handled_) / clock_s;
      double perf = ((double) num_events_handled_) / stat_total_distance_;
      
      printf("Handled %d events in %d seconds, events handled per second = %.2f\n",
             num_events_handled_, (int) clock_ / 1000, ehr);
      printf("Performance: %f\n", perf);
      return false;
    } 
    else { return true;} //continue
  } // << step() <<
