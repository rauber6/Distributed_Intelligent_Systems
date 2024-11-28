#include "include/supervisor-lib.hpp"

WbNodeRef g_event_nodes[NUM_EVENTS];
vector<WbNodeRef> g_event_nodes_free;

double gauss(void) 
{
  double x1, x2, w;
  do {
      x1 = 2.0 * RAND - 1.0;
      x2 = 2.0 * RAND - 1.0;
      w = x1*x1 + x2*x2;
  } while (w >= 1.0);

  w = sqrt((-2.0 * log(w))/w);
  return(x1*w);
}

double rand_coord() {
  // return -1.0 + 2.0*RAND;
  // return -0.95 + 1.9*RAND;
  return -0.45 + 0.9*RAND;  // FIXME arena dimentions????
}

double expovariate(double mu) {
  double uniform = RAND;
  while (uniform < 1e-7) uniform = RAND;
  return -log(uniform) * mu;
}

void Supervisor::addEvent() {
    events_.push_back(unique_ptr<Event>(new Event(next_event_id_++))); // add to list
    assert(num_active_events_ < NUM_EVENTS); // check max. active events not reached
    num_active_events_++;
    t_next_event_ = clock_ + expovariate(EVENT_GENERATION_DELAY);
  }

  // Init robot and get robot_ids and receivers
void Supervisor::linkRobot(uint16_t id) {
    const char kRobotNameFormat[] = "e-puck(%d)";
    const char kReceiverNameFormat[] = "rec%d";
    char node_name[16];

    // Get the robot node's handle
    snprintf(node_name, sizeof(node_name), kRobotNameFormat, id);
    robots_[id] = wb_supervisor_node_get_from_def(node_name);
    if (!robots_[id]) {
      DBG(("Missing node for robot #%d\n", id));
      exit(1);
    }

    // Get the respective receiver
    snprintf(node_name, sizeof(node_name), kReceiverNameFormat, id); 
    // printf("%s %s %d", node_name, kReceiverNameFormat, id);
    receivers_[id] = wb_robot_get_device(node_name);
    if (!receivers_[id]) {
      DBG(("Missing receiver for robot #%d\n", id));
      exit(1);
    }
    wb_receiver_enable(receivers_[id], 2); //32
    wb_receiver_set_channel(receivers_[id], id+1);
  }

  // Assemble a new message to be sent to robots
void Supervisor::buildMessage(uint16_t robot_id, const Event* event,
      message_event_state_t event_state, message_t* msg) {
    WbFieldRef f_rot = wb_supervisor_node_get_field(robots_[robot_id],
                                                    "rotation");
    const double *pos = getRobotPos(robot_id);
    const double *rot = wb_supervisor_field_get_sf_rotation(f_rot);

    

    msg->robot_id = robot_id;
    msg->robot_x = pos[0]; // no gps noise used here
    msg->robot_y = pos[1]; // no gps noise used here
    double heading = -rot[2] *rot[3]; // no gps noise used here
    msg->heading = heading > 2*M_PI ? heading - 2*M_PI : heading;
    msg->event_state = event_state;
    msg->event_id = -1;
    msg->event_x = 0.0;
    msg->event_y = 0.0;
    

    if (event) {
      assert(event_state != MSG_EVENT_INVALID && 
             event_state != MSG_EVENT_GPS_ONLY);
      msg->event_id = event->id_;
      msg->event_type = event->taskType;
      msg->event_x = event->pos_.x;
      msg->event_y = event->pos_.y;
      msg->event_index = event->bidder_index;
    }
  }

const double* Supervisor::getRobotPos(uint16_t robot_id) {
    WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id],
      "translation");
    return wb_supervisor_field_get_sf_vec3f(f_pos);
  }

void Supervisor::setRobotPos(uint16_t robot_id, double x, double y) {
    WbFieldRef f_pos = wb_supervisor_node_get_field(robots_[robot_id],
      "translation");
    double pos[3] = {x, y, 0.01};
    return wb_supervisor_field_set_sf_vec3f(f_pos, pos);
  }

  // Marks one event as done, if one of the robots is within the range
void Supervisor::markEventsDone(event_queue_t& event_queue) {
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

void Supervisor::markEventsReached(event_queue_t& event_queue) {
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

void Supervisor::handleAuctionEvents(event_queue_t& event_queue) {
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

  // Calculate total distance travelled by robots
void Supervisor::statTotalDistance() {
    for (int i=0; i<NUM_ROBOTS; ++i) {
      const double *robot_pos = getRobotPos(i);
      double delta[2] = {
        robot_pos[0] - stat_robot_prev_pos_[i][0],
        robot_pos[1] - stat_robot_prev_pos_[i][1]
      };
      stat_total_distance_ += sqrt(delta[0]*delta[0] + delta[1]*delta[1]);
      stat_robot_prev_pos_[i][0] = robot_pos[0];
      stat_robot_prev_pos_[i][1] = robot_pos[1];
    }
  }

  // Reset robots & events
void Supervisor::reset() {
    clock_ = 0;

    // initialize & link events
    next_event_id_ = 0;
    events_.clear();
    num_active_events_ = 0;
    t_next_event_ = 0; // invalid state
    auction = NULL;
    t_next_gps_tick_ = 0;

    num_events_handled_ = 0;
    stat_total_distance_ = 0.0;

    // add the first few events
    for (int i=0; i<NUM_EVENTS; ++i) {
      addEvent();
    }

    // link & initialize robots
    for (int i=0;i<NUM_ROBOTS;i++) {
      linkRobot(i);

      double pos[2] = {rand_coord(), rand_coord()};
      setRobotPos(i, pos[0], pos[1]);
      stat_robot_prev_pos_[i][0] = pos[0];
      stat_robot_prev_pos_[i][1] = pos[1];
    }

    // initialize the emitter
    emitter_ = wb_robot_get_device("sup_emitter");
    if (!emitter_) {
      DBG(("Missing supervisor emitter!\n"));
      exit(1);
    }
  }

  //Do a step
bool Supervisor::step(uint64_t step_size) {
    
    clock_ += step_size;

    // Events that will be announced next or that have just been assigned/done
    event_queue_t event_queue;

    markEventsReached(event_queue);
    // markEventsDone(event_queue);

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

//Links up all the nodes we are interested in.
//Gets called by webots at robot_live(reset)
void link_event_nodes() {
  const char kEventNameFormat[] = "e%d";
  char node_name[16];
  
  for (int i=0; i<NUM_EVENTS; ++i) {
    snprintf(node_name, sizeof(node_name), kEventNameFormat, i);
    g_event_nodes[i] = wb_supervisor_node_get_from_def(node_name);
    g_event_nodes_free.push_back(g_event_nodes[i]);
  }
}