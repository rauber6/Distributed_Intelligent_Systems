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

double expovariate(double mu) { //to delete?
  double uniform = RAND;
  while (uniform < 1e-7) uniform = RAND;
  return -log(uniform) * mu;
}

void Supervisor::addEvent() {
    events_.push_back(unique_ptr<Event>(new Event(next_event_id_++))); // add to list
    assert(num_active_events_ < NUM_EVENTS); // check max. active events not reached
    num_active_events_++;
    //t_next_event_ = clock_ + expovariate(EVENT_GENERATION_DELAY); //to comment
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

double Supervisor::distanceBetweenRobots(uint16_t robot_id_1, uint16_t robot_id_2)
{
  double dx = getRobotPos(robot_id_1)[0] - getRobotPos(robot_id_2)[0];
  double dy = getRobotPos(robot_id_1)[1] - getRobotPos(robot_id_2)[1];
  return(sqrt(pow(dx,2) + pow(dy,2)));
}

void Supervisor::statTotalCollisions()
{
  for (int i = 0; i < NUM_ROBOTS; i++)
  {
    for (int j = 0; j < NUM_ROBOTS; j++)
    {
      if(i != j)
      {
        if(ongoing_collisions_[i][j] == 1 && distanceBetweenRobots(i, j) > COLLISION_RANGE + ROBOT_DIAMETER)
        {
          ongoing_collisions_[i][j] = 0;
          ongoing_collisions_[j][i] = 0;
        } 
        else if(ongoing_collisions_[i][j] == 0 && distanceBetweenRobots(i, j) < COLLISION_RANGE + ROBOT_DIAMETER)
        {
          stat_total_collisions_++;
          ongoing_collisions_[i][j] = 1;
          ongoing_collisions_[j][i] = 1;

          printf("collision between %d and %d \n", i, j);
          printf("stat total collisions: %d \n", stat_total_collisions_);
        }
      }
    }
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

void update_collisions()
{

}
