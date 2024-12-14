//******************************************************************************
//  Name:   supervisor-lib.hpp
//  Author: -
//  Date: -
//  Rev: -
//******************************************************************************

#include <stdexcept>

#include <assert.h>
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <iostream>

#include <vector>
#include <memory>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <random>

using namespace std;

#include "include/Point2d.h"
#include "include/message.h"

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#define DBG(x) printf x
#define RAND ((float) rand()/RAND_MAX)

#define STEP_SIZE 64            // simulation step size
#define AUCTION_TIMEOUT 1000    // number of steps after which an auction stops

#define EVENT_RANGE (0.1)      // distance within which a robot must come to do event
#define EVENT_TIMEOUT (10000)   // ticks until an event auction runs out
#define EVENT_GENERATION_DELAY (1000) // average time between events ms (expo distribution)

#define GPS_INTERVAL (500)

// Parameters that can be changed
#define NUM_ROBOTS 5               // Change this also in the epuck_crown.c
#define NUM_EVENTS 10               // number of total tasks //FIXME remove and use NUM_TASK in message.h
#define TOTAL_EVENTS_TO_HANDLE 100   // Events after which simulation stops or...
#define MAX_RUNTIME (3*60*1000)      // ...total runtime after which simulation stops - 3
#define COLLISION_RANGE (0.01)
#define ROBOT_DIAMETER (0.07)

#define WB_CHANNEL_BROADCAST -1
#define SUPERVISOR_SENDER_ID 999

#define SUP_REC_BASE_CHANNEL 900
#define RX_PERIOD 2  // time difference between two received elements (ms) (1000)


extern WbNodeRef g_event_nodes[NUM_EVENTS];
extern vector<WbNodeRef> g_event_nodes_free;

double gauss(void);
double rand_coord();
double expovariate(double mu);
// --------------------------------------

// Event class
class Event {

// Public variables
public:
  uint16_t id_;          //event id - unique
  Point2d pos_;          //event pos
  WbNodeRef node_;       //event node ref
  uint16_t assigned_to_; //id of the robot that will handle this event

  uint8_t event_index;   //index in [0, NUM_EVENT -1]
  TaskType taskType;

  // Auction data
  uint64_t t_announced_;        //time at which event was announced to robots
  bitset<NUM_ROBOTS> bids_in_;
  bitset<NUM_ROBOTS> bids_out_;
  uint16_t best_bidder_;        //id of the robot that had the best bid so far
  double best_bid_;             //value of the best bid (lower is better)
  uint64_t t_done_;             //time at which the assigned robot reached the event
  int bidder_index;             //index at which the bidder will put event in tasklist

  uint64_t reached_;
  uint64_t in_progress_;

// Public functions
public:
  //Event creation
  Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()),
    assigned_to_(-1), t_announced_(-1), best_bidder_(-1), best_bid_(0.0), t_done_(-1), reached_(0.0), in_progress_(0.0)
  {
    node_ = g_event_nodes_free.back();  // Place node
    g_event_nodes_free.pop_back();

    taskType = generate_random_task();
    // cout << "Task " << strType(taskType) << " generate" << endl;
    
    double event_node_pos[3];           // Place event in arena
    event_node_pos[0] = pos_.x;
    event_node_pos[1] = pos_.y;
    event_node_pos[2] = .01;
    wb_supervisor_field_set_sf_vec3f(
      wb_supervisor_node_get_field(node_,"translation"),
      event_node_pos);

    // set color properly
    if (node_ != NULL) {
      // Get the Shape node within the Transform node
      WbFieldRef children_field = wb_supervisor_node_get_field(node_, "children");
      WbNodeRef shape_node = wb_supervisor_field_get_mf_node(children_field, 0);

      if (shape_node != NULL) {
        // Get the Appearance node within the Shape node
        WbFieldRef appearance_field = wb_supervisor_node_get_field(shape_node, "appearance");
        WbNodeRef appearance_node = wb_supervisor_field_get_sf_node(appearance_field);

        if (appearance_node != NULL) {
          // Get the Material node within the Appearance node
          WbFieldRef material_field = wb_supervisor_node_get_field(appearance_node, "material");
          WbNodeRef material_node = wb_supervisor_field_get_sf_node(material_field);

          if (material_node != NULL) {
            // Change the diffuseColor field of the Material node
            WbFieldRef color_field = wb_supervisor_node_get_field(material_node, "diffuseColor");
            double* new_color = getColor(taskType); // get correct color according to task type
            wb_supervisor_field_set_sf_color(color_field, new_color);
          }
        }
      }
    }
  }
  
  Event(uint16_t id, uint8_t index) : Event(id) {
    event_index = index;
  }

  bool is_assigned() const { return assigned_to_ != (uint16_t) -1; }
  bool was_announced() const { return t_announced_ != (uint64_t) -1; }
  bool was_reached() const { return reached_ != (uint64_t) -1; }
  bool has_bids() const { return best_bidder_ != (uint16_t) -1; }
  bool is_done() const { return t_done_ != (uint64_t) -1; }

  // Check if event can be assigned
  void updateAuction(uint16_t bidder, double bid, int index) {
    if (bid >= 0.0 && (!has_bids() || bid < best_bid_)) {
      best_bidder_ = bidder;
      best_bid_ = bid;
      bidder_index = index;  
    }

    if(bid < 0){
      bids_out_.set(bidder);
    }
    else{
      bids_in_.set(bidder);
    }

    if (bids_in_.any() && (bids_in_.count() + bids_out_.count()) == NUM_ROBOTS) assigned_to_ = best_bidder_;
  }

  void restartAuction() {
    assigned_to_ = -1;
    t_announced_ = -1;
    bids_in_.reset();
    best_bidder_ = -1;
    best_bid_ = 0.0;
    t_done_ = -1;
    reached_ = 0;
    in_progress_ = 0;
  }

  void markDone(uint64_t clk) {
    t_done_ = clk;
    double event_node_pos[3] = {-5,-5,0.1};
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),
                                     event_node_pos);
    g_event_nodes_free.push_back(node_);
  }
};

class Supervisor {

//Private variables
protected:
  uint64_t clock_;

  uint16_t next_event_id_;
  vector<unique_ptr<Event> > events_;
  uint16_t num_active_events_;
  uint64_t t_next_event_;
  uint64_t t_next_gps_tick_;

  uint16_t num_events_handled_; // total number of events handled
  double stat_total_distance_;  // total distance traveled
  double stat_robot_prev_pos_[NUM_ROBOTS][2];
  double distanceBetweenRobots(uint16_t, uint16_t);
  void statTotalCollisions();
  uint16_t ongoing_collisions_[NUM_ROBOTS][NUM_ROBOTS] = {};
  uint16_t stat_total_collisions_ = 0;

  WbNodeRef robots_[NUM_ROBOTS];
  WbDeviceTag emitter_;
  WbDeviceTag receivers_[NUM_ROBOTS];
  typedef vector<pair<Event*, message_event_state_t> > event_queue_t;

  virtual void addEvent();
  void linkRobot(uint16_t id);
    // Assemble a new message to be sent to robots
  void buildMessage(int16_t robot_id, const Event* event,
      message_event_state_t event_state, message_t* msg);
  const double* getRobotPos(uint16_t robot_id);
  void setRobotPos(uint16_t robot_id, double x, double y);

  void statTotalDistance();

public:
  Supervisor() : events_(NUM_EVENTS){};
  virtual ~Supervisor(){}
  virtual void reset();
  virtual bool step(uint64_t step_size) = 0;

};

void link_event_nodes();



class SupervisorCentralised : public Supervisor{
    private:
        Event* auction; // the event currently being auctioned
        void markEventsDone(event_queue_t& event_queue);
        void markEventsReached(event_queue_t& event_queue);
        void handleAuctionEvents(event_queue_t& event_queue);
    public:
        SupervisorCentralised():Supervisor(){} 
        ~SupervisorCentralised() override {}
        void reset() override;
        bool step(uint64_t step_size) override;
};




class SupervisorDistributed : public Supervisor {
  public:
    SupervisorDistributed();
    ~SupervisorDistributed() override {}
    void reset() override;
    bool step(uint64_t step_size) override;

  private:
    std::vector<Event> active_events_;
    void addEvent() override;
    void addEvent(int8_t index);
    void buildMessage(int16_t robot_id, const Event* event, message_event_state_t event_state, message_t* msg);

    void markEventsReached(event_queue_t& event_queue);

};
