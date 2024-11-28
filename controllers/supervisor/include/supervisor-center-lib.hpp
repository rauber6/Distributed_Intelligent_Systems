#include "supervisor-lib.hpp"

class SupervisorCentralised : public Supervisor{
    private:
        Event* auction; // the event currently being auctioned
        void markEventsDone(event_queue_t& event_queue);
        void markEventsReached(event_queue_t& event_queue);
        void handleAuctionEvents(event_queue_t& event_queue);
    public:
        SupervisorCentralised():Supervisor(){} 
        void reset();
        bool step(uint64_t step_size);
};