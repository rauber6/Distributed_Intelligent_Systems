#include "include/e-puck-lib.hpp"
#include "../supervisor/include/taskType.hpp"

EpuckDistributed::EpuckDistributed() : Epuck(){
    //TODO set emitter range to 0.3

}

void EpuckDistributed::reset(){
    // printf("Epuck Reset in distributed child\n");
    Epuck::reset();
    wb_receiver_set_channel(receiver_tag, WB_CHANNEL_BROADCAST);
    // printf("Epuck Reset completed\n");

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
   
   std::cout << "Epuck " << robot_id << ": new message arrived with id " << msg.event_index  << std::endl;
           
}

void EpuckDistributed::msgEventCustom(message_t msg){

}

void EpuckDistributed::update_state_custom(){

}

void EpuckDistributed::run_custom_pre_update(){

}

void EpuckDistributed::run_custom_post_update(){
    
}