#include <ros/ros.h>
#include "../include/simulator/planner.h"


int main(int argc,char **argv){
    ros::init(argc,argv,"planner");
    ros::NodeHandle ph("~");

    ros::Rate transitRate(0.5);
    state_machine::fsm_ machine;
    ros::Rate loopRate(30);
     
    machine.start();    
     int i=0;
     while(ros::ok()&&i<150){
         i++;
         loopRate.sleep();
     }
  
    // auto state = std::async(std::launch::async, state_machine::statePublish, ph, &machine);
    machine.process_event(state_machine::CmdTakeoff());       
    state_machine::echo_state(machine);

     i=0;
     while(ros::ok()&&i<150){
         i++;
         loopRate.sleep();
     }
    machine.process_event(state_machine::CmdGotoP1());
    state_machine::echo_state(machine);

    i=0;
     while(ros::ok()&&i<150){
         i++;
         loopRate.sleep();
     }
    machine.process_event(state_machine::CmdGotoP2());
    state_machine::echo_state(machine);

    

    machine.stop();
    return 0;
}
    
