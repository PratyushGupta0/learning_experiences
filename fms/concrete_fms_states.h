#include "fms_state.h"
#include "fms.h"

class no_movement:public fms_state
{
    public:
        void enter(fms* fms){
            std::cout<<"Entered state 1"<<std::endl;
        }
        void toggle(fms* fms);
        void exit(fms* fms){
            std::cout<<"Exited state 1"<<std::endl;
        }
        static fms_state& get_Instance();
        
    private:
        no_movement() {}
        no_movement(const no_movement& other);
	    no_movement& operator=(const no_movement& other);

};

class move_upward:public fms_state
{
    public:
        void enter(fms* fms){
            std::cout<<"Entered state 2"<<std::endl;
        }
        void toggle(fms* fms);
        void exit(fms* fms){
            std::cout<<"Exited state 2"<<std::endl;
        }
        static fms_state& get_Instance();
        
    private:
        move_upward() {}
        move_upward(const move_upward& other);
	    move_upward& operator=(const move_upward& other);

};

class move_forward:public fms_state
{
    public:
        void enter(fms* fms){
            std::cout<<"Entered state 3"<<std::endl;
        }
        void toggle(fms* fms);
        void exit(fms* fms){std::cout<<"Exited state 3"<<std::endl;}
        static fms_state& get_Instance();
        
    private:
        move_forward() {}
        move_forward(const move_forward& other);
	    move_forward& operator=(const move_forward& other);

};

class move_constant_speed:public fms_state
{
    public:
        void enter(fms* fms){
            std::cout<<"Entered state 4"<<std::endl;
        }
        void toggle(fms* fms);
        void exit(fms* fms){std::cout<<"Exited state 4"<<std::endl;}
        static fms_state& get_Instance();
        
    private:
        move_constant_speed() {}
        move_constant_speed(const move_constant_speed& other);
	    move_constant_speed& operator=(const move_constant_speed& other);

};