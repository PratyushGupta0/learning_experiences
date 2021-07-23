#include "concrete_fms_states.h"

void no_movement::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(move_upward::get_Instance());
}

fms_state& no_movement::get_Instance()
{
	static no_movement singleton;
	return singleton;
}

void move_upward::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(move_forward::get_Instance());
}

fms_state& move_upward::get_Instance()
{
	static move_upward singleton;
    return singleton;
}

void move_forward::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(move_constant_speed::get_Instance());
}

fms_state& move_forward::get_Instance()
{
	static move_forward singleton;
	return singleton;
}


void move_constant_speed::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(no_movement::get_Instance());
}

fms_state& move_constant_speed::get_Instance()
{
	static move_constant_speed singleton;
	return singleton;
}