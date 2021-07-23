#include "concrete_fms_states.h"

void no_movement::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(move_upward::getInstance());
}

fms_state& no_movement::getInstance()
{
	static no_movement singleton;
	return singleton;
}

void move_upward::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(move_forward::getInstance());
}

fms_state& move_upward::getInstance()
{
	static move_upward singleton;
    return singleton;
}

void move_forward::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(move_constant_speed::getInstance());
}

fms_state& move_forward::getInstance()
{
	static move_forward singleton;
	return singleton;
}


void move_constant_speed::toggle(fms* fms)
{
	// Off -> Low
	fms->setState(no_movement::getInstance());
}

fms_state& move_constant_speed::getInstance()
{
	static move_constant_speed singleton;
	return singleton;
}