
#include <iostream>
#include <string>
#include<math.h>


#include "fms.h"
#include "concrete_fms_states.h"

fms::fms()
{
	// All lights are initially turned off
	currentState = &no_movement::get_Instance();
}

void fms::setState(fms_state& newState)
{
	currentState->exit(this);  // do stuff before we change state
	currentState = &newState;  // actually change states now
	currentState->enter(this); // do stuff after we change state
}

void fms::toggle()
{
	// Delegate the task of determining the next state to the current state
	currentState->toggle(this);
}
int main()
{
    fms iris;

    return 0;
}