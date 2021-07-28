#pragma once
#include <iostream>

#include "fms_state.h"

class fms_state;

class fms
{
    public:
        fms();
        inline  fms_state* getCurrentState() const { return currentState; }
        void toggle();
        // This is where the magic happens
        void setState   (fms_state& newState);

    private:
        fms_state* currentState;
};