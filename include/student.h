#pragma once

#include "raylib.h"

#if defined(__cplusplus)
extern "C" {            // Prevents name mangling of functions
#endif


// Be aware that Vector2 does not exist in your current code and would need to be defined as global variables
// Student write the controller interface here!

Vector2 WheelToVelocity(float desiredV, float desiredW){
    Vector2 command = {0.0, 0.0};

    return command;
}

#if defined(__cplusplus)
}
#endif