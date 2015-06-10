#pragma once

#include "ProbCogSearchStates.h"
#include "Shortcut.h"

template<>
bool subdivision_collision_check<arm_state, action>(arm_state start,
                                                    action action,
                                                    int depth);
