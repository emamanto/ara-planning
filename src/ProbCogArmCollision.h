#pragma once

#include "Shortcut.h"

class arm_state;
typedef std::vector<float> action;

template<>
bool subdivision_collision_check<arm_state, action>(arm_state start,
                                                    action action,
                                                    int depth);
