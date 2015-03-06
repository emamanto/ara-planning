#pragma once

#include "Arm.h"
#include <map>

class target
{
public:
    static target* the_instance();

    float x, y;
    float err_x, err_y;

private:
    static target* instance;
    target() : x(0), y(0), err_x(0), err_y(0) {};
    target(target const&) {};
};

class arm_state
{
public:
    pose position;

    arm_state();
    arm_state(pose p);

    bool operator == (const arm_state& other) const;
    bool operator < (const arm_state& other) const;
    bool operator > (const arm_state& other) const;

    arm_state apply(action a);
    float cost(action a);
    bool valid() const;
    bool is_goal() const;
    float heuristic();
    void print() const;
};
