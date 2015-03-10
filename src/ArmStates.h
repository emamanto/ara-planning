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

class obstacles
{
public:
    static obstacles* the_instance();
    std::vector<obstacle> get_obstacles() { return obs; }
    obstacle obstacle_num(int i) { return obs.at(i); }
    int num_obstacles() { return obs.size(); }
    void init(std::vector<obstacle> i) { obs = i; }

private:
    static obstacles* instance;
    obstacles() {};
    obstacles(obstacles const&) {};

    std::vector<obstacle> obs;
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
    bool small_steps() const;
    bool is_goal() const;
    float heuristic() const;
    void print() const;
};
