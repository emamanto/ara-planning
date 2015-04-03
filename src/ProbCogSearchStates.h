#pragma once

#include "ProbCogArm.h"

// Doesn't actually check if this is a self-consistent world.
// Don't make objects on wrong sides of walls.
class object_world
{
public:
    point_3d object_xyz;
    std::vector<float> obj_dim;
    float wall_x, wall_y;

    bool collision(pose p);
    point_3d grasp_point();
};

class arm_state
{
public:
    pose position;
    static object_world world;
    static point_3d target;

    arm_state();
    arm_state(pose p);

    bool operator == (const arm_state& other) const;
    bool operator < (const arm_state& other) const;
    bool operator > (const arm_state& other) const;

    arm_state apply(action a);
    float cost(action a);
    bool valid() const;
    bool small_steps() const;
    bool use_finisher() const;
    action compute_finisher() const;
    bool is_goal() const;
    float heuristic() const;
    float target_distance() const;
    void print() const;
};
