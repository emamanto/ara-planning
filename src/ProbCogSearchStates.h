#pragma once

#include "FetchArm.h"
#include "3DCollisionWorld.h"
#include "ProbCogArmCollision.h"

class arm_state
{
public:
    pose position;
    float hand_position;
    static point_3d target;
    static float target_pitch;

    // Make static collision world too?

    arm_state();
    arm_state(pose p, float h = 0.05);

    bool operator == (const arm_state& other) const;
    bool operator < (const arm_state& other) const;
    bool operator > (const arm_state& other) const;

    arm_state apply(action a);
    bool action_valid(action a);
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
