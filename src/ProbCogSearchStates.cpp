#include "ProbCogSearchStates.h"
#include <iostream>
#define D_SMALL 0.15
#define D_IK 0.05

point_3d arm_state::target = point_3d(3, 0);

arm_state::arm_state() :
    position(probcog_arm::get_num_joints(), 0)
{
}

arm_state::arm_state(pose p) : position(p)
{
}

bool arm_state::operator == (const arm_state& other) const
{
    return (position == other.position);
}

bool arm_state::operator < (const arm_state& other) const
{
    return (position < other.position);
}

bool arm_state::operator > (const arm_state& other) const
{
    return (position > other.position);
}

arm_state arm_state::apply(action a)
{
    pose child = probcog_arm::apply(position, a);
    return arm_state(child);
}

float arm_state::cost(action a)
{
    pose next = probcog_arm::apply(position, a);
    return probcog_arm::ee_dist_to(position, probcog_arm::ee_xyz(next));
}

bool arm_state::valid() const
{
    return probcog_arm::is_valid(position);
}

bool arm_state::small_steps() const
{
    if (target_distance() < D_SMALL) return true;
    return false;
}

bool arm_state::use_finisher() const
{
    if (target_distance() < D_IK) return true;
    return false;
}

action arm_state::compute_finisher() const
{
    return probcog_arm::solve_ik(position, target);
}

bool arm_state::is_goal() const
{
    if (target_distance() < 0.001) return true;
    else return false;
}

float arm_state::heuristic() const
{
    // euclidean
    return target_distance();
}

float arm_state::target_distance() const
{
    return probcog_arm::ee_dist_to(position, target);
}

void arm_state::print() const
{
    std::cout << "POSE: ";
    for (int i = 0; i < position.size(); i++)
    {
        std::cout << "joint " << i << " at " << position.at(i)
                  << " degrees ";
    }
    std::cout << std::endl;
}
