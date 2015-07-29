#include "ProbCogSearchStates.h"
#include <iostream>
#define D_SMALL 0.1
#define D_IK 0.05

point_3d arm_state::target = point_3d(3, 0);
float arm_state::target_pitch = 0.f;
bool arm_state::pitch_matters = true;

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
    if(!probcog_arm::is_valid(position)) return false;
    if(collision_world::collision(position)) return false;
    return true;
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
    action xyz = probcog_arm::solve_ik(position, target);
    if (!pitch_matters) return xyz;

    action pitch = probcog_arm::solve_gripper(position, target_pitch);

    action total;
    bool invalid = true;
    for (int i = 0; i < probcog_arm::get_num_joints(); i++)
    {
        total.push_back(xyz.at(i) + pitch.at(i));
        if (pitch.at(i) != 0) invalid = false;
    }
    invalid = (invalid ||
               probcog_arm::ee_dist_to(probcog_arm::apply(position, total),
                                       target) > 0.01);
    if (invalid) return pitch;
    return total;
}

bool arm_state::is_goal() const
{
    if (target_distance() < 0.01 &&
        fabs(probcog_arm::ee_pitch(position) - target_pitch) < 0.01) return true;
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
