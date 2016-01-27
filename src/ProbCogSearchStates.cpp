#include "ProbCogSearchStates.h"
#include <iostream>
#define D_SMALL 0.3
#define D_IK 0.2

point_3d arm_state::target = point_3d(3, 0);
float arm_state::target_pitch = 0.f;

arm_state::arm_state() :
    position(fetch_arm::get_num_joints(), 0)
{
}

arm_state::arm_state(pose p, float h) : position(p),
                                        hand_position(h)
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
    pose child = fetch_arm::apply(position, a);
    return arm_state(child);
}

bool arm_state::action_valid(action a)
{
    bool is_collision = subdivision_collision_check<arm_state, action>(position, a, 0);
    return !is_collision;
    return true;
}

float arm_state::cost(action a)
{
    pose next = fetch_arm::apply(position, a);
    return fetch_arm::ee_dist_to(position, fetch_arm::ee_xyz(next));
}

bool arm_state::valid() const
{
    if(!fetch_arm::is_valid(position)) return false;
    if(collision_world::collision(position, hand_position, false)) return false;
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
    Eigen::Matrix4f target_xform =
        fetch_arm::translation_matrix(target.at(0),
                                      target.at(1),
                                      target.at(2));
    target_xform *= fetch_arm::rotation_matrix(M_PI/2, Y_AXIS);

    action xyz = fetch_arm::solve_ik(position, target_xform);
    return xyz;
}

bool arm_state::is_goal() const
{
    if (target_distance() > 0.01) return false;
    if (fabs(M_PI/2 - fetch_arm::ee_rpy(position).at(1)) > 0.01) return false;
    return true;
}

float arm_state::heuristic() const
{
    // euclidean
    return target_distance();
}

float arm_state::target_distance() const
{
    return fetch_arm::ee_dist_to(position, target);
}

void arm_state::print() const
{
    std::cout << "POSE: " << std::endl;
    for (int i = 0; i < position.size(); i++)
    {
        std::cout << "\tjoint " << i << " at " << position.at(i)
                  << " rad  " << std::endl;
    }
    std::cout << std::endl;
}
