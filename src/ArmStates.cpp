#include "ArmStates.h"
#include <iostream>
#include <cmath>

target* target::instance = 0;

target* target::the_instance()
{
    if (!instance) instance = new target();
    return instance;
}

obstacles* obstacles::instance = 0;

obstacles* obstacles::the_instance()
{
    if (!instance) instance = new obstacles();
    return instance;
}

arm_state::arm_state() :
    position(Arm::the_instance()->get_num_joints(), 0)
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
    Arm* arm = Arm::the_instance();
    pose next = arm->apply_at(a, position);
    return arm_state(next);
}

float arm_state::cost(action a)
{
    Arm* arm = Arm::the_instance();
    pose next = arm->apply_at(a, position);
    float dx = arm->get_ee_x_at(position) - arm->get_ee_x_at(next);
    float dy = arm->get_ee_y_at(position) - arm->get_ee_y_at(next);
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

bool arm_state::valid() const
{
    Arm* a = Arm::the_instance();
    if (!a->is_valid(position)) return false;

    std::vector<obstacle> obs = obstacles::the_instance()->get_obstacles();

    for (std::vector<obstacle>::iterator o = obs.begin();
         o != obs.end(); o++)
    {
        if (a->collision(*o, position)) return false;
    }

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
    return Arm::the_instance()->solve_ik(target::the_instance()->x,
                                         target::the_instance()->y,
                                         position);
}

bool arm_state::is_goal() const
{
    if (target_distance() < 0.01) return true;
    else return false;
}

float arm_state::heuristic() const
{
    return target_distance();
}

float arm_state::target_distance() const
{
    return Arm::the_instance()->get_ee_dist_to(target::the_instance()->x,
                                               target::the_instance()->y,
                                               position);
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
