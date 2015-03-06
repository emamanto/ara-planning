#include "ArmStates.h"
#include <iostream>
#include <cmath>

target* target::instance = 0;

target* target::the_instance()
{
    if (!instance) instance = new target();
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
    return Arm::the_instance()->is_valid(position);
}

bool arm_state::is_goal() const
{
    float x = Arm::the_instance()->get_ee_x_at(position);
    float y = Arm::the_instance()->get_ee_y_at(position);

    target* t = target::the_instance();
    if (x < (t->x - t->err_x) || x > (t->x + t->err_x)) return false;
    if (y < (t->y - t->err_y) || y > (t->y + t->err_y)) return false;
    return true;
}

float arm_state::heuristic()
{
    float x = Arm::the_instance()->get_ee_x_at(position);
    float y = Arm::the_instance()->get_ee_y_at(position);

    float t_x = target::the_instance()->x;
    float t_y = target::the_instance()->y;

    return sqrt(pow(t_x-x, 2) + pow(t_y-y, 2));
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
