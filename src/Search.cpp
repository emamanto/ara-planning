#include "Search.h"
#include <math.h>

using namespace std;

Search::Search() : arm(2),
                   target(arm.get_ee_x(), arm.get_ee_y(), 0, 0)
{
}

Search::Search(Arm& start) :
    arm(start),
    target(arm.get_ee_x(), arm.get_ee_y(), 0, 0)
{
}

Search::Search(int arm_num_joints) :
    arm(arm_num_joints),
    target(arm.get_ee_x(), arm.get_ee_y(), 0, 0)
{
}

float Search::euclidean_heuristic()
{
    return euclidean_heuristic(arm.get_joints());
}

float Search::euclidean_heuristic(vector<float> position)
{
    float x = arm.get_ee_x_at(position);
    float y = arm.get_ee_y_at(position);

    return sqrt(pow(target.x-x, 2) + pow(target.y-y, 2));
}

const Arm Search::get_current_arm()
{
    return arm;
}

void Search::set_arm(Arm& a)
{
    arm = a;
}

void Search::set_arm_position(vector<float> angles)
{
    arm.set_joints(angles);
}

void Search::set_arm_num_joints(int num_joints)
{
    arm = Arm(num_joints);
}

void Search::set_target(target_t target)
{
    target = target;
}

void Search::set_target(float x, float y, float err_x, float err_y)
{
    target = target_t(x, y, err_x, err_y);
}

vector<float> Search::astar(Arm start, target_t target)
{
    set_arm(start);
    set_target(target);
    return astar();
}

vector<float> Search::astar(target_t target)
{
    set_target(target);
    return astar();
}

vector<float> Search::astar(vector<float> start, target_t target)
{
    set_arm_position(start);
    set_target(target);
    return astar();
}

vector<float> Search::astar()
{
    return vector<float>(arm.get_num_joints(), 0.f);
}
