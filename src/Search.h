#pragma once

#include <set>
#include "Arm.h"

// Rectangle centered at x, y with error err_x, err_y in each
// dimension
struct target_t
{
    float x;
    float y;
    float err_x;
    float err_y;

    target_t(float x, float y, float ex, float ey): x(x),
                                                    y(y),
                                                    err_x(ex),
                                                    err_y(ey) {}
};

// Change joint by change degrees
struct action
{
    int joint;
    float change;

    action(int j, float c): joint(j), change(c) {}
};

typedef std::vector<action> plan;

// Search node
struct node
{
    pose joints;
    action primitive;
    node* parent;
    float cost;

    node(pose j, action pr, node* pa, float c = 0) : joints(j),
                                                     primitive(pr),
                                                     parent(pa),
                                                     cost(c) {}

    bool operator < (const node& other) const {return cost > other.cost; }
    bool operator > (const node& other) const {return cost < other.cost; }
};

class Search
{
public:
    Search();
    Search(Arm& start);
    Search(int arm_num_joints);

    float get_target_x() {return target.x;}
    float get_target_y() {return target.y;}
    target_t get_target() {return target;}

    float euclidean_heuristic();
    float euclidean_heuristic(pose position);

    const Arm get_current_arm();
    void set_arm(Arm& a);
    void set_arm_position(pose angles);
    void set_arm_num_joints(int num_joints);

    void set_target(target_t target);
    void set_target(float x, float y, float err_x, float err_y);

    bool is_in_goal(pose pos);

    plan astar(Arm start, target_t target);
    plan astar(target_t target);
    plan astar(pose start, target_t target);
    plan astar();

private:
    Arm arm;
    target_t target;
    std::set<action> primitives;
};
