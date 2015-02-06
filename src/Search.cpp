#include "Search.h"
#include <math.h>
#include <queue>

Search* Search::instance = 0;

Search* Search::the_instance()
{
    if (!instance) instance = new Search();
    return instance;
}

plan Search::run_search(Arm start, target_t goal)
{
    return astar(start, goal);
}

float Search::euclidean_heuristic(Arm& a, target_t goal)
{
    float x = a.get_ee_x();
    float y = a.get_ee_y();

    return sqrt(pow(goal.x-x, 2) + pow(goal.y-y, 2));
}

bool Search::is_in_goal(float ee_x, float ee_y, target_t goal)
{
    if(ee_x > (goal.x + goal.err_x) ||
       ee_x < (goal.x - goal.err_x))
    {
        return false;
    }
    if(ee_y > (goal.y + goal.err_y) ||
       ee_y < (goal.y - goal.err_y))
    {
        return false;
    }

    return true;
}

struct node
{
    pose state;
    float f_value;
    node* parent;
    action cause;

    bool operator < (const node& other) const
    {return f_value > other.f_value; }
    bool operator > (const node& other) const
    {return f_value < other.f_value; }
};

plan Search::astar(Arm start, target_t target)
{
    // Possible actions to take from each state
    std::set<action> primitives;
    for (int i = 0; i < start.get_num_joints(); i++)
    {
        primitives.insert(action(i, 10));
        primitives.insert(action(i, -10));
    }

    // Cost function g(s)
    std::map<pose, float> costs;

    // g(s) = 0
    node start_state;
    start_state.state = start.get_joints();
    costs[start_state.state] = 0.f;

    // OPEN = empty
    std::priority_queue<node> OPEN;

    plan p;
    return p;
}
