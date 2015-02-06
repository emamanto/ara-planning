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

plan Search::astar(Arm start, target_t target)
{
    // Fake ASTAR
    plan p;
    p.push_back(action(0,10));
    p.push_back(action(1,-10));
    p.push_back(action(2,-10));
    return p;
}
