#include "Search.h"
#include <math.h>
#include <queue>
#include <map>
#include <iostream>

#define DEBUG

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

    euclidean_heuristic(x, y, goal);
}

float Search::euclidean_heuristic(float x, float y, target_t goal)
{
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

float Search::cost(Arm& start, pose from, pose to)
{
    float dx = start.get_ee_x_at(to) - start.get_ee_x_at(from);
    float dy = start.get_ee_y_at(to) - start.get_ee_y_at(from);
    return sqrt(pow(dx, 2) + pow(dy, 2));
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

plan Search::astar(Arm start, target_t target, float epsilon)
{
    // Possible actions to take from each state
    std::vector<action> primitives;
    for (int i = 0; i < start.get_num_joints(); i++)
    {
        primitives.push_back(action(i, 10));
        primitives.push_back(action(i, -10));
    }

#ifdef DEBUG
    std::cout << "THE POSSIBLE ACTIONS ARE:" << std::endl;
    for (std::vector<action>::iterator p = primitives.begin();
         p != primitives.end(); p++)
    {
        std::cout << "  joint " << p->joint << " by "
                  << p->change << std::endl;
    }
#endif

    // Cost function g(s)
    std::map<pose, float> costs;

    // g(s) = 0
    node start_state;
    start_state.parent = NULL;
    start_state.state = start.get_joints();
    // f(start) = epsilon * heuristic(start)
    start_state.f_value = (epsilon *
                           euclidean_heuristic(start, target));
    costs[start_state.state] = 0.f;

    // OPEN = empty
    std::priority_queue<node> OPEN;
    // insert start into OPEN
    OPEN.push(start_state);

    node end;
    end.f_value = -1;

    while(true)
    {
        // remove s with smallest f-value from OPEN
        node current = OPEN.top();
        OPEN.pop();
        if (is_in_goal(start.get_ee_x_at(current.state),
                       start.get_ee_y_at(current.state),
                       target))
        {
#ifdef DEBUG
            std::cout << "*Found a goal state!*" << std::endl;
#endif
            end = current;
            break;
        }

        // for each successor s' of s
        for (std::vector<action>::iterator p = primitives.begin();
             p != primitives.end(); p++)
        {
            pose next_pose = start.apply_at(*p, current.state);
            if (!start.is_valid(next_pose)) continue;

            node successor;
            successor.state = next_pose;
            float new_cost = (costs[current.state] +
                              cost(start, current.state, next_pose));

            // if s' not visited before (g(s') = inf)
            // or g(s') > g(s) + c(s, s')
            float cost = sqrt(start.get_ee_x_at(current.state));
            if ( !costs.count(next_pose) ||
                 costs[next_pose] >  new_cost)
            {
                // g(s') = g(s) + c(s, s')
                costs[next_pose] = new_cost;
                // f(s') = g(s') + epsilon*heuristic(s')
                float h = euclidean_heuristic(start.get_ee_x_at(next_pose),
                                              start.get_ee_y_at(next_pose),
                                              target);
                successor.f_value = (new_cost + epsilon*h);
                successor.parent = &current;
                successor.cause = *p;
                // insert s' into OPEN with above f(s')
                OPEN.push(successor);
            }
        }
    }

    plan p;
    if (end.f_value == -1) return p;

    node* path = &end;

#ifdef DEBUG
    std::cout << "The [reversed] path to the goal is:" << std::endl;
#endif

    while (path->parent)
    {
        p.push_back(path->cause);
#ifdef DEBUG
        std::cout << "  joint " << path->cause.joint << " by "
                  << path->cause.change << std::endl;
#endif

        path = path->parent;
    }

    return p;
}
