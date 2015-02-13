#include "Search.h"
#include <math.h>
#include <queue>
#include <map>
#include <iostream>

Search* Search::instance = 0;

Search* Search::the_instance()
{
    if (!instance) instance = new Search();
    return instance;
}

// plan Search::run_search(Arm start, target_t goal)
// {
//     return astar(start, goal);
// }

// float Search::euclidean_heuristic(Arm& a, target_t goal)
// {
//     float x = a.get_ee_x();
//     float y = a.get_ee_y();

//     euclidean_heuristic(x, y, goal);
// }

// float Search::euclidean_heuristic(float x, float y, target_t goal)
// {
//     return sqrt(pow(goal.x-x, 2) + pow(goal.y-y, 2));
// }

// bool Search::is_in_goal(float ee_x, float ee_y, target_t goal)
// {
//     if(ee_x > (goal.x + goal.err_x) ||
//        ee_x < (goal.x - goal.err_x))
//     {
//         return false;
//     }
//     if(ee_y > (goal.y + goal.err_y) ||
//        ee_y < (goal.y - goal.err_y))
//     {
//         return false;
//     }

//     return true;
// }

// float Search::cost(Arm& start, pose from, pose to)
// {
//     float dx = start.get_ee_x_at(to) - start.get_ee_x_at(from);
//     float dy = start.get_ee_y_at(to) - start.get_ee_y_at(from);
//     return sqrt(pow(dx, 2) + pow(dy, 2));
// }

// struct node
// {
//     pose state;
//     float f_value;
//     plan path;

//     bool operator < (const node& other) const
//     {return f_value > other.f_value; }
//     bool operator > (const node& other) const
//     {return f_value < other.f_value; }
// };

// plan Search::astar(Arm start, target_t target, float epsilon)
// {
//     // Possible actions to take from each state
//     std::vector<action> primitives;
//     for (int i = 0; i < start.get_num_joints(); i++)
//     {
//         primitives.push_back(action(i, 10));
//         primitives.push_back(action(i, -10));
//     }

//     // Cost function g(s)
//     std::map<pose, float> costs;

//     // g(s) = 0
//     node start_state;
//     start_state.state = start.get_joints();
//     // f(start) = epsilon * heuristic(start)
//     start_state.f_value = (epsilon *
//                            euclidean_heuristic(start, target));

//     costs[start_state.state] = 0.f;

//     // OPEN = empty
//     std::priority_queue<node> OPEN;
//     // insert start into OPEN
//     OPEN.push(start_state);

//     node end;
//     end.f_value = -1;

//     while(true)
//     {
//         // remove s with smallest f-value from OPEN
//         node current(OPEN.top());
//         OPEN.pop();

//         if (is_in_goal(start.get_ee_x_at(current.state),
//                        start.get_ee_y_at(current.state),
//                        target))
//         {
//             end = current;
//             break;
//         }

//         pose next_pose;
//         // for each successor s' of s
//         for (std::vector<action>::iterator p = primitives.begin();
//              p != primitives.end(); p++)
//         {
//             next_pose = start.apply_at(*p, current.state);
//             if (!start.is_valid(next_pose)) continue;

//             float new_cost = (costs[current.state] +
//                               cost(start, current.state, next_pose));

//             // if s' not visited before (g(s') = inf)
//             // or g(s') > g(s) + c(s, s')
//             float cost = sqrt(start.get_ee_x_at(current.state));
//             if ( !costs.count(next_pose) ||
//                  costs[next_pose] >  new_cost)
//             {
//                 // g(s') = g(s) + c(s, s')
//                 costs[next_pose] = new_cost;
//                 // f(s') = g(s') + epsilon*heuristic(s')
//                 float h = euclidean_heuristic(start.get_ee_x_at(next_pose),
//                                               start.get_ee_y_at(next_pose),
//                                               target);
//                 node successor;
//                 successor.state = next_pose;
//                 successor.path = current.path;
//                 successor.path.push_back(*p);
//                 successor.f_value = (new_cost + epsilon*h);
//                 // insert s' into OPEN with above f(s')
//                 OPEN.push(successor);
//             }
//         }
//     }

//     return end.path;
// }

struct node
{
    box state;
    float f_value;
    maze_boxes path;

    bool operator < (const node& other) const
    {return f_value > other.f_value; }
    bool operator > (const node& other) const
    {return f_value < other.f_value; }
};

maze_solution Search::maze_astar(maze_boxes obs, float epsilon)
{
    maze_solution sol;

    // Cost function g(s)
    std::map<box, int> costs;

    // g(s) = 0
    node start_state;
    start_state.state = std::make_pair(0, 0);
    // f(start) = epsilon * heuristic(start)
    start_state.f_value = (epsilon *
                           maze_heuristic(start_state.state));

    costs[start_state.state] = 0;

    // OPEN = empty
    std::priority_queue<node> OPEN;
    // insert start into OPEN
    OPEN.push(start_state);

    node end;
    end.f_value = -1;

    while(true)
    {
        // remove s with smallest f-value from OPEN
        node current(OPEN.top());
        OPEN.pop();

        if (current.state.first == 5 && current.state.second == 6)
        {
            end = current;
            break;
        }
        sol.expanded.push_back(current.state);

        std::cout << "**Popped (" << current.state.first << ", "
                  << current.state.second << ") from pqueue.**"
                  << std::endl;

        box next;
        // for each successor s' of s
        for (int i = 1; i >= -1; i--)
        {
            for (int j = 1; j >= -1; j--)
            {
                if (i==0 && j==0) continue;
                next = std::make_pair(current.state.first + i,
                                      current.state.second + j);
                if (next.first < 0 || next.first > 5 ||
                    next.second < 0 || next.second > 6)
                    continue;
                if (obstacle(next, obs)) continue;

                int new_cost = costs[current.state] + 1;

                // if s' not visited before (g(s') = inf)
                // or g(s') > g(s) + c(s, s')
                if ( !costs.count(next) ||
                     costs[next] >  new_cost)
                {
                    // g(s') = g(s) + c(s, s')
                    costs[next] = new_cost;
                    // f(s') = g(s') + epsilon*heuristic(s')
                    int h = maze_heuristic(next);

                    node successor;
                    successor.state = next;
                    successor.path = current.path;
                    successor.path.push_back(next);
                    successor.f_value = (new_cost + epsilon*h);

                    std::cout << "\tAdded (" << next.first << ", "
                              << next.second << ") to pqueue with "
                              << "fval " << successor.f_value
                              << std::endl;

                    // insert s' into OPEN with above f(s')
                    OPEN.push(successor);
                }
            }
        }
    }

    sol.path = end.path;
    return sol;
}

int Search::maze_heuristic(box cell)
{
    int xdist = 5-cell.first;
    int ydist = 6-cell.second;
    if (ydist > xdist) return ydist;
    return xdist;
}

bool Search::obstacle(box cell, maze_boxes obs)
{
    for (maze_boxes::iterator o = obs.begin(); o != obs.end();
        o++)
    {
        if (o->first == cell.first && o->second == cell.second)
            return true;
    }
    return false;
}
