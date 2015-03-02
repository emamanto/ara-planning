#include "Search.h"
#include <math.h>
#include <iostream>

Search* Search::instance = 0;

Search* Search::the_instance()
{
    if (!instance) instance = new Search();
    return instance;
}

Search::Search() : goal(std::make_pair(5, 6)),
                   epsilon(1.f)
{
}

maze_solution Search::maze_astar(maze_boxes obs, float eps)
{
    epsilon = eps;
    OPEN = std::priority_queue<node>();
    maze_solution sol;

    // g(s) = 0
    box start = std::make_pair(0, 0);
    node snode;
    snode.state = start;
    costs[start] = 0;
    // f(start) = epsilon * heuristic(start)
    snode.f_value = fvalue(start);

    // insert start into OPEN
    OPEN.push(snode);

    maze_boxes final_path;

    while(true)
    {
        // remove s with smallest f-value from OPEN
        node s(OPEN.top());
        OPEN.pop();

        if (s.state.first == 5 && s.state.second == 6)
        {
            final_path = s.path;
            break;
        }
        sol.expanded.push_back(s.state);

        box s_prime;
        // for each successor s' of s
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i==0 && j==0) continue;
                s_prime = std::make_pair(s.state.first + i,
                                         s.state.second + j);
                if (s_prime.first < 0 || s_prime.first > 5 ||
                    s_prime.second < 0 || s_prime.second > 6)
                    continue;
                if (obstacle(s_prime, obs)) continue;

                int new_cost = costs[s.state] + 1;

                // if s' not visited before (g(s') = inf)
                // or g(s') > g(s) + c(s, s')
                if ( !costs.count(s_prime) ||
                     costs[s_prime] > new_cost)
                {
                    // g(s') = g(s) + c(s, s')
                    costs[s_prime] = new_cost;

                    node nnode;
                    nnode.state = s_prime;
                    nnode.path = s.path;
                    nnode.path.push_back(s_prime);
                    nnode.f_value = fvalue(s_prime);

                    // insert s' into OPEN with above f(s')
                    OPEN.push(nnode);
                }
            }
        }
    }

    sol.path = final_path;
    epsilon = 1.f;
    return sol;
}

arastar_solution Search::maze_arastar(maze_boxes obs, float e_start)
{
    box start = std::make_pair(0, 0);

    // g(goal) = inf; g(start) = 0;
    costs[goal] = -1;
    costs[start] = 0;

    // OPEN/CLOSED/INCONS already initialized
    node start_node;
    start_node.state = start;
    start_node.f_value =
        epsilon*Search::the_instance()->maze_heuristic(start);
    OPEN.push(start_node);

    improve_path();

    return solutions;
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

void Search::improve_path()
{
    maze_solution sol;
    maze_boxes path_to_goal;

    while(fvalue(goal) > OPEN.top().f_value || costs[goal] == -1)
    {
        std::cout << "Goal fval: " << fvalue(goal) <<
            " Min fval: " << OPEN.top().f_value << std::endl;

        // remove s with smallest fvalue from OPEN
        node s = OPEN.top();
        OPEN.pop();
        sol.expanded.push_back(s.state);

        // CLOSED = CLOSED U s
        CLOSED.insert(s.state);

        box next;
        // for each successor s' of s
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i==0 && j==0) continue;
                next = std::make_pair(s.state.first + i,
                                      s.state.second + j);
                if (next.first < 0 || next.first > 5 ||
                    next.second < 0 || next.second > 6)
                    continue;
                if (Search::the_instance()->obstacle(next, obstacles)) continue;

                int new_cost = costs[s.state] + 1;

                // if s' not visited before (g(s') = inf)
                // or g(s') > g(s) + c(s, s')
                if ( !costs.count(next) ||
                     costs[next] > new_cost ||
                     costs[next] == -1)
                {
                    costs[next] = new_cost;
                    if (!CLOSED.count(next))
                    {
                        // f(s') = g(s') + epsilon*heuristic(s')
                        int h = Search::the_instance()->maze_heuristic(next);
                        node successor;
                        successor.state = next;
                        successor.path = s.path;
                        successor.path.push_back(next);
                        successor.f_value = new_cost + epsilon*h;
                        if (next.first == 5 && next.second == 6)
                        {
                            path_to_goal = successor.path;
                        }
                        OPEN.push(successor);
                    }
                    else
                    {
                        INCONS.insert(next);
                    }
                }
            }
        }
    }

    sol.path = path_to_goal;
    std::cout << "Size of path: " << sol.path.size() << std::endl;
    std::cout << "Expanded: " << sol.expanded.size() << std::endl;
    solutions.push_back(sol);
}

float Search::fvalue(box state)
{
    return (costs[state] + epsilon*(maze_heuristic(state)));
}
