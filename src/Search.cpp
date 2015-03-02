#include "Search.h"
#include <math.h>
#include <iostream>

Search* Search::instance = 0;

Search* Search::the_instance()
{
    if (!instance) instance = new Search();
    return instance;
}

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
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
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
                     costs[next] > new_cost)
                {
                    // g(s') = g(s) + c(s, s')
                    costs[next] = new_cost;
                    // f(s') = g(s') + epsilon*heuristic(s')
                    int h = maze_heuristic(next);

                    node successor;
                    successor.state = next;
                    successor.path = current.path;
                    successor.path.push_back(next);
                    successor.f_value = new_cost + epsilon*h;

                    std::cout << "\tAdded (" << next.first << ", "
                              << next.second << ") to pqueue with "
                              << "fval " << successor.f_value
                              << ", updated cost " << new_cost
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

arastar_solution Search::maze_arastar(maze_boxes obs, float e_start)
{
    ARAStarUnifier aras(obs, e_start);
    return aras.run();
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

arastar_solution Search::ARAStarUnifier::run()
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

void Search::ARAStarUnifier::improve_path()
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

float Search::ARAStarUnifier::fvalue(box state)
{
    return (costs[state] +
            epsilon*(Search::the_instance()->maze_heuristic(state)));
}
