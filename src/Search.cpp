#include "Search.h"
#include <math.h>
#include <iostream>

arastar_solution Search::maze_arastar(maze_boxes obs, float e_start)
{
    OPEN = std::priority_queue<node>();
    CLOSED.clear();
    INCONS.clear();
    obstacles = obs;
    epsilon = e_start;
    box start = std::make_pair(0, 0);

    // g(goal) = inf; g(start) = 0;
    costs[goal] = -1;
    costs[start] = 0;

    node start_node;
    start_node.state = start;
    start_node.f_value = maze_heuristic(start);
    OPEN.push(start_node);

    improve_path();
    best_path = solutions.at(0).path;

    // g(goal)/min{s E OPEN U INCONS} (g+h)
    float alt = costs[goal] / min_gh();

    float e_prime = epsilon;
    if (alt < e_prime) e_prime = alt;
    std::cout << "The first solution is suboptimal by: " <<
        e_prime << std::endl;

    while (e_prime > 1.f)
    {
        // decrease epsilon
        epsilon = epsilon - 0.5;

        // Move states from INCONS to OPEN
        for (std::set<node>::iterator i = INCONS.begin();
         i != INCONS.end(); i++)
        {
            OPEN.push(*i);
        }
        INCONS.clear();

        // Update priorities of all states in OPEN
        std::vector<node> OPEN_update;
        while (!OPEN.empty())
        {
            OPEN_update.push_back(OPEN.top());
            OPEN.pop();
        }
        for (std::vector<node>::iterator o = OPEN_update.begin();
             o != OPEN_update.end(); o++)
        {
            o->f_value = fvalue(o->state);
            OPEN.push(*o);
        }

        // Closed = empty
        CLOSED.clear();

        improve_path();

        // g(goal)/min{s E OPEN U INCONS} (g+h)
        float alt = costs[goal] / min_gh();

        e_prime = epsilon;
        if (alt < e_prime) e_prime = alt;
        std::cout << "Next solution is suboptimal by: " <<
            e_prime << std::endl;
    }

    epsilon = 1.f;
    return solutions;
}

int Search::maze_heuristic(box cell)
{
    int xdist = 5-cell.first;
    int ydist = 6-cell.second;
    if (ydist > xdist) return ydist;
    return xdist;
}

bool Search::obstacle(box cell)
{
    for (maze_boxes::iterator o = obstacles.begin();
         o != obstacles.end(); o++)
    {
        if (o->first == cell.first && o->second == cell.second)
            return true;
    }
    return false;
}

void Search::improve_path()
{
    maze_solution sol;
    sol.path = best_path;

    while(fvalue(goal) > OPEN.top().f_value || costs[goal] == -1)
    {
        // remove s with smallest fvalue from OPEN
        node s(OPEN.top());
        OPEN.pop();
        sol.expanded.push_back(s.state);

        // CLOSED = CLOSED U s
        CLOSED.insert(s.state);

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
                if (obstacle(s_prime)) continue;

                int new_cost = costs[s.state] + 1;

                // if s' not visited before (g(s') = inf)
                // or g(s') > g(s) + c(s, s')
                if ( !costs.count(s_prime) ||
                     costs[s_prime] > new_cost ||
                     costs[s_prime] == -1)
                {
                    costs[s_prime] = new_cost;
                    node snode;
                    snode.state = s_prime;
                    snode.path = s.path;
                    snode.path.push_back(s_prime);
                    snode.f_value = fvalue(s_prime);
                    if (s_prime.first == 5 &&
                        s_prime.second == 6)
                    {
                        sol.path = snode.path;
                    }

                    if (!CLOSED.count(s_prime))
                    {
                        // f(s') = g(s') + epsilon*heuristic(s')
                        OPEN.push(snode);
                    }
                    else
                    {
                        INCONS.insert(snode);
                    }
                }
            }
        }
    }

    if (sol.path != best_path)
    {
        best_path = sol.path;
    }

    for (std::set<node>::iterator i = INCONS.begin();
         i != INCONS.end(); i++)
    {
        sol.incons.push_back(i->state);
    }
    std::priority_queue<node> cop(OPEN);
    while(!cop.empty())
    {
        sol.incons.push_back(cop.top().state);
        cop.pop();
    }

    solutions.push_back(sol);
}

float Search::fvalue(box state)
{
    return (costs[state] + epsilon*(maze_heuristic(state)));
}

float Search::min_gh()
{
    // Min over s in OPEN, INCONS g(s) + h(s)
    float min_g_plus_h = (costs[OPEN.top().state] +
                          maze_heuristic(OPEN.top().state));
    std::priority_queue<node> OPEN_copy(OPEN);
    while (!OPEN_copy.empty())
    {
        OPEN_copy.pop();
        float g_h = (costs[OPEN_copy.top().state] +
                     maze_heuristic(OPEN_copy.top().state));
        if ( g_h < min_g_plus_h)
        {
            min_g_plus_h = g_h;
        }
    }
    for (std::set<node>::iterator i = INCONS.begin();
         i != INCONS.end(); i++)
    {
        float g_h = (costs[i->state] + maze_heuristic(i->state));
        if ( g_h < min_g_plus_h)
        {
            min_g_plus_h = g_h;
        }
    }
    return min_g_plus_h;
}
