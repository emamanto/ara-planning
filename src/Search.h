#pragma once

#include <vector>
#include <utility>
#include <map>
#include <queue>
#include <set>
#include <iostream>
#include <unistd.h>
#include <ctime>

#define FIRST_SOL
//#define MAZE_FIGURE
#ifdef MAZE_FIGURE
#undef FIRST_SOL
#endif

template <typename S, typename P>
class search_node
{
public:
    search_node() {}
    search_node(S s) : state(s) {}

    bool operator < (const search_node<S, P>& other) const
    {return f_value > other.f_value; }
    bool operator > (const search_node<S, P>& other) const
    {return f_value < other.f_value; }

    S state;
    float f_value;
    std::vector<P> path;
};

template <typename S, typename P>
class search_result
{
public:
    std::vector<P> path;
    std::set<S> expanded;
    std::set<S> inconsistent;
};

template <class S>
float heuristic(S s) { return s.heuristic(); }

template <class S, typename P>
search_result<S, P> astar(S begin,
                          std::vector<P> big_primitives,
                          std::vector<P> small_primitives,
                          float epsilon = 1.f)
{
    std::priority_queue<search_node<S, P> > OPEN =
        std::priority_queue<search_node<S, P> >();
    std::map<S, float> costs = std::map<S, float>();
    search_result<S, P> sol;

    // g(s) = 0
    search_node<S, P> snode;
    snode.state = begin;
    costs[begin] = 0;

    // f(start) = epsilon * heuristic(start)
    snode.f_value = costs[begin] + epsilon*heuristic(begin);

    // insert start into OPEN
    OPEN.push(snode);

    std::vector<P> final_path;
    std::vector<P>* primitives;
    bool extra_prim = false;

    while(true)
    {
        if (OPEN.empty())
        {
            sol.path.clear();
            std::cout << "No solution found by astar"
                      << std::endl;
            return sol;
        }

        // remove s with smallest f-value from OPEN
        search_node<S, P> s(OPEN.top());
        OPEN.pop();

        if (s.state.is_goal())
        {
            sol.path = s.path;
            break;
        }

        sol.expanded.insert(s.state);

        if (s.state.small_steps())
        {
            primitives = &small_primitives;
            if (s.state.use_finisher())
            {
                primitives->push_back(s.state.compute_finisher());
                extra_prim = true;
            }
        }
        else
        {
            primitives = &big_primitives;
        }

        for (typename std::vector<P>::iterator p =
                 primitives->begin();
             p != primitives->end(); p++)
        {
            S child = s.state.apply(*p);
            if (!child.valid()) continue;
            float new_cost = costs[s.state] + s.state.cost(*p);

            // if s' not visited before (g(s') = inf)
            // or g(s') > g(s) + c(s, s')
            if ( !costs.count(child) ||
                 costs[child] > new_cost)
            {
                // g(s') = g(s) + c(s, s')
                costs[child] = new_cost;

                search_node<S, P> nnode;
                nnode.state = child;
                nnode.path = s.path;
                nnode.path.push_back(*p);
                nnode.f_value = (costs[child] +
                                 epsilon*heuristic(child));

                // insert s' into OPEN with above f(s')
                OPEN.push(nnode);
            }
        }
        if (extra_prim)
        {
            primitives->pop_back();
            extra_prim = false;
        }
    }

    return sol;
}

template <class S, typename P>
bool improve_path(S& goal,
                  bool goal_found,
                  bool* dead,
                  std::vector<P>& big_primitives,
                  std::vector<P>& small_primitives,
                  std::vector<search_result<S, P> >* solutions,
                  std::vector<P>& best_path,
                  std::set<search_node<S, P> >& INCONS,
                  std::priority_queue<search_node<S, P> >& OPEN,
                  std::map<S, float>& costs,
                  float epsilon,
                  bool* kill)
{
    std::set<S> CLOSED = std::set<S>();

    search_result<S, P> result;
    result.path = best_path;
    std::vector<P>* primitives;
    bool extra_prim = false;

    bool can_kill = !solutions->empty();

    while(!goal_found ||
          (costs[goal] + epsilon*heuristic(goal)) >
          OPEN.top().f_value)
    {
        if (can_kill && *kill)
        {
            *dead = true;
            return false;
        }

        if (OPEN.empty())
        {
            result.path.clear();
            solutions->push_back(result);
            std::cout << "No solution found by improve_path"
                      << std::endl;
            return false;
        }
        // remove s with smallest fvalue from OPEN
        search_node<S, P> s(OPEN.top());
        OPEN.pop();
        result.expanded.insert(s.state);

        // CLOSED = CLOSED U s
        CLOSED.insert(s.state);

        if (s.state.small_steps())
        {
            primitives = &small_primitives;
            if (s.state.use_finisher())
            {
                primitives->push_back(s.state.compute_finisher());
                extra_prim = true;
            }
        }
        else
        {
            primitives = &big_primitives;
        }

        for (typename std::vector<P>::iterator p = primitives->begin();
             p != primitives->end(); p++)
        {
            S child = s.state.apply(*p);
            if (!child.valid()) continue;
            float new_cost = costs[s.state] + s.state.cost(*p);

            // if s' not visited before (g(s') = inf)
            // or g(s') > g(s) + c(s, s')
            if ( !costs.count(child) ||
                 costs[child] > new_cost)
            {
                // g(s') = g(s) + c(s, s')
                costs[child] = new_cost;

                search_node<S, P> nnode;
                nnode.state = child;
                nnode.path = s.path;
                nnode.path.push_back(*p);
                nnode.f_value = (costs[child] +
                                 epsilon*heuristic(child));

                if (nnode.state.is_goal())
                {
                    result.path = nnode.path;
                    goal = nnode.state;
                    goal_found = true;
                }

                if (!CLOSED.count(child))
                {
                    // f(s') = g(s') + epsilon*heuristic(s')
                    OPEN.push(nnode);
                }
                else
                {
                    INCONS.insert(nnode);
                }
            }
        }
        if (extra_prim)
        {
            primitives->pop_back();
            extra_prim = false;
        }
    }

    if (result.path != best_path)
    {
        best_path = result.path;
    }

    for (typename std::set<search_node<S, P> >::iterator i = INCONS.begin();
         i != INCONS.end(); i++)
    {
        result.inconsistent.insert(i->state);
    }

    std::priority_queue<search_node<S, P> > cop(OPEN);
    while(!cop.empty())
    {
        result.inconsistent.insert(cop.top().state);
        cop.pop();
    }
    solutions->push_back(result);
    return goal_found;
}

template <class S, typename P>
float min_gh(std::map<S, float>& costs,
             std::priority_queue<search_node<S, P> > OPEN_copy,
             std::set<search_node<S, P> > INCONS)
{
    // Min over s in OPEN, INCONS g(s) + h(s)
    float min_g_plus_h = (costs[OPEN_copy.top().state] +
                          heuristic(OPEN_copy.top().state));
    while (!OPEN_copy.empty())
    {
        OPEN_copy.pop();
        float g_h = (costs[OPEN_copy.top().state] +
                     heuristic(OPEN_copy.top().state));
        if ( g_h < min_g_plus_h)
        {
            min_g_plus_h = g_h;
        }
    }
    for (typename std::set<search_node<S, P> >::iterator i = INCONS.begin();
         i != INCONS.end(); i++)
    {
        float g_h = (costs[i->state] + heuristic(i->state));
        if ( g_h < min_g_plus_h)
        {
            min_g_plus_h = g_h;
        }
    }
    return min_g_plus_h;
}

template <typename S, typename P>
void arastar(std::vector<search_result<S, P> >* solutions,
             bool* kill,
             S begin,
             std::vector<P> big_prs,
             std::vector<P> s_prs,
             float e_start = 5.f)
{
    std::clock_t t;
    t = std::clock();
    bool goal_found = false;
    std::priority_queue<search_node<S, P> > OPEN =
        std::priority_queue<search_node<S, P> >();
    solutions->clear();
    std::vector<P> best_path = std::vector<P>();
    std::set<search_node<S, P> > INCONS =
        std::set<search_node<S, P> >();
    std::map<S, float> costs = std::map<S, float>();
    float epsilon = e_start;
    S goal;

    // g(goal) = inf; g(start) = 0;
    costs[begin] = 0;

    search_node<S, P> snode;
    snode.state = begin;
    snode.f_value = (costs[begin] + epsilon*heuristic(begin));
    OPEN.push(snode);

    bool killed = false;
    goal_found = improve_path<S, P>(goal, goal_found, &killed,
                                    big_prs, s_prs,
                                    solutions, best_path,
                                    INCONS, OPEN,
                                    costs, epsilon, kill);
    if (killed) return;
    if (OPEN.empty()) return;
    best_path = solutions->at(0).path;

    // g(goal)/min{s E OPEN U INCONS} (g+h)
    float alt = costs[goal] / min_gh<S, P>(costs, OPEN, INCONS);

    t = std::clock() - t;

    float e_prime = epsilon;
    if (alt < e_prime) e_prime = alt;
    std::cout << "The first solution is suboptimal by: " <<
        e_prime << ", ";
    std::cout << solutions->at(0).expanded.size()
              << " expansions, ";
    std::cout << ((float)t) / CLOCKS_PER_SEC << " s"  << std::endl;
#ifdef FIRST_SOL
    return;
#endif

    while (e_prime > 1.f)
    {
        sleep(0.001);

        // decrease epsilon
#ifdef MAZE_FIGURE
        epsilon = epsilon - 0.5f;
#else
        epsilon = epsilon - 1.f;
#endif

        // Move states from INCONS to OPEN
        for (typename std::set<search_node<S, P> >::iterator i = INCONS.begin();
             i != INCONS.end(); i++)
        {
            OPEN.push(*i);
        }
        INCONS.clear();

        // Update priorities of all states in OPEN
        std::vector<search_node<S, P> > OPEN_update;
        while (!OPEN.empty())
        {
            OPEN_update.push_back(OPEN.top());
            OPEN.pop();
        }
        for (typename std::vector<search_node<S, P> >::iterator o = OPEN_update.begin();
             o != OPEN_update.end(); o++)
        {
            o->f_value = (costs[o->state] +
                          epsilon*heuristic(o->state));
            OPEN.push(*o);
        }

        goal_found = improve_path<S,P>(goal, goal_found, &killed,
                                       big_prs, s_prs,
                                       solutions, best_path,
                                       INCONS, OPEN,
                                       costs, epsilon, kill);
        if (killed) return;
        if (OPEN.empty()) return;

        // g(goal)/min{s E OPEN U INCONS} (g+h)
        float alt = costs[goal] / min_gh<S,P>(costs, OPEN, INCONS);

        e_prime = epsilon;
        if (alt < e_prime) e_prime = alt;
        std::cout << "Epsilon is: " << epsilon <<
            ", Solution is suboptimal by: " << e_prime << ", ";
        std::cout << solutions->at(solutions->size()-1).expanded.size()
                  << " expansions" << std::endl;

    }
}


