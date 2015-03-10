#pragma once

#include <vector>
#include <utility>
#include <map>
#include <queue>
#include <set>
#include <iostream>

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

template <class S, typename P>
class Search
{
public:
    Search() {};

    search_result<S, P> astar(S begin,
                              std::vector<P> large_prs,
                              std::vector<P> small_prs,
                              float eps = 1.f)
    {
        big_primitives = large_prs;
        small_primitives = small_prs;
        epsilon = eps;
        OPEN = std::priority_queue<search_node<S, P> >();
        costs.clear();
        search_result<S, P> sol;

        // g(s) = 0
        search_node<S, P> snode;
        snode.state = begin;
        costs[begin] = 0;

        // f(start) = epsilon * heuristic(start)
        snode.f_value = fvalue(begin);

        // insert start into OPEN
        OPEN.push(snode);

        std::vector<P> final_path;
        std::vector<P>* primitives;

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
                    nnode.f_value = fvalue(child);

                    // insert s' into OPEN with above f(s')
                    OPEN.push(nnode);
                }
            }
        }

        return sol;
    }

    std::vector<search_result<S, P> > arastar(S begin,
                                              std::vector<P> big_prs,
                                              std::vector<P> s_prs,
                                              float e_start = 5.f)
    {
        big_primitives = big_prs;
        small_primitives = s_prs;
        goal_found = false;
        OPEN = std::priority_queue<search_node<S, P> >();
        CLOSED.clear();
        INCONS.clear();

        epsilon = e_start;

        // g(goal) = inf; g(start) = 0;
        costs[begin] = 0;

        search_node<S, P> snode;
        snode.state = begin;
        snode.f_value = fvalue(begin);
        OPEN.push(snode);

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

private:
    Search(Search const&) {};
    Search& operator=(Search const&) {};

    virtual float heuristic(S s) { return s.heuristic(); }
    virtual float fvalue(S s) { return (costs[s] +
                                        epsilon*heuristic(s));}

    void improve_path()
    {
        search_result<S, P> result;
        result.path = best_path;
        std::vector<P>* primitives;

        while(!goal_found || fvalue(goal) > OPEN.top().f_value)
        {
            if (OPEN.empty())
            {
                result.path.clear();
                solutions.push_back(result);
                std::cout << "No solution found by improve_path"
                          << std::endl;
                return;
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
                    nnode.f_value = fvalue(child);

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
        solutions.push_back(result);
    }

    float min_gh()
    {
        // Min over s in OPEN, INCONS g(s) + h(s)
        float min_g_plus_h = (costs[OPEN.top().state] +
                              heuristic(OPEN.top().state));
        std::priority_queue<search_node<S, P> > OPEN_copy(OPEN);
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

    // These things should be cleared out every
    // search round.
    bool goal_found;
    S goal;
    std::vector<P> big_primitives;
    std::vector<P> small_primitives;
    std::vector<search_result<S, P> >solutions;
    std::vector<P> best_path;
    std::set<S> CLOSED;
    std::set<search_node<S, P> > INCONS;
    std::priority_queue<search_node<S, P> > OPEN;
    std::map<S, float> costs;
    float epsilon;
};
