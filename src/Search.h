#pragma once

#include <vector>
#include <utility>
#include <map>
#include <queue>
#include <set>
#include <iostream>

template <typename S>
class search_node
{
public:
    search_node() {}
    search_node(S s) : state(s) {}

    bool operator < (const search_node<S>& other) const
    {return f_value > other.f_value; }
    bool operator > (const search_node<S>& other) const
    {return f_value < other.f_value; }

    S state;
    float f_value;
    std::vector<S> path;
};

template <class S>
class search_result
{
public:
    std::vector<S> path;
    std::set<S> expanded;
    std::set<S> inconsistent;
};

template <class S>
class Search
{
public:
    Search() {};

    search_result<S> astar(S begin,
                           S end,
                           float eps = 1.f)
    {
        goal = end;
        epsilon = eps;
        OPEN = std::priority_queue<search_node<S> >();
        costs.clear();
        search_result<S> sol;

        // g(s) = 0
        search_node<S> snode;
        snode.state = begin;
        costs[begin] = 0;

        // f(start) = epsilon * heuristic(start)
        snode.f_value = fvalue(begin);

        // insert start into OPEN
        OPEN.push(snode);

        std::vector<S> final_path;

        while(true)
        {
            // remove s with smallest f-value from OPEN
            search_node<S> s(OPEN.top());
            OPEN.pop();

            if (s.state.is_goal())
            {
                sol.path = s.path;
                break;
            }

            sol.expanded.insert(s.state);

            std::map<S, float> next = s.state.children();

            for (typename std::map<S, float>::iterator i = next.begin();
                 i != next.end(); i++)
            {
                //i->first.print();
                if (!i->first.valid()) continue;
                float new_cost = costs[s.state] + i->second;

                // if s' not visited before (g(s') = inf)
                // or g(s') > g(s) + c(s, s')
                if ( !costs.count(i->first) ||
                     costs[i->first] > new_cost)
                {
                    // g(s') = g(s) + c(s, s')
                    costs[i->first] = new_cost;

                    search_node<S> nnode;
                    nnode.state = i->first;
                    nnode.path = s.path;
                    nnode.path.push_back(i->first);
                    nnode.f_value = fvalue(i->first);

                    // insert s' into OPEN with above f(s')
                    OPEN.push(nnode);
                }
            }
        }

        return sol;
    }

    std::vector<search_result<S> > arastar(S begin,
                                           S end,
                                           float e_start = 5.f)
    {
        goal = end;
        OPEN = std::priority_queue<search_node<S> >();
        CLOSED.clear();
        INCONS.clear();

        epsilon = e_start;

        // g(goal) = inf; g(start) = 0;
        costs[goal] = -1;
        costs[begin] = 0;

        search_node<S> snode;
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
            for (typename std::set<search_node<S> >::iterator i = INCONS.begin();
                 i != INCONS.end(); i++)
            {
                OPEN.push(*i);
            }
            INCONS.clear();

            // Update priorities of all states in OPEN
            std::vector<search_node<S> > OPEN_update;
            while (!OPEN.empty())
            {
                OPEN_update.push_back(OPEN.top());
                OPEN.pop();
            }
            for (typename std::vector<search_node<S> >::iterator o = OPEN_update.begin();
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
        search_result<S> result;
        result.path = best_path;

        while(fvalue(goal) > OPEN.top().f_value || costs[goal] == -1)
        {
            // remove s with smallest fvalue from OPEN
            search_node<S> s(OPEN.top());
            OPEN.pop();
            result.expanded.insert(s.state);

            // CLOSED = CLOSED U s
            CLOSED.insert(s.state);


            std::map<S, float> next = s.state.children();

            for (typename std::map<S, float>::iterator i = next.begin();
                 i != next.end(); i++)
            {
                //i->first.print();
                if (!i->first.valid()) continue;

                float new_cost = costs[s.state] + i->second;

                // if s' not visited before (g(s') = inf)
                // or g(s') > g(s) + c(s, s')
                if ( !costs.count(i->first) ||
                     costs[i->first] > new_cost ||
                     costs[i->first] == -1)
                {
                    // g(s') = g(s) + c(s, s')
                    costs[i->first] = new_cost;

                    search_node<S> nnode;
                    nnode.state = i->first;
                    nnode.path = s.path;
                    nnode.path.push_back(i->first);
                    nnode.f_value = fvalue(i->first);

                    if (nnode.state.is_goal())
                    {
                        result.path = nnode.path;
                    }

                    if (!CLOSED.count(i->first))
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

        for (typename std::set<search_node<S> >::iterator i = INCONS.begin();
             i != INCONS.end(); i++)
        {
            result.inconsistent.insert(i->state);
        }

        std::priority_queue<search_node<S> > cop(OPEN);
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
        std::priority_queue<search_node<S> > OPEN_copy(OPEN);
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
        for (typename std::set<search_node<S> >::iterator i = INCONS.begin();
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
    S goal;
    std::vector<search_result<S> >solutions;
    std::vector<S> best_path;
    std::set<S> CLOSED;
    std::set<search_node<S> > INCONS;
    std::priority_queue<search_node<S> > OPEN;
    std::map<S, float> costs;
    float epsilon;
};
