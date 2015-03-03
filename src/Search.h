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

            if (s.state == end)
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

    //std::vector<search_result<S> > arastar(search_state begin,
    //                                       search_state end,
    //                                       float e_start = 5.f);

private:
    Search(Search const&) {};
    Search& operator=(Search const&) {};

    virtual float heuristic(S s) { return s.heuristic(goal); }
    virtual float fvalue(S s) { return (costs[s] +
                                        epsilon*heuristic(s));}
    //void improve_path();
    //float min_gh();

    // These things should be cleared out every
    // search round.
    S goal;
    // std::vector<search_result<S> >solutions;
    // std::vector<S> best_path;
    // std::set<S> CLOSED;
    // std::set<search_node<S> > INCONS;
    std::priority_queue<search_node<S> > OPEN;
    std::map<S, float> costs;
    float epsilon;
};
