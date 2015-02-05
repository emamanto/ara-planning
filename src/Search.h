#pragma once

#include <set>
#include "Arm.h"

// Rectangle centered at x, y with error err_x, err_y in each
// dimension
struct target_t
{
    float x;
    float y;
    float err_x;
    float err_y;

    target_t(float x, float y, float ex, float ey): x(x),
                                                    y(y),
                                                    err_x(ex),
                                                    err_y(ey) {}
};

class Search
{
public:
    static Search* the_instance();

    plan run_search(Arm start, target_t goal);
    float euclidean_heuristic(Arm& a, target_t goal);

private:
    Search() {};
    Search(Search const&) {};
    Search& operator=(Search const&) {};

    bool is_in_goal(float ee_x, float ee_y, target_t goal);
    plan astar(Arm start, target_t target);

    static Search* instance;

    std::set<action> primitives;
};
