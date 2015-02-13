#pragma once

#include "Arm.h"
#include <vector>
#include <utility>

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

typedef std::vector<std::pair<int, int> > maze_boxes;

struct maze_solution
{
    maze_boxes path;
    maze_boxes expanded;
};

class Search
{
public:
    static Search* the_instance();

//    plan run_search(Arm start, target_t goal);
//    float euclidean_heuristic(Arm& a, target_t goal);
//    float euclidean_heuristic(float x, float y, target_t goal);
    maze_solution maze_astar(maze_boxes obs, float epsilon = 1.f);

private:
    Search() {};
    Search(Search const&) {};
    Search& operator=(Search const&) {};

//    bool is_in_goal(float ee_x, float ee_y, target_t goal);
//    float cost(Arm& start, pose from, pose to);
//    plan astar(Arm start, target_t target, float epsilon = 1.f);

    static Search* instance;
};
