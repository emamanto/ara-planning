#pragma once

#include "Arm.h"
#include <vector>
#include <utility>
#include <map>
#include <queue>
#include <set>


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
typedef std::pair<int, int> box;

struct maze_solution
{
    maze_boxes path;
    maze_boxes expanded;
};

typedef std::vector<maze_solution> arastar_solution;

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

class Search
{
public:
    static Search* the_instance();

//    plan run_search(Arm start, target_t goal);
//    float euclidean_heuristic(Arm& a, target_t goal);
//    float euclidean_heuristic(float x, float y, target_t goal);
    maze_solution maze_astar(maze_boxes obs, float epsilon = 1.f);
    arastar_solution maze_arastar(maze_boxes obs,
                                  float e_start = 5.f);

private:
    Search() {};
    Search(Search const&) {};
    Search& operator=(Search const&) {};

    int maze_heuristic(box cell);
    bool obstacle(box cell, maze_boxes obs);

//    bool is_in_goal(float ee_x, float ee_y, target_t goal);
//    float cost(Arm& start, pose from, pose to);
//    plan astar(Arm start, target_t target, float epsilon = 1.f);

    static Search* instance;

    class ARAStarUnifier
    {
    public:
        ARAStarUnifier(maze_boxes obs, float e_start) :
            obstacles(obs),
            epsilon(e_start) { goal = std::make_pair(5, 6); }
        arastar_solution run();

    private:
        void improve_path();
        float fvalue(box state);
        arastar_solution solutions;

        std::set<box> CLOSED;
        std::set<box> INCONS;
        std::priority_queue<node> OPEN;
        std::map<box, int> costs;
        maze_boxes obstacles;
        float epsilon;
        box goal;
    };
};
