#include "Search.h"
#include <math.h>
#include <iostream>

arastar_solution Search::maze_arastar(maze_boxes obs, float e_start)
{
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
}

float Search::fvalue(box state)
{
    return (costs[state] + epsilon*(maze_heuristic(state)));
}

float Search::min_gh()
{
}
