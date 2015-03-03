#include "MazeStates.h"
#include <cmath>
#include <iostream>
#include <utility>

bool box::operator == (const box& other) const
{
    if (x == other.x && y == other.y) return true;
    return false;
}

bool box::operator < (const box& other) const
{
    return std::make_pair(x, y) < std::make_pair(other.x, other.y);
}

bool box::operator > (const box& other) const
{
    return std::make_pair(x, y) > std::make_pair(other.x, other.y);
}

std::map<box, float> box::children()
{
    std::map<box, float> cs;
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (i == 0 && j == 0) continue;
            box c(x + i, y + j);
            cs[c] = 1.f;
        }
    }
    return cs;
}

bool box::valid() const
{
    if (x < 0 || x > 5 || y < 0 || y > 6) return false;

    if (x == 0 && y == 1) return false;
    if (x == 0 && y == 2) return false;
    if (x == 0 && y == 6) return false;

    if (x == 1 && y == 4) return false;
    if (x == 1 && y == 6) return false;

    if (x == 2 && y == 1) return false;
    if (x == 2 && y == 2) return false;
    if (x == 2 && y == 3) return false;
    if (x == 2 && y == 4) return false;
    if (x == 2 && y == 6) return false;

    if (x == 3 && y == 1) return false;
    if (x == 3 && y == 2) return false;
    if (x == 3 && y == 6) return false;

    if (x == 4 && y == 1) return false;
    if (x == 4 && y == 2) return false;
    if (x == 4 && y == 4) return false;
    if (x == 4 && y == 5) return false;
    if (x == 4 && y == 6) return false;

    return true;
}

bool box::is_goal() const
{
    if (x == 5 && y == 6) return true;
    return false;
}

float box::heuristic(box goal)
{
    int xdiff = std::abs(goal.x - x);
    int ydiff = std::abs(goal.y - y);

    if (xdiff > ydiff) return xdiff;
    return ydiff;
}

void box::print() const
{
    std::cout << "(" << x << ", " << y << ")" << std::endl;
}
