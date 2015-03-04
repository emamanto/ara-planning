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

box box::apply(primitive p)
{
    return box(x + p.first, y + p.second);
}

float box::cost(primitive p)
{
    return 1.f;
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

float box::heuristic()
{
    int xdiff = std::abs(5 - x);
    int ydiff = std::abs(6 - y);

    if (xdiff > ydiff) return xdiff;
    return ydiff;
}

void box::print() const
{
    std::cout << "(" << x << ", " << y << ")" << std::endl;
}
