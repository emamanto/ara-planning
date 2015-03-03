#pragma once
#include <map>

class box
{
public:
    int x, y;

    box() : x(0), y(0) {}
    box(int x, int y) : x(x), y(y) {}

    bool operator == (const box& other) const;
    bool operator < (const box& other) const;
    bool operator > (const box& other) const;
    std::map<box, float> children();
    bool valid() const;
    bool is_goal() const;
    float heuristic(box goal);
    void print() const;
};
