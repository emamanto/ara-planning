#pragma once
#include <map>
#include <utility>

typedef std::pair<int, int> primitive;

class box
{
public:
    int x, y;

    box() : x(0), y(0) {}
    box(int x, int y) : x(x), y(y) {}

    bool operator == (const box& other) const;
    bool operator < (const box& other) const;
    bool operator > (const box& other) const;

    box apply(primitive p);
    float cost(primitive p);
    bool valid() const;
    bool is_goal() const;
    float heuristic();
    void print() const;
};
