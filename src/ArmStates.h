#pragma once

#include "Arm.h"
#include <map>
#include <queue>
#include <set>

#define D_SMALL 30.f
#define D_IK 15.f

class target
{
public:
    static target* the_instance();

    float x, y;

private:
    static target* instance;
    target() : x(0), y(0) {};
    target(target const&) {};
};

class obstacles
{
public:
    static obstacles* the_instance();
    std::vector<obstacle> get_obstacles() { return obs; }
    obstacle obstacle_num(int i) { return obs.at(i); }
    int num_obstacles() { return obs.size(); }
    void init(std::vector<obstacle> i) { obs = i; }

private:
    static obstacles* instance;
    obstacles() {};
    obstacles(obstacles const&) {};

    std::vector<obstacle> obs;
};

class arm_state
{
public:
    pose position;

    arm_state();
    arm_state(pose p);

    bool operator == (const arm_state& other) const;
    bool operator < (const arm_state& other) const;
    bool operator > (const arm_state& other) const;

    arm_state apply(action a);
    float cost(action a);
    bool valid() const;
    bool small_steps() const;
    bool use_finisher() const;
    action compute_finisher() const;
    bool is_goal() const;
    float heuristic() const;
    float target_distance() const;
    void print() const;

    static void new_goal(float x, float y);
    static void bfs(std::pair<int,int> cell);
    static std::pair<int,int> make_cell(float x, float y);

private:
    static std::map<std::pair<int,int>, int> bfs_heuristics;
    static std::queue<std::pair<int,int> > bfs_queue;
    static std::set<std::pair<int,int> > bfs_expanded;
    static int grid_size;
};
