#include "ArmStates.h"
#include <iostream>
#include <cmath>
//#define EUCLIDEAN

target* target::instance = 0;

target* target::the_instance()
{
    if (!instance) instance = new target();
    return instance;
}

obstacles* obstacles::instance = 0;

obstacles* obstacles::the_instance()
{
    if (!instance) instance = new obstacles();
    return instance;
}

std::map<search_cell, int> arm_state::bfs_heuristics =
    std::map<search_cell, int>();
std::priority_queue<bfs_node> arm_state::bfs_queue =
    std::priority_queue<bfs_node>();
std::set<search_cell> arm_state::bfs_expanded =
    std::set<search_cell>();
int arm_state::grid_size = 10;

arm_state::arm_state() :
    position(Arm::the_instance()->get_num_joints(), 0)
{
}

arm_state::arm_state(pose p) : position(p)
{
}

bool arm_state::operator == (const arm_state& other) const
{
    return (position == other.position);
}

bool arm_state::operator < (const arm_state& other) const
{
    return (position < other.position);
}

bool arm_state::operator > (const arm_state& other) const
{
    return (position > other.position);
}

arm_state arm_state::apply(action a)
{
    Arm* arm = Arm::the_instance();
    pose next = arm->apply_at(a, position);
    return arm_state(next);
}

float arm_state::cost(action a)
{
    Arm* arm = Arm::the_instance();
    pose next = arm->apply_at(a, position);
    float dx = arm->get_ee_x_at(position) - arm->get_ee_x_at(next);
    float dy = arm->get_ee_y_at(position) - arm->get_ee_y_at(next);
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

bool arm_state::valid() const
{
    Arm* a = Arm::the_instance();
    if (!a->is_valid(position)) return false;

    std::vector<obstacle> obs = obstacles::the_instance()->get_obstacles();

    for (std::vector<obstacle>::iterator o = obs.begin();
         o != obs.end(); o++)
    {
        if (a->collision(*o, position)) return false;
    }

    return true;
}

bool arm_state::small_steps() const
{
    if (target_distance() < D_SMALL) return true;
    return false;
}

bool arm_state::use_finisher() const
{
    if (target_distance() < D_IK) return true;
    return false;
}

action arm_state::compute_finisher() const
{
    return Arm::the_instance()->solve_ik(target::the_instance()->x,
                                         target::the_instance()->y,
                                         position);
}

bool arm_state::is_goal() const
{
    if (target_distance() < 0.001) return true;
    else return false;
}

float arm_state::heuristic() const
{
#ifdef EUCLIDEAN
    return target_distance();
#else
    std::pair<int, int> grid_cell =
        make_cell(Arm::the_instance()->get_ee_x_at(position),
                  Arm::the_instance()->get_ee_y_at(position));
    if (bfs_heuristics.count(grid_cell))
    {
        return bfs_heuristics[grid_cell];
    }

    bfs(grid_cell);
    return bfs_heuristics[grid_cell];
#endif
}

void arm_state::bfs(search_cell end)
{
    while (!bfs_queue.empty())
    {
        bfs_node curr = bfs_queue.top();
        bfs_queue.pop();
        bfs_expanded.insert(curr.cell);
        if (curr.cell == end) return;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i==0 && j==0) continue;
                if ((curr.cell.second + j*grid_size) < 0) continue;
                if ( abs(curr.cell.first) > 300 ||
                     curr.cell.second > 300 ) continue;
                search_cell child =
                    std::make_pair(curr.cell.first + i*grid_size,
                                   curr.cell.second + j*grid_size);
                if (bfs_expanded.count(child)) continue;

                float potential;
                if (j==0 || i==0)
                {
                    potential = (bfs_heuristics[curr.cell] +
                                 grid_size);
                }
                else
                {
                    potential = (bfs_heuristics[curr.cell] +
                                 sqrt(2*pow(grid_size, 2)));
                }
                if (!bfs_heuristics.count(child) ||
                    bfs_heuristics[child] > potential)
                {
                    bfs_heuristics[child] = potential;
                    bfs_node c;
                    c.cell = child;
                    c.dist = potential;
                    bfs_queue.push(c);
                }
            }
        }
    }
}

float arm_state::target_distance() const
{
    return Arm::the_instance()->get_ee_dist_to(target::the_instance()->x,
                                               target::the_instance()->y,
                                               position);
}

void arm_state::print() const
{
    std::cout << "POSE: ";
    for (int i = 0; i < position.size(); i++)
    {
        std::cout << "joint " << i << " at " << position.at(i)
                  << " degrees ";
    }
    std::cout << std::endl;
}

search_cell arm_state::make_cell(float x, float y)
{
    int xf = int(floor(x));
    int yf = int(floor(y));
    return std::make_pair((xf - (xf % grid_size)),
                          (yf - (yf % grid_size)));
}

void arm_state::new_goal(float x, float y)
{
    arm_state::bfs_heuristics = std::map<search_cell, int>();
    arm_state::bfs_queue = std::priority_queue<bfs_node>();
    arm_state::bfs_expanded = std::set<search_cell>();
    bfs_node n;
    n.cell = make_cell(x, y);
    n.dist = 0;
    bfs_heuristics[n.cell] = 0;
    bfs_queue.push(n);
}
