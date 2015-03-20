#include "ArmStates.h"
#include <iostream>
#include <cmath>

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

std::map<std::pair<int,int>, int> arm_state::bfs_heuristics = std::map<std::pair<int,int>, int>();
std::queue<std::pair<int,int> > arm_state::bfs_queue = std::queue<std::pair<int,int> >();
std::set<std::pair<int,int> > arm_state::bfs_expanded = std::set<std::pair<int,int> >();
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
    std::pair<int, int> grid_cell =
        make_cell(Arm::the_instance()->get_ee_x_at(position),
                  Arm::the_instance()->get_ee_y_at(position));
    if (bfs_heuristics.count(grid_cell))
    {
        return bfs_heuristics[grid_cell];
    }

    bfs(grid_cell);
    return bfs_heuristics[grid_cell];
}

void arm_state::bfs(std::pair<int,int> cell)
{
    while (!bfs_queue.empty())
    {
        std::pair<int, int> curr = bfs_queue.front();
        bfs_queue.pop();
        bfs_expanded.insert(curr);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i==0 && j==0) continue;
                std::pair<int, int> child =
                    std::make_pair(curr.first + i*grid_size,
                                   curr.second + j*grid_size);
                if (bfs_expanded.count(child)) continue;
                if (i==0 || j==0)
                {
                    bfs_heuristics[child] = (bfs_heuristics[curr] +
                                             grid_size);
                }
                else
                {
                    bfs_heuristics[child] = (bfs_heuristics[curr] +
                                             sqrt(2*pow(grid_size,
                                                        2)));
                }
                bfs_queue.push(child);
                if (child == cell) return;
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

std::pair<int,int> arm_state::make_cell(float x, float y)
{
    int xf = int(floor(x));
    int yf = int(floor(y));
    return std::make_pair((xf - (xf % grid_size))/grid_size,
                          (yf - (yf % grid_size))/grid_size);
}

void arm_state::new_goal(float x, float y)
{
    arm_state::bfs_heuristics = std::map<std::pair<int,int>, int>();
    arm_state::bfs_queue = std::queue<std::pair<int,int> >();
    bfs_queue.push(make_cell(x, y));
    bfs_heuristics[make_cell(x, y)] = 0;
}
