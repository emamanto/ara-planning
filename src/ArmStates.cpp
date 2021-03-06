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

void obstacles::init(std::vector<obstacle> i)
{
    obs = i;
    for (std::vector<obstacle>::iterator o = i.begin();
         o != i.end(); o++)
    {
        search_cell tl = arm_state::make_cell(o->x, o->y);
        blocked_cells.insert(tl);

        for (int x = tl.first; x <= (o->x + o->width); x+= grid_size)
        {
            for (int y = tl.second; y >= (o->y - o->height);
                 y -= grid_size)
            {
                blocked_cells.insert(arm_state::make_cell(x,y));
            }
        }
    }
}

bool obstacles::contains_obstacle(search_cell c)
{
    return blocked_cells.count(c);
}

std::map<search_cell, int> arm_state::bfs_heuristics =
    std::map<search_cell, int>();
std::map<search_cell, search_path> arm_state::bfs_paths =
    std::map<search_cell, search_path>();
std::priority_queue<bfs_node> arm_state::bfs_queue =
    std::priority_queue<bfs_node>();
std::set<search_cell> arm_state::bfs_expanded =
    std::set<search_cell>();

bool arm_state::euclidean = false;
bool arm_state::heuristic_debug = false;

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
    if (euclidean)
    {
        return target_distance();
    }
    else
    {
        std::pair<int, int> grid_cell =
            make_cell(Arm::the_instance()->get_ee_x_at(position),
                      Arm::the_instance()->get_ee_y_at(position));
        if (!bfs_heuristics.count(grid_cell))
        {
            bfs(grid_cell);
        }

        if (bfs_heuristics[grid_cell] < 2*grid_size)
        {
            return target_distance();
        }

        return bfs_heuristics[grid_cell];
    }
}

search_path arm_state::heuristic_path() const
{
    if (!heuristic_debug)
    {
        std::cout << "Heuristic debugging off." << std::endl;
        return search_path();
    }
    if (euclidean)
    {
        std::cout << "No path because using euclidean heuristic."
                  << std::endl;
        return search_path();
    }

    std::pair<int, int> grid_cell =
        make_cell(Arm::the_instance()->get_ee_x_at(position),
                  Arm::the_instance()->get_ee_y_at(position));

    if (bfs_paths.count(grid_cell))
    {
        return bfs_paths[grid_cell];
    }

    bfs(grid_cell);
    return bfs_paths[grid_cell];
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
                     curr.cell.second > 300 ||
                     curr.cell.second < 0) continue;
                search_cell child =
                    std::make_pair(curr.cell.first + i*grid_size,
                                   curr.cell.second + j*grid_size);
                if (obstacles::the_instance()->contains_obstacle(child)) continue;
                if (sqrt(pow(child.first,2) +
                         pow(child.second,2)) > ARM_LENGTH )
                {
                    continue;
                }

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
                    if (heuristic_debug)
                    {
                        bfs_paths[child] = bfs_paths[curr.cell];
                        bfs_paths[child].push_back(child);
                    }
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
    arm_state::bfs_paths = std::map<search_cell, search_path>();
    arm_state::bfs_queue = std::priority_queue<bfs_node>();
    arm_state::bfs_expanded = std::set<search_cell>();
    bfs_node n;
    n.cell = make_cell(x, y);
    n.dist = 0;
    bfs_heuristics[n.cell] = 0;
    if (heuristic_debug) bfs_paths[n.cell] = search_path();
    bfs_queue.push(n);
}
