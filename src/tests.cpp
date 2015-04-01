#include "ArmStates.h"
#include "Search.h"
#include <iostream>

int main(int argc, char* argv[])
{
    pose start;
    start.push_back(28.f);
    start.push_back(7.f);
    start.push_back(9.f);
    start.push_back(71.f);
    start.push_back(42.f);

    std::vector<obstacle> the_obstacles;
    the_obstacles.push_back(obstacle(-10, 280, 20, 180));
    obstacles::the_instance()->init(the_obstacles);

    arm_state::use_euclidean(false);
    std::cout << "BFS HEURISTIC GOOD VERSION" << std::endl;
    for (int x = -60; x < -21; x+=10)
    {
        for (int y = 120; y < 191; y+=10)
        {
            std::cout << "Target " << x << ", " << y << std::endl;
            std::vector<search_result<arm_state, action> > latest_search;
            bool kill_search = false;
            target::the_instance()->x = x;
            target::the_instance()->y = y;
            arm_state::new_goal(x, y);
            arastar<arm_state, action>(&latest_search,
                                       &kill_search,
                                       arm_state(start),
                                       Arm::the_instance()->get_big_primitives(),
                                       Arm::the_instance()->get_small_primitives(),
                                       100.f);
        }
    }

    arm_state::use_euclidean(true);
    std::cout << "EUCLIDEAN COMPARISON" << std::endl;
    for (int x = -60; x < -21; x+=10)
    {
        for (int y = 120; y < 191; y+=10)
        {
            std::cout << "Target " << x << ", " << y << std::endl;
            std::vector<search_result<arm_state, action> > latest_search;
            bool kill_search = false;
            target::the_instance()->x = x;
            target::the_instance()->y = y;
            arastar<arm_state, action>(&latest_search,
                                       &kill_search,
                                       arm_state(start),
                                       Arm::the_instance()->get_big_primitives(),
                                       Arm::the_instance()->get_small_primitives(),
                                       100.f);
        }
    }

    arm_state::use_euclidean(false);
    std::cout << "BFS HEURISTIC BAD VERSION" << std::endl;
    for (int x = -60; x < -21; x+=10)
    {
        for (int y = 200; y < 251; y+=10)
        {
            std::cout << "Target " << x << ", " << y << std::endl;
            std::vector<search_result<arm_state, action> > latest_search;
            bool kill_search = false;
            target::the_instance()->x = x;
            target::the_instance()->y = y;
            arm_state::new_goal(x, y);
            arastar<arm_state, action>(&latest_search,
                                       &kill_search,
                                       arm_state(start),
                                       Arm::the_instance()->get_big_primitives(),
                                       Arm::the_instance()->get_small_primitives(),
                                       100.f);
        }
    }

    arm_state::use_euclidean(true);
    std::cout << "EUCLIDEAN COMPARISON" << std::endl;
    for (int x = -60; x < -21; x+=10)
    {
        for (int y = 200; y < 251; y+=10)
        {
            std::cout << "Target " << x << ", " << y << std::endl;
            std::vector<search_result<arm_state, action> > latest_search;
            bool kill_search = false;
            target::the_instance()->x = x;
            target::the_instance()->y = y;
            arastar<arm_state, action>(&latest_search,
                                       &kill_search,
                                       arm_state(start),
                                       Arm::the_instance()->get_big_primitives(),
                                       Arm::the_instance()->get_small_primitives(),
                                       100.f);
        }
    }

}
