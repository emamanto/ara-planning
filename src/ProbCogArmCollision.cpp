#include "ProbCogArmCollision.h"

template<>
bool subdivision_collision_check<arm_state, action>(arm_state start,
                                                    action action,
                                                    int depth)
{
    std::cout << "Specialized" << std::endl;
    return false;
}

