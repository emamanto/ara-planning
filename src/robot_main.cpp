#include "ProbCogArm.h"
#include <lcm/lcm-cpp.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    lcm::LCM lcm;
    if (!lcm.good())
    {
        std::cout << "Failed to initialize LCM." << std::endl;
        return 1;
    }

    return 0;
}
