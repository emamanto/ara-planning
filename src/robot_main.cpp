#include <lcm/lcm-cpp.hpp>
#include <iostream>

#include "ProbCogArm.h"
#include "dynamixel_status_list_t.hpp"

class lcm_handler
{
public:
    ~lcm_handler() {};

    void handle_message(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel,
                        const dynamixel_status_list_t* stats)
    {
        std::cout << "Recieved message" << std::endl;
    }
};

int main(int argc, char* argv[])
{
    lcm::LCM lcm;
    if (!lcm.good())
    {
        std::cout << "Failed to initialize LCM." << std::endl;
        return 1;
    }

    lcm_handler handler;
    lcm.subscribe("ARM_STATUS", &lcm_handler::handle_message,
                  &handler);

    while(0 == lcm.handle());

    return 0;
}
