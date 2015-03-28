#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>

#include "ProbCogArm.h"
#include "dynamixel_status_list_t.hpp"
#include "dynamixel_command_list_t.hpp"

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

    while(0 == lcm.handle())
    {
        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints();
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = 0.1;
            c.speed = probcog_arm::get_default_speed(i);
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
        }

        lcm.publish("ARM_COMMANDS", &command);
        sleep(0.1);
    }

return 0;
}
