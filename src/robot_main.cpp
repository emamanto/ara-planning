#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "ProbCogArm.h"
#include "dynamixel_status_list_t.hpp"
#include "dynamixel_command_list_t.hpp"
#include "search_target_t.hpp"

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

    probcog_arm::INIT();

    lcm_handler handler;
    lcm.subscribe("ARM_STATUS", &lcm_handler::handle_message,
                  &handler);
    lcm.subscribe("SEARCH_TARGET", &lcm_handler::handle_message,
                  &handler);

    while(0 == lcm.handle())
    {
        dynamixel_command_list_t command;
        pose p;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = -M_PI/(i+3);
            c.speed = probcog_arm::get_default_speed(i);
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
            p.push_back(-M_PI/(i+3));
        }

        dynamixel_command_t hand;
        hand.position_radians = 0;
        hand.speed = 0;
        hand.max_torque = 0;
        command.commands.push_back(hand);

        point_3d xyz = probcog_arm::ee_xyz(p);
        std::cout << xyz.at(0) << ", " << xyz.at(1) << ", "
                  << xyz.at(2) << std::endl;

        lcm.publish("ARM_COMMAND", &command);
        sleep(0.1);
    }

return 0;
}
