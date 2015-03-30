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

    void handle_status_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const dynamixel_status_list_t* stats)
    {
        pose np;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            np.push_back(stats->statuses[i].position_radians);
        }
        if (np != latest) latest = np;
    }

    void handle_target_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const search_target_t* targ)
    {
        lcm::LCM lcm;
        point_3d goal;
        for (int i = 0; i < 3; i++)
        {
            goal.push_back(targ->target[i]);
        }
        action a = probcog_arm::solve_ik(latest, goal);
        pose p;
        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = a.at(i) + latest.at(i);
            p.push_back(a.at(i) + latest.at(i));
            c.speed = probcog_arm::get_default_speed(i);
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
        }

        dynamixel_command_t hand;
        hand.position_radians = 0;
        hand.speed = 0;
        hand.max_torque = 0;
        command.commands.push_back(hand);

        int t = 0;
        while (t<10)
        {
        lcm.publish("ARM_COMMAND", &command);
        t++;
        sleep(1);
        }

    point_3d fxyz = probcog_arm::ee_xyz(p);
    std::cout << "**EE should be at " << fxyz[0] << ", "
    << fxyz[1] << ", " << fxyz[2] << std::endl;
    std::cout << "\t Because commanded pose is ";
    for (int i = 0; i < 5-1; i++)
    {
        std::cout << p.at(i) << ", ";
    }
    std::cout << p.at(5-1) << std::endl << std::endl;
    }

    pose latest;
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
    lcm.subscribe("ARM_STATUS", &lcm_handler::handle_status_message,
                  &handler);
    lcm.subscribe("SEARCH_TARGET", &lcm_handler::handle_target_message,
                  &handler);

        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = M_PI/6;
            c.speed = probcog_arm::get_default_speed(i);
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
        }

        dynamixel_command_t hand;
        hand.position_radians = 0;
        hand.speed = 0;
        hand.max_torque = 0;
        command.commands.push_back(hand);

        lcm.publish("ARM_COMMAND", &command);

    while(0 == lcm.handle())
    {
    }

return 0;
}
