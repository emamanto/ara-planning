#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "ProbCogSearchStates.h"
#include "Search.h"
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
        arm_state as = arm_state(latest);
        action a(probcog_arm::get_num_joints(), 0.f);
        a[1] = 0.1;
        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = as.apply(a).position.at(i);
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

        arm_state::target[0] = 0.1;
        arm_state::target[1] = 0.3;
        arm_state::target[2] = 0.4;

        // std::vector<search_result<arm_state, action> >latest_search;
        // bool kill_search = false;
        // arastar<arm_state, action>(&latest_search,
        //                            &kill_search,
        //                            arm_state(pose(5, M_PI/6)),
        //                            probcog_arm::big_primitives(),
        //                            probcog_arm::small_primitives(),
        //                            10.f);

    while(0 == lcm.handle())
    {
    }

return 0;
}
