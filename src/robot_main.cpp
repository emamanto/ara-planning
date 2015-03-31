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
    lcm_handler() : current_command(5,M_PI/6) {};
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
        if (np != status) status = np;

        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = current_command.at(i);
            c.speed = probcog_arm::get_default_speed(i);
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
        }

        dynamixel_command_t hand;
        hand.position_radians = 0;
        hand.speed = 0;
        hand.max_torque = 0;
        command.commands.push_back(hand);

        lcm::LCM lcm;
        lcm.publish("ARM_COMMAND", &command);
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

        current_command = probcog_arm::apply(status,
                                             probcog_arm::solve_ik(status, goal));
    }

    pose status;
    pose current_command;
};

int main(int argc, char* argv[])
{
    // lcm::LCM lcm;
    // if (!lcm.good())
    // {
    //     std::cout << "Failed to initialize LCM." << std::endl;
    //     return 1;
    // }

    probcog_arm::INIT();
    // lcm_handler handler;
    // lcm.subscribe("ARM_STATUS", &lcm_handler::handle_status_message,
    //               &handler);
    // lcm.subscribe("SEARCH_TARGET", &lcm_handler::handle_target_message,
    //               &handler);

    // while(0 == lcm.handle());

    pose p = pose(5, 0);
    point_3d target = probcog_arm::ee_xyz(p);
    std::cout << "Starting at: " << target[0] << ", " << target[1]
              << ", "  << target[2] << std::endl;
    target[1] += 0.05;
    target[2] -= 0.1;
    arm_state::target = target;

    std::cout << "Going to: " << target[0] << ", " << target[1]
              << ", "  << target[2] << std::endl;

    std::cout << "Using " << probcog_arm::big_primitives().size()
              << " big and " << probcog_arm::small_primitives().size()
              << " small" << std::endl;

    std::vector<search_result<arm_state, action> > res;
    bool kill_search = false;
    arastar<arm_state, action>(&res,
                               &kill_search,
                               arm_state(p),
                               probcog_arm::big_primitives(),
                               probcog_arm::small_primitives(),
                               100.f);

    return 0;
}
