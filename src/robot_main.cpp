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
    lcm_handler() : current_command(probcog_arm::get_num_joints(), 0) {};
    ~lcm_handler() {};

    void handle_status_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const dynamixel_status_list_t* stats)
    {
        if (searching) return;
        pose np;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            np.push_back(stats->statuses[i].position_radians);
        }
        if (np != status) status = np;

        bool done = true;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            if (fabs(status[i] - current_command[i]) > 0.01)
            {
                done = false;
                break;
            }
        }
        if (done && current_plan.size() > 0)
        {
            current_command_index++;
            current_command = current_plan[current_command_index];
        }

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
        searching = true;
        lcm::LCM lcm;
        point_3d goal;
        for (int i = 0; i < 3; i++)
        {
            goal.push_back(targ->target[i]);
        }
        arm_state::target = goal;
        std::vector<search_result<arm_state, action> > latest_search;
        bool kill_search = false;
        arastar<arm_state, action>(&latest_search,
                                   &kill_search,
                                   arm_state(status),
                                   probcog_arm::big_primitives(),
                                   probcog_arm::small_primitives(),
                                   100.f);
        current_plan = latest_search.at(latest_search.size()-1).path;
        current_command = current_plan.at(0);
        current_command_index = 0;
        searching = false;
    }

    pose status;
    pose current_command;
    int current_command_index;
    bool searching;
    std::vector<pose> current_plan;
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

    while(0 == lcm.handle());

    return 0;
}
