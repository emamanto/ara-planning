#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "ProbCogSearchStates.h"
#include "Search.h"
//#include "RRTStarPlanner.h"
#include "dynamixel_status_list_t.hpp"
#include "dynamixel_command_list_t.hpp"
#include "search_target_t.hpp"

class lcm_handler
{
public:
    lcm_handler() :
        current_command(probcog_arm::get_num_joints(), 0),
        searching(false) {};
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
        if (done && current_plan.size() > 0 &&
            current_command_index < current_plan.size()-1)
        {
            current_command_index++;
            for (int i = 0; i < probcog_arm::get_num_joints(); i++)
            {
                current_command.at(i) +=
                    current_plan.at(current_command_index).at(i);
            }
        }
        else if (done && current_plan.size() > 0 &&
                 current_command_index == current_plan.size() - 1)
        {
            std::cout << "Arm fixing pitch!" << std::endl;
            current_command_index++;
            orientation o = probcog_arm::ee_rpy(current_command);
            float pitch_diff = o[1] - M_PI/2.f;
            if (pitch_diff < -M_PI/2.f)
            {
                pitch_diff = pitch_diff + M_PI;
            }
            else if (pitch_diff > M_PI/2.f)
            {
                pitch_diff = pitch_diff - M_PI;
            }
            current_command[3] += pitch_diff;
        }
        else if (done && current_plan.size() > 0 &&
                 current_command_index == current_plan.size())
        {
            std::cout << "Arm forward to grasp!" << std::endl;
            current_command_index++;
            point_3d ee = probcog_arm::ee_xyz(current_command);
            point_3d forwd = ee;
            if (ee[0] > 0) forwd[0] += 0.03;
            else forwd[0] -= 0.03;
            if (ee[1] > 0) forwd[1] += 0.03;
            else forwd[1] -= 0.03;
            action ik = probcog_arm::solve_ik(current_command, forwd);
            current_command = probcog_arm::apply(current_command, ik);
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
        if (done && current_plan.size() > 0 &&
            current_command_index > current_plan.size())
        {
            hand.position_radians = 112.f*DEG_TO_RAD;
            hand.speed = 0.15;
            hand.max_torque = 0.5;
        }
        else
        {
            hand.position_radians = 0;
            hand.speed = 0.15;
            hand.max_torque = 0.5;
        }
        command.commands.push_back(hand);

        lcm::LCM lcm;
        lcm.publish("ARM_COMMAND", &command);
    }

    void handle_target_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const search_target_t* targ)
    {
        searching = true;

        object_world world;
        point_3d obj;
        obj.push_back(0.2);
        obj.push_back(0.2);
        obj.push_back(0);
        world.object_xyz = obj;

        std::vector<float> dimensions;
        dimensions.push_back(0.05);
        dimensions.push_back(0.05);
        dimensions.push_back(0.4);
        world.obj_dim = dimensions;

        world.wall_x = 0.3;
        world.wall_y = 0.3;

        arm_state::world = world;

        lcm::LCM lcm;
        point_3d goal = world.grasp_point();
        for (int i = 0; i < 3; i++)
        {
            std::cout << goal[i] << std::endl;
//            goal.push_back(targ->target[i]);
        }

        std::cout << "About to search" << std::endl;
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
        current_command = status;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            current_command.at(i) += current_plan.at(0).at(i);
        }
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
