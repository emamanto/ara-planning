#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "ProbCogSearchStates.h"
#include "Search.h"
#include "ProbCogArmCollision.h"
#include "Shortcut.h"
#include "dynamixel_status_list_t.hpp"
#include "dynamixel_command_list_t.hpp"
#include "observations_t.hpp"

class experiment_handler
{
public:
    pose apos;
    pose dpos;
    float ahand;
    float dhand;
    std::vector<pose> current_plan;
    int plan_index;
    bool searching;
    int num_collisions;
    std::vector<object_data_t> latest_objects;

    int target_obj_id;
    char target_obj_color[2];
    float drop_x;
    float drop_y;

    bool picked_up;
    bool moved_drop;
    bool dropped_off;

    int observe_time;

    experiment_handler(int obj_id,
                       char color[2],
                       float drop_target_x,
                       float drop_target_y) :
        ahand(0),
        dhand(0),
        plan_index(0),
        searching(false),
        num_collisions(0),
        target_obj_id(obj_id),
        drop_x(drop_target_x),
        drop_y(drop_target_y),
        picked_up(false),
        moved_drop(false),
        dropped_off(false),
        observe_time(0)
    {
        target_obj_color[0] = color[0];
        target_obj_color[1] = color[1];

        dpos.push_back(M_PI/8);
        dpos.push_back(M_PI/2);
        dpos.push_back(-M_PI/2 + M_PI/8);
        dpos.push_back(M_PI/2);
        dpos.push_back(0);
        dpos.push_back(M_PI/2);
        dpos.push_back(0);
    };

    ~experiment_handler() {};

    void handle_status_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const dynamixel_status_list_t* stats)
    {
        if (searching) return;

        lcm::LCM lcm;

        // Update pose from status
        pose np;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            np.push_back(stats->statuses[i].position_radians);
        }
        if (np != apos)
        {
            apos = np;
        }

        ahand = stats->statuses[fetch_arm::get_num_joints()].position_radians;

        // Check collision status based on new pose
        if (collision_world::collision(apos, true))
        {
            if (num_collisions < collision_world::num_collisions())
            {
                int new_collisions = collision_world::num_collisions() - num_collisions;
                std::cout << "There are "
                          << new_collisions
                          << " new collisions. All current collisions: ";
                for (int i = 0;
                     i < collision_world::num_collisions(); i++)
                {
                    collision_pair pr = collision_world::get_collision_pair(i);
                    std::cout << pr.first.type << ", "
                              << pr.first.color << " + "
                              << pr.second.type << ", "
                              << pr.second.color << " ";
                }
                std::cout << std::endl;
            }
            num_collisions = collision_world::num_collisions();
        }
        else
        {
            num_collisions = 0;
        }

        if (observe_time < 100)
        {
            publish_command();
            return;
        }

        // If we don't have a plan to execute yet, find one
        if (current_plan.size() == 0)
        {
            if (num_collisions == 0)
            {
                searching = true;
                compute_next_plan();
                searching = false;
            }
            else
            {
                std::cout << "Trying to start a search from a collision state"
                          << std::endl;
            }
            return;
        }

        // Execution control if we do have a plan
        bool done = true;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            if (fabs(apos[i] - dpos[i]) > 0.01)
            {
                done = false;
                if (picked_up && !moved_drop && i == 5)
                {
                    done = true;
                }
                break;
            }
        }
        if (fabs(dhand - ahand) > 0.02)
        {
            done = false;
        }
        // if(current_plan.at(plan_index).size() == 1 &&
        //    current_plan.at(plan_index).at(0) == 0)
        // {
        //     std::cout << ahand << std::endl;
        // }

        // if(current_plan.at(plan_index).size() == 1 &&
        //    current_plan.at(plan_index).at(0) == 0 &&
        //    fabs(dhand - ahand) < 0.02)
        // {
        //     done = true;
        // }

        if (done && current_plan.size() > 0 &&
            plan_index < current_plan.size()-1)
        {
            std::cout << "Finished step " << plan_index << std::endl;
            plan_index++;

            if (current_plan.at(plan_index).size() == 1)
            {
                if (current_plan.at(plan_index).at(0) == 1)
                {
                    std::cout << "Opening hand" << std::endl;
                    dhand = 0.05;
                }
                else
                {
                    std::cout << "Closing hand" << std::endl;
                    dhand = 0.0;
                }
            }
            else
            {
                for (int i = 0; i < fetch_arm::get_num_joints(); i++)
                {
                    dpos.at(i) +=
                        current_plan.at(plan_index).at(i);
                }
            }
        }
        else if (done && !picked_up)
        {
            std::cout << "Going to compute grasp plan." << std::endl;
            compute_grasp_plan();
            picked_up = true;
        }
        else if (done && !moved_drop)
        {
            std::cout << "Going to compute second move." << std::endl;
            compute_next_plan();
            moved_drop = true;
        }
        else if (done && !dropped_off)
        {
            std::cout << "Going to compute drop plan." << std::endl;
            compute_grasp_plan();
            std::vector<pose> reverse_plan;
            reverse_plan.push_back(current_plan.at(2));
            reverse_plan.push_back(current_plan.at(1));
            reverse_plan.push_back(current_plan.at(4));
            current_plan = reverse_plan;
            dpos = apos;
            for (int i = 0; i < fetch_arm::get_num_joints(); i++)
            {
                dpos.at(i) += current_plan.at(0).at(i);
            }
            dropped_off = true;
        }

        publish_command();
    }

    void publish_command()
    {
        lcm::LCM lcm;
        dynamixel_command_list_t command;
        command.len = fetch_arm::get_num_joints() + 2;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = dpos.at(i);
            c.max_torque = fetch_arm::get_default_torque(i);
            c.speed = fetch_arm::get_default_speed(i)*0.05;
            command.commands.push_back(c);
        }

        dynamixel_command_t hand;
        hand.position_radians = dhand;
        hand.speed = 0.05;
        hand.max_torque = 0.5;
        // The hand has two separate joints in it
        command.commands.push_back(hand);
        command.commands.push_back(hand);


        lcm.publish("ARM_COMMAND", &command);
    }

    void handle_observations_message(const lcm::ReceiveBuffer* rbuf,
                                     const std::string& channel,
                                     const observations_t* obs)
    {
        if (searching) return;

        latest_objects = obs->observations;
        object_data od;
        collision_world::clear();
        for (std::vector<object_data_t>::iterator i =
                 latest_objects.begin();
             i != latest_objects.end(); i++)
        {
            collision_world::add_object(i->bbox_dim,
                                        i->bbox_xyzrpy,
                                        od);
        }

        observe_time++;
    }

    void compute_next_plan()
    {
        if (!picked_up)
        {
            arm_state::target[0] = 0;
            arm_state::target[1] = 0;
            arm_state::target[2] = 0.06;
        }
        else
        {
            arm_state::target[0] = 0.1;
            arm_state::target[1] = 0.2;
            arm_state::target[2] = 0.06;
        }
        arm_state::pitch_matters = true;

        std::cout << "[PLANNER] Initiating a search to "
                  << arm_state::target[0] << ", "
                  << arm_state::target[1] << ", "
                  << arm_state::target[2]
                  << std::endl;

        std::vector<search_result<arm_state, action> > latest_search;
        search_request<arm_state, action> req(arm_state(apos),
                                              fetch_arm::big_primitives(),
                                              fetch_arm::small_primitives(),
                                              5.0,
                                              true);
        arastar<arm_state, action>(req);

        latest_search = req.copy_solutions();
        current_plan = latest_search.at(latest_search.size()-1).path;
        current_plan = shortcut<arm_state, action>(current_plan,
                                                   arm_state(apos));
        std::cout << "Shortcutted to " << current_plan.size()
                  << std::endl;
        dpos = apos;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            dpos.at(i) += current_plan.at(0).at(i);
        }
        plan_index = 0;
    }

    void compute_grasp_plan()
    {
        current_plan.clear();
        action spin(fetch_arm::get_num_joints(), 0);
        float spin_angle = (0);

        spin.at(fetch_arm::get_num_joints()-1) = spin_angle;
        current_plan.push_back(spin);

        // 2. Open hand
        action hand_open(1, 1);
        current_plan.push_back(hand_open);

        // 3. Down onto obj
        Eigen::Matrix4f target_xform =
            fetch_arm::translation_matrix(fetch_arm::ee_xyz(apos)[0],
                                          fetch_arm::ee_xyz(apos)[1],
                                          -0.05);
        target_xform *= fetch_arm::rotation_matrix(M_PI/2, Y_AXIS);

    // action xyz = fetch_arm::solve_ik(position, target_xform);

    //     Eigen::Matrix4f start_xform = fetch_arm::ee_xform(apos);
    //     std::cout << "Start height " << start_xform(2,3) << std::endl;
    //     start_xform(2, 3) = start_xform(2, 3) - (0.09);
    //     std::cout << "End height " << start_xform(2,3) << std::endl;
        action down = fetch_arm::solve_ik(apos, target_xform);
        current_plan.push_back(down);

        // 4. Close hand
        action hand_close(1, 0);
        current_plan.push_back(hand_close);

        // 5. Back up to grasp point
        action up = down;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
            up.at(i) *= -1;
        current_plan.push_back(up);

        // 6. Unspin wrist
        action unspin = spin;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
            unspin.at(i) *= -1;
        current_plan.push_back(unspin);

        std::cout << "Made a grasp plan of len " << current_plan.size()  << std::endl;
        dpos = apos;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            dpos.at(i) += current_plan.at(0).at(i);
        }
        plan_index = 0;
    }

//     void handle_target_message(const lcm::ReceiveBuffer* rbuf,
//                                const std::string& channel,
//                                const search_target_t* targ)
//     {
//         searching = true;

//         lcm::LCM lcm;
//         point_3d goal;
//         for (int i = 0; i < 3; i++)
//         {
//             goal.push_back(targ->target[i]);
//         }

//         //////////////////////////
//         std::cout << "=== STARTING SEARCH ===" << std::endl;
// #ifdef USE_RRTSTAR
//         bool valid_sol = false;
//         pose end_pose;
//         pose ik_start = status;
//         int iterations = 0;

//         while (!valid_sol)
//         {
//             iterations++;
//             if (iterations > 500) break;
//             action ik_sol = fetch_arm::solve_ik(ik_start, goal);
//             for (action::iterator i = ik_sol.begin();
//                  i != ik_sol.end(); i++)
//             {
//                 if (*i != 0)
//                 {
//                     valid_sol = true;
//                     break;
//                 }
//             }

//             end_pose = fetch_arm::apply(ik_start, ik_sol);
//             end_pose.at(fetch_arm::get_num_joints() - 1) = 0;
//             action grip_sol = fetch_arm::solve_gripper(end_pose, -M_PI/2.f);
//             valid_sol = false;
//             for (action::iterator i = grip_sol.begin();
//                  i != grip_sol.end(); i++)
//             {
//                 if (*i != 0)
//                 {
//                     valid_sol = true;
//                     break;
//                 }
//             }
//             end_pose = fetch_arm::apply(end_pose, grip_sol);

//             valid_sol = (arm_state(end_pose).valid() &&
//                          (fetch_arm::ee_dist_to(end_pose, goal)
//                           < 0.01));

//             if (valid_sol)
//             {
//                 std::cout << "Got an end pose on iteration "
//                           << iterations << std::endl;
//                 break;
//             }

//             for (int i = 0; i < fetch_arm::get_num_joints(); i++)
//             {
//                 float prop = (((float)rand())/((float)RAND_MAX));
//                 ik_start.at(i) = (prop*(fetch_arm::get_joint_max(i) -
//                                         fetch_arm::get_joint_min(i)))
//                     + fetch_arm::get_joint_min(i);
//             }
//         }

//         if (!valid_sol)
//         {
//             std::cout << "TOTAL FAILURE TO FIND END POSE"
//                       << std::endl;
//             searching = false;
//             return;
//         }

//         current_plan = rrtstar::plan(status, end_pose);
//         current_plan.push_back(end_pose);
// #else
//         arm_state::target = goal;
//         arm_state::pitch_matters = true;
//         //arm_state::target_pitch = -M_PI/2.f;
//         std::vector<search_result<arm_state, action> > latest_search;

//         search_request<arm_state, action> req(arm_state(status),
//                                               fetch_arm::big_primitives(),
//                                               fetch_arm::small_primitives(),
//                                               100.0,
//                                               true);


//         arastar<arm_state, action>(req);

//         latest_search = req.copy_solutions();
//         current_plan = latest_search.at(latest_search.size()-1).path;
//         current_plan = shortcut<arm_state, action>(current_plan,
//                                                    arm_state(status));
//         std::cout << "Shortcutted to " << current_plan.size()
//                   << std::endl;
// #endif
//         ///////////////////////////

// #ifdef USE_RRTSTAR
//         dpos = current_plan.at(0);
// #else
//         dpos = status;
//         for (int i = 0; i < fetch_arm::get_num_joints(); i++)
//         {
//             dpos.at(i) += current_plan.at(0).at(i);
//         }
// #endif

//         plan_index = 0;
//         searching = false;
//     }

};

int main(int argc, char* argv[])
{
    int obj_id = -1;
    char color[2];
    int target_x = 0;
    int target_y = 0;

    for (int i = 1; i < argc-1; i++)
    {
        if (argv[i] == "-i")
        {
            obj_id = boost::lexical_cast<int>(argv[i + 1]);
        }
        else if (argv[i] == "-c")
        {
            color[0] = argv[i + 1][0];
            color[1] = argv[i + 1][1];
        }
        else if (argv[i] == "-x")
        {
            target_x = boost::lexical_cast<float>(argv[i + 1]);
        }
        else if (argv[i] == "-y")
        {
            target_y = boost::lexical_cast<float>(argv[i + 1]);
        }
    }

    lcm::LCM lcm;
    if (!lcm.good())
    {
        std::cout << "Failed to initialize LCM." << std::endl;
        return 1;
    }

    fetch_arm::INIT();
    collision_world::clear();

    experiment_handler handler(obj_id, color, target_x, target_y);
    lcm.subscribe("ARM_STATUS", &experiment_handler::handle_status_message,
                  &handler);
    lcm.subscribe("OBSERVATIONS", &experiment_handler::handle_observations_message,
                  &handler);

    while(0 == lcm.handle());


    return 0;
}
