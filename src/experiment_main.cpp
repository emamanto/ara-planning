#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "experiment_main.h"

experiment_handler::experiment_handler(int obj_id,
                                       std::string color,
                                       float drop_target_x,
                                       float drop_target_y,
                                       bool reset = true) :
    ahand(0),
    dhand(0),
    plan_index(0),
    num_collisions(0),
    target_obj_id(obj_id),
    target_obj_color(color),
    drop_x(drop_target_x),
    drop_y(drop_target_y),
    current_stage(REACH),
    current_status(WAIT),
    observe_time(0)
{
    if (target_obj_color != "none")
    {
        std::cout << "Moving the " << target_obj_color
                  << " object ";
    }
    else if (target_obj_id != -1)
    {
        std::cout << "Moving the object with id "
                  << target_obj_id
                  << " ";
    }

    std::cout << "to x = " << drop_x << ", y = "
              << drop_y << std::endl;


    if (reset)
    {
        dpos.push_back(M_PI/8);
        dpos.push_back(M_PI/2);
        dpos.push_back(-M_PI/2 + M_PI/8);
        dpos.push_back(M_PI/2);
        dpos.push_back(0);
        dpos.push_back(M_PI/2);
        dpos.push_back(0);
    }

    dhand = 0;
};

void experiment_handler::handle_status_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const dynamixel_status_list_t* stats)
{
    if (current_status == SEARCH) return;

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
    if (dpos.size() == 0)
    {
        dpos = apos;
    }

    ahand = stats->statuses[fetch_arm::get_num_joints()].position_radians;

    check_collisions();

    if (observe_time < 10 ||
        (current_stage == REACH && !motion_done()))
    {
        publish_command();
        return;
    }

    if (current_status == WAIT)
    {
        if (num_collisions == 0)
        {
            compute_next_plan();
        }
        else
        {
            std::cout << "Trying to start a search from a collision state"
                      << std::endl;
        }
        return;
    }

    // Execution control if we do have a plan
    if (current_status == EXECUTE)
    {
        if (!motion_done())
        {
            publish_command();
            return;
        }

        // Not yet at end of plan execution
        if (current_plan.size() > 0 &&
            plan_index < current_plan.size()-1)
        {
            plan_index++;

            if (current_plan.at(plan_index).size() == 1)
            {
                request_hand_motion(current_plan[plan_index][0]);
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
        // Finished executing plan, move to next stage
        else if (current_stage == REACH)
        {
            compute_grasp_plan();
            current_stage = GRASP;
        }
        else if (current_stage == GRASP)
        {
            compute_next_plan();
            current_stage = MOVE;
        }
        else if (current_stage == MOVE)
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
            current_stage = DROP;
        }
        else
        {
            current_status = WAIT;
            exit(0);
        }
    }
}

void experiment_handler::check_collisions()
{
    if (collision_world::collision(apos, ahand, true))
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
}

void experiment_handler::publish_command()
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

void experiment_handler::handle_observations_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const observations_t* obs)
{
    if (current_status == SEARCH) return;

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

void experiment_handler::compute_next_plan()
{
    current_status = SEARCH;
    if (current_stage == REACH)
    {
        set_reach_point();
    }
    else
    {
        arm_state::target[0] = drop_x;
        arm_state::target[1] = drop_y;
        arm_state::target[2] = 0.1;
    }
    arm_state::pitch_matters = true;

    std::cout << "New search to target: "
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
    current_status = EXECUTE;
}

void experiment_handler::compute_grasp_plan()
{
    current_status = SEARCH;
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
    current_status = EXECUTE;
}

void experiment_handler::set_reach_point()
{
    float x = 0;
    float y = 0;

    // USE ID
    if (target_obj_id != -1)
    {
        bool found = false;
        for (std::vector<object_data_t>::iterator i =
                 latest_objects.begin();
             i != latest_objects.end(); i++)
        {
            if (i->id == target_obj_id)
            {
                x = i->bbox_xyzrpy[0];
                y = i->bbox_xyzrpy[1];
                found = true;
                break;
            }
        }
        if (!found)
        {
            std::cout << "No object with requested id" << std::endl;
            exit(1);
        }
    }
    // USE COLOR
    else
    {
        bool found = false;
        for (std::vector<object_data_t>::iterator i =
                 latest_objects.begin();
             i != latest_objects.end(); i++)
        {
            for (int j = 0; j < i->num_cat; j++)
            {
                if (i->cat_dat[j].cat.cat == 1)
                {
                    for (int k = 0; k < i->cat_dat[j].len; k++)
                    {
                        if (i->cat_dat[j].label[k].compare(target_obj_color) == 0 &&
                        i->cat_dat[j].confidence[k] > 0.5)
                        {
                            x = i->bbox_xyzrpy[0];
                            y = i->bbox_xyzrpy[1];
                            found = true;
                            break;
                        }
                    }
                }
            }
        }
        if (!found)
        {
            std::cout << "No object with requested color" << std::endl;
            exit(1);
        }
    }

    arm_state::target[0] = x;
    arm_state::target[1] = y;
    arm_state::target[2] = 0.1;
}

bool experiment_handler::motion_done()
{
    bool done = true;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        if (fabs(apos[i] - dpos[i]) > 0.01)
        {
            done = false;
            break;
        }
    }
    if (fabs(dhand - ahand) > 0.02)
    {
        done = false;
    }
    return done;
}

void experiment_handler::request_hand_motion(bool opening)
{
    if (opening == 1)
    {
        std::cout << "Opening hand" << std::endl;
        dhand = 0.05;
    }
    else if (opening == 0)
    {
        std::cout << "Closing hand" << std::endl;
        dhand = 0.0;
    }
    else
    {
        std::cout << "Invalid hand request" << std::endl;
    }
}

int main(int argc, char* argv[])
{
    int obj_id = -1;
    std::string color = "none";
    float target_x = 0;
    float target_y = 0;
    bool reset = false;

    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "-i")
        {
            obj_id = boost::lexical_cast<int>(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "-c")
        {
            color = std::string(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "-x")
        {
            target_x = boost::lexical_cast<float>(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "-y")
        {
            target_y = boost::lexical_cast<float>(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "-r")
        {
            reset = true;
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

    experiment_handler handler(obj_id, color, target_x, target_y, reset);
    lcm.subscribe("ARM_STATUS", &experiment_handler::handle_status_message,
                  &handler);
    lcm.subscribe("OBSERVATIONS", &experiment_handler::handle_observations_message,
                  &handler);

    while(0 == lcm.handle());


    return 0;
}
