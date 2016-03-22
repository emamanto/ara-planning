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
                                       float time_lim,
                                       float step_size_deg,
                                       bool first_sol,
                                       bool reset) :
    ahand(0),
    dhand(0),
    hand_speed(0),
    holding_object(false),
    plan_index(0),
    stage_collision_time(0),
    current_min_distance(0),
    target_obj_id(obj_id),
    target_obj_color(color),
    drop_x(drop_target_x),
    drop_y(drop_target_y),
    drop_z(0),
    search_time_lim(time_lim),
    search_step_size(step_size_deg),
    search_first_sol(first_sol),
    current_stage(REACH),
    current_status(WAIT),
    observe_time(0)
{
    fetch_arm::set_primitive_change(search_step_size*DEG_TO_RAD);

    if (target_obj_color != "none")
    {
        std::cout << std::endl;
        std::cout << "[INPUT] Moving the " << target_obj_color
                  << " object ";
    }
    else if (target_obj_id != -1)
    {
        std::cout << "[INPUT] Moving the object with id "
                  << target_obj_id
                  << " ";
    }

    std::cout << "to x = " << drop_x << ", y = "
              << drop_y << std::endl;

    std::cout << "[INPUT] Search will ";
    if (!first_sol)
    {
        std::cout << "not ";
    }
    std::cout << "return first solution, using step size "
              << search_step_size << " deg and time limit "
              << search_time_lim << std::endl;


    if (reset)
    {
        dpos.push_back(fetch_arm::get_joint_max(0));
        dpos.push_back(0);
        dpos.push_back(-M_PI/2);
        dpos.push_back(M_PI/3);
        dpos.push_back(M_PI/2);
        dpos.push_back(M_PI/2);
        dpos.push_back(-M_PI/2);
    }

    arm_state::collision_model = &observed;
};

void experiment_handler::handle_status_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const dynamixel_status_list_t* stats)
{
    if (current_status == SEARCH) return;
    if (observe_time < 10) return;

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
    hand_speed = stats->statuses[fetch_arm::get_num_joints()].speed;

    check_collisions();

    if (current_stage == REACH && !motion_done())
    {
        publish_command();
        return;
    }

    if (current_status == WAIT)
    {
        if (collision_ids.size() == 0)
        {
            compute_next_plan();
        }
        else if (collision_ids.size() == 1 && holding_object)
        {
            compute_next_plan();
        }
        else
        {
            std::cout << "[ERROR] Trying to start a search from a collision state"
                      << std::endl;
            exit(1);
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
            std::cout << "[STATS] Reach execution time was "
                      << (execution_timer.elapsed().wall / 1e9)
                      << " s" <<std::endl;

            double sum = 0;
            for (std::vector<double>::iterator i = current_distances.begin();
                 i != current_distances.end(); i++)
            {
                sum += *i;
            }

            std::cout << "[STATS] Reach average clearance was "
                      << (sum / current_distances.size())
                      << " m" <<std::endl;
            std::cout << "[STATS] Reach minimum clearance was "
                      << current_min_distance
                      << " m" <<std::endl;

            print_stage(GRASP);
            compute_grasp_plan();
            current_stage = GRASP;
        }
        else if (current_stage == GRASP)
        {
            std::cout << "[STATS] Grasp execution time was "
                      << (execution_timer.elapsed().wall / 1e9)
                      << " s" <<std::endl;

            print_stage(MOVE);
            current_stage = MOVE;
            current_status = WAIT;
        }
        else if (current_stage == MOVE)
        {
            double sum = 0;
            for (std::vector<double>::iterator i = current_distances.begin();
                 i != current_distances.end(); i++)
            {
                sum += *i;
            }

            std::cout << "[STATS] Move execution time was "
                      << (execution_timer.elapsed().wall / 1e9)
                      << " s" <<std::endl;
            std::cout << "[STATS] Move average clearance was "
                      << (sum / current_distances.size())
                      << " m" <<std::endl;
            std::cout << "[STATS] Move minimum clearance was "
                      << current_min_distance
                      << " m" <<std::endl;


            print_stage(DROP);
            compute_drop_plan();
            current_stage = DROP;
            current_distances.clear();
            current_min_distance = 99999;
        }
        else
        {
            current_status = WAIT;

            std::cout << "[STATS] Drop execution time was "
                      << (execution_timer.elapsed().wall / 1e9)
                      << " s" <<std::endl;
            std::cout << "[COLLISION] Total frames in stage = "
                      << stage_collision_time << std::endl;

            exit(0);
        }
    }
}

void experiment_handler::check_collisions()
{
    collision_info inf = ground_truth.collision(apos, ahand, true, true);

    if (inf.distance > 0)
    {
        current_distances.push_back(inf.distance);
        if (inf.distance < current_min_distance)
            current_min_distance = inf.distance;
    }

    if (inf.collision)
    {
        std::vector<int> updated;
        if (ground_truth.num_collisions() == 1 &&
            (current_stage == GRASP ||
             current_stage == DROP ||
             holding_object))
        {
            collision_ids.clear();
            return;
        }
        for (int i = 0; i < ground_truth.num_collisions(); i++)
        {
            collision_pair pr = ground_truth.get_collision_pair(i);
            int id;
            if (pr.first.type == "hand" || pr.first.type == "arm")
            {
                id = pr.second.id;
            }
            else
            {
                id = pr.first.id;
            }

            bool is_new = true;
            for (std::vector<int>::iterator j = collision_ids.begin();
                 j != collision_ids.end(); j++)
            {
                if (*j == id)
                {
                    is_new = false;
                    break;
                }
            }

            if (is_new)
            {
                std::cout << "[COLLISION] Between "
                          << pr.first.type << " + "
                          << pr.second.type
                          << std::endl;
            }
            updated.push_back(id);
        }

        stage_collision_time++;
        collision_ids = updated;
    }
    else
    {
        collision_ids.clear();
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
        if (current_stage == GRASP ||
            current_stage == DROP)
        {
            c.speed *= 0.5;
        }
        command.commands.push_back(c);
    }

    dynamixel_command_t hand;
    hand.position_radians = dhand;
    hand.speed = 0.01;
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

    if (channel == "GROUND_TRUTH_OBJECTS")
    {
        latest_objects = obs->observations;
        ground_truth.clear();
        for (std::vector<object_data_t>::iterator i =
                 latest_objects.begin();
             i != latest_objects.end(); i++)
        {
            // HACK--If I change the size of the target
            // objects, I need to change this!
            if (i->bbox_dim[0] == 0.06 &&
                i->bbox_dim[1] == 0.06 &&
                i->bbox_dim[2] == 0.06)
            {
                continue;
            }

            object_data od;
            od.id = i->id;
            od.type = "block";
            od.color = "ground truth";
            ground_truth.add_object(i->bbox_dim,
                                i->bbox_xyzrpy,
                                od);
        }
    }
    else if (channel == "OBSERVATIONS")
    {
        observe_time++;
        latest_observations = obs->observations;
        observed.clear();
        for (std::vector<object_data_t>::iterator i =
                 latest_observations.begin();
             i != latest_observations.end(); i++)
        {
            object_data od;
            od.id = i->id;
            od.type = "block";
            od.color = "observed";
            observed.add_object(i->bbox_dim,
                                i->bbox_xyzrpy,
                                od);
        }
    }
}

void experiment_handler::compute_next_plan()
{
    current_status = SEARCH;
    if (current_stage == REACH)
    {
        print_stage(REACH);
        set_reach_point();

        current_distances.clear();
        current_min_distance = 99999;
    }
    else
    {
        arm_state::target[0] = drop_x;
        arm_state::target[1] = drop_y;
        arm_state::target[2] = drop_z;
    }

    std::cout << "[CONTROL] New search to target: x = "
              << arm_state::target[0] << ", y = "
              << arm_state::target[1] << ", z = "
              << arm_state::target[2]
              << std::endl;

    std::vector<search_result<arm_state, action> > latest_search;
    search_request<arm_state, action> req(arm_state(apos),
                                          fetch_arm::big_primitives(),
                                          fetch_arm::small_primitives(),
                                          search_time_lim,
                                          true);
    arastar<arm_state, action>(req);

    latest_search = req.copy_solutions();
    current_plan = latest_search.at(latest_search.size()-1).path;
    current_plan = shortcut<arm_state, action>(current_plan,
                                               arm_state(apos));
    std::cout << "[CONTROL] Shortcutted plan to " << current_plan.size()
              << " actions" << std::endl;

    float joint_dist = 0;
    for (std::vector<action>::iterator i = current_plan.begin();
         i != current_plan.end(); i++)
    {
        for (action::iterator j = i->begin(); j != i->end(); j++)
        {
            joint_dist += fabs(*j);
        }
    }
    std::cout << "[STATS] Plan length in joint space = "
              << joint_dist << " rad" << std::endl;

    float hand_dist = 0;
    pose step = apos;
    for (int i = 0; i < current_plan.size(); i++)
    {
        pose next_step = fetch_arm::apply(step, current_plan.at(i));
        hand_dist += fetch_arm::ee_dist_to(step, fetch_arm::ee_xyz(next_step));
        step = next_step;
    }
    std::cout << "[STATS] Plan length in hand distance traveled = "
              << hand_dist << " m" << std::endl;

    dpos = apos;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        dpos.at(i) += current_plan.at(0).at(i);
    }
    plan_index = 0;
    execution_timer.start();
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
                                      fetch_arm::ee_xyz(apos)[2]-0.04);
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

    dpos = apos;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        dpos.at(i) += current_plan.at(0).at(i);
    }
    plan_index = 0;
    execution_timer.start();
    current_status = EXECUTE;
}

void experiment_handler::compute_drop_plan()
{
    current_status = SEARCH;
    current_plan.clear();

    // 1. Down
    Eigen::Matrix4f target_xform =
        fetch_arm::translation_matrix(fetch_arm::ee_xyz(apos)[0],
                                      fetch_arm::ee_xyz(apos)[1],
                                      fetch_arm::ee_xyz(apos)[2]-0.04);
    target_xform *= fetch_arm::rotation_matrix(M_PI/2, Y_AXIS);
    action down = fetch_arm::solve_ik(apos, target_xform);
    current_plan.push_back(down);

    // 2. Open hand
    action hand_open(1, 1);
    current_plan.push_back(hand_open);

    // 3. Back up to grasp point
    action up = down;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        up.at(i) *= -1;
    current_plan.push_back(up);

    // 4. Close hand
    action hand_close(1, 0);
    current_plan.push_back(hand_close);

    dpos = apos;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        dpos.at(i) += current_plan.at(0).at(i);
    }
    plan_index = 0;
    execution_timer.start();
    current_status = EXECUTE;
}

void experiment_handler::set_reach_point()
{
    float x = 0;
    float y = 0;
    float z = 0;

    while (latest_observations.size() == 0)
    {
        sleep(0.01);
    }

    // USE ID
    if (target_obj_id != -1)
    {
        bool found = false;
        for (std::vector<object_data_t>::iterator i =
                 latest_observations.begin();
             i != latest_observations.end(); i++)
        {
            if (i->id == target_obj_id)
            {
                x = i->bbox_xyzrpy[0];
                y = i->bbox_xyzrpy[1];
                z = i->bbox_xyzrpy[2] + (i->bbox_dim[2]/2) + 0.02;
                for (int j = 0; j < 3; j++)
                    target_obj_dim.push_back(i->bbox_dim[j]);
                found = true;
                break;
            }
        }
        if (!found)
        {
            std::cout << "[ERROR] No object with requested id" << std::endl;
            exit(1);
        }
    }
    // USE COLOR
    else
    {
        bool found = false;
        for (std::vector<object_data_t>::iterator i =
                 latest_observations.begin();
             i != latest_observations.end(); i++)
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
                            z = i->bbox_xyzrpy[2] + (i->bbox_dim[2]/2) + 0.02;
                            for (int c = 0; c < 3; c++)
                                target_obj_dim.push_back(i->bbox_dim[c]);
                            found = true;
                            break;
                        }
                    }
                }
            }
        }
        if (!found)
        {
            std::cout << "[ERROR] No object with requested color" << std::endl;
            exit(1);
        }
    }

    arm_state::target[0] = x;
    arm_state::target[1] = y;
    arm_state::target[2] = z;
    drop_z = z;
}

bool experiment_handler::motion_done()
{
    bool done = true;
    if (fabs(dhand - ahand) > 0.01)
    {
        done = false;
    }
    // Checks if the hand is closing for a grasp
    if (dhand == 0 && hand_speed < 0.0001  &&
        current_plan.size() > 0 &&
        current_stage == GRASP &
        current_plan.at(plan_index).size() == 1 &&
        !holding_object)
    {
        std::cout << "[CONTROL] Grasped an object" << std::endl;
        done = true;
        holding_object = true;
        collision_world::set_held_object(target_obj_dim);
    }
    else if (hand_speed < 0.001 && holding_object)
    {
        done = true;
    }

    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        if (fabs(apos[i] - dpos[i]) > 0.01)
        {
            done = false;
            break;
        }
    }
    return done;
}

void experiment_handler::request_hand_motion(bool opening)
{
    if (opening == 1)
    {
        dhand = 0.05;
        if (holding_object)
            std::cout << "[CONTROL] Releasing object" << std::endl;
        collision_world::clear_held_object();
        holding_object = false;
    }
    else if (opening == 0)
    {
        dhand = 0.0;
    }
    else
    {
        std::cout << "[ERROR] Invalid hand request" << std::endl;
    }
}

void experiment_handler::print_stage(stage s)
{
    std::cout << "[COLLISION] Total frames in stage = "
              << stage_collision_time << std::endl;
    stage_collision_time = 0;
    std::cout << std::endl;
    switch (s)
    {
    case 0:
        std::cout << "[STAGE] REACH" << std::endl;
        break;
    case 1:
        std::cout << "[STAGE] GRASP" << std::endl;
        break;
    case 2:
        std::cout << "[STAGE] MOVE" << std::endl;
        break;
    case 3:
        std::cout << "[STAGE] DROP" << std::endl;
        break;
    }
}

int main(int argc, char* argv[])
{
    int obj_id = -1;
    std::string color = "none";
    float target_x = 0;
    float target_y = 0;
    float time_lim = 5;
    float step_size_deg = 15;
    bool reset = false;
    bool first_sol = false;

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
        else if (std::string(argv[i]) == "-t")
        {
            time_lim = boost::lexical_cast<float>(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "-s")
        {
            step_size_deg = boost::lexical_cast<float>(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "-f")
        {
            first_sol = true;
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

    experiment_handler handler(obj_id, color, target_x, target_y,
                               time_lim, step_size_deg, first_sol, reset);

    lcm.subscribe("ARM_STATUS", &experiment_handler::handle_status_message,
                  &handler);
    lcm.subscribe("GROUND_TRUTH_OBJECTS", &experiment_handler::handle_observations_message,
                  &handler);
    lcm.subscribe("OBSERVATIONS", &experiment_handler::handle_observations_message,
                  &handler);

    while(0 == lcm.handle());


    return 0;
}
