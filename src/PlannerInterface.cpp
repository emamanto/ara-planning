#include "PlannerInterface.h"

#define PUBLISH_COLLISION_MODEL
//#define DEBUG_EXECUTION
//#define SLOW_SPEED

pthread_t planner_interface::thrd = pthread_t();

float planner_interface::PRIMITIVE_SIZE_MIN = 2.f;
float planner_interface::PRIMITIVE_SIZE_MAX = 20.f;
float planner_interface::MIN_PROP_SPEED = 0.2f;

float planner_interface::GRASP_TOP_OFFSET = 0.05;
float planner_interface::GRASP_INTO_OBJ_OFFSET = 0.03;

planner_interface::planner_interface() :
    arm_status(probcog_arm::get_num_joints(), 0),
    search_cmd_id(-1),
    execute_cmd_id(-1),
    task(WAITING_INITIAL),
    current_command_index(0),
    requested_speed(1),
    in_collision(false),
    last_id_handled(-1),
    current_plan_type(MOVE),
    current_plan_is_rrt(false)
{
}

void* planner_interface::search_thread(void* arg)
{
    planner_interface* pi = static_cast<planner_interface*>(arg);
    std::cout << "[PLANNER] Search thread started." << std::endl;
    arastar<arm_state, action>(pi->latest_request);
    pi->search_complete();
}

// Called by search thread once search is finished/stopped
void planner_interface::search_complete()
{
    lcm::LCM lcm;
    planner_response_t resp;

    if (latest_request.check_killed())
    {
        resp.response_type = "STOP";
    }
    else
    {
        resp.response_type = "PLAN";
    }

    resp.finished = true;
    resp.response_id = search_cmd_id;
    latest_search = latest_request.copy_solutions();
    if (latest_search.size() > 0 &&
        !latest_search.at(0).path.empty())
    {
        resp.success = true;
    }
    else
    {
        resp.success = false;
    }

    resp.plan_size =
        latest_search.at(latest_search.size()-1).path.size();

    current_plan = latest_search.at(latest_search.size()-1).path;

    // Keep track of what kind of plan we just made because we
    // can't add the grasp/drop plan onto it yet (shortcutting)
    if (task == PLANNING_GRASP) current_plan_type = GRASP;
    else if (task == PLANNING_DROP) current_plan_type = DROP;
    else current_plan_type = MOVE;

    task = WAITING;
    lcm.publish("PLANNER_RESPONSES", &resp);
    last_response = resp;
}

void planner_interface::set_grasp_target(double dim[], double xyzrpy[])
{
    arm_state::pitch_matters = true;
    if (fabs(xyzrpy[0]) < 0.2 && fabs(xyzrpy[1]) < 0.2
        && xyzrpy[2] < 0.3)
    {
        arm_state::target_pitch = -M_PI/2.f;
        arm_state::target[0] = xyzrpy[0];
        arm_state::target[1] = xyzrpy[1];
        arm_state::target[2] = (xyzrpy[2] + dim[2]/2.f +
                                GRASP_TOP_OFFSET);
    }
    else// if (xyzrpy[2] < 0.2)
    {
        arm_state::target_pitch = -M_PI/2.f;
        arm_state::target[0] = xyzrpy[0];
        arm_state::target[1] = xyzrpy[1];
        arm_state::target[2] = (xyzrpy[2] + dim[2]/2.f +
                                GRASP_TOP_OFFSET);
    }
    if (arm_state::target[2] > 0.15) arm_state::target[2] = 0.15;

    // IGNORE SIDE GRABS UGH
    // else
    // {
    //     arm_state::target_pitch = 0.f;
    //     arm_state::target[2] = xyzrpy[2];

    //     if (fabs(xyzrpy[0]) < 0.01)
    //     {
    //         arm_state::target[0] = 0;
    //         float offset = dim[1]/2.f + 0.04;
    //         if (xyzrpy[1] < 0) offset *= -1;
    //         arm_state::target[1] = xyzrpy[1] - offset;
    //     }
    //     else if (fabs(xyzrpy[1]) < 0.01)
    //     {
    //         arm_state::target[1] = 0;
    //         float offset = dim[0]/2.f + 0.04;
    //         if (xyzrpy[0] < 0) offset *= -1;
    //         arm_state::target[0] = xyzrpy[0] - offset;
    //     }
    //     else if (fabs(xyzrpy[0]) > fabs(xyzrpy[1]))
    //     {
    //         float offset = dim[0]/2.f + 0.04;
    //         if (xyzrpy[0] < 0) offset *= -1;
    //         arm_state::target[0] = xyzrpy[0] - offset;
    //         arm_state::target[1] = (arm_state::target[0] *
    //                                 xyzrpy[1]) / xyzrpy[0];
    //     }
    //     else
    //     {
    //         float offset = dim[1]/2.f + 0.04;
    //         if (xyzrpy[1] < 0) offset *= -1;
    //         arm_state::target[1] = xyzrpy[1] - offset;
    //         arm_state::target[0] = (arm_state::target[1] *
    //                                 xyzrpy[0]) / xyzrpy[1];
    //     }
    // }
}

pose planner_interface::compute_rrt_target_pose(point_3d xyz,
                                                float pitch)
{
        bool valid_sol = false;
        pose end_pose;
        pose ik_start = arm_status;
        int iterations = 0;

        while (!valid_sol)
        {
            iterations++;
            if (iterations > 500) break;
            action ik_sol = probcog_arm::solve_ik(ik_start, xyz);
            for (action::iterator i = ik_sol.begin();
                 i != ik_sol.end(); i++)
            {
                if (*i != 0)
                {
                    valid_sol = true;
                    break;
                }
            }

            end_pose = probcog_arm::apply(ik_start, ik_sol);
            end_pose.at(probcog_arm::get_num_joints() - 1) = 0;
            action grip_sol = probcog_arm::solve_gripper(end_pose, pitch);
            valid_sol = false;
            for (action::iterator i = grip_sol.begin();
                 i != grip_sol.end(); i++)
            {
                if (*i != 0)
                {
                    valid_sol = true;
                    break;
                }
            }
            end_pose = probcog_arm::apply(end_pose, grip_sol);

            valid_sol = (arm_state(end_pose).valid() &&
                         (probcog_arm::ee_dist_to(end_pose, xyz)
                          < 0.01));

            if (valid_sol)
            {
                std::cout << "Got an end pose on iteration "
                          << iterations << std::endl;
                break;
            }

            for (int i = 0; i < probcog_arm::get_num_joints(); i++)
            {
                float prop = (((float)rand())/((float)RAND_MAX));
                ik_start.at(i) = (prop*(probcog_arm::get_joint_max(i) -
                                        probcog_arm::get_joint_min(i)))
                    + probcog_arm::get_joint_min(i);
            }
        }

        if (!valid_sol)
        {
            std::cout << "TOTAL FAILURE TO FIND END POSE"
                      << std::endl;
            return pose(probcog_arm::get_num_joints(), 0);
        }
        return end_pose;
}

std::vector<action> planner_interface::plan_grasp(pose start)
{
    std::vector<action> plan;
    // Straight down grab
    if (fabs(probcog_arm::ee_pitch(start) - -M_PI/2) < 0.1)
    {
        // 1. Spin wrist
        action spin(probcog_arm::get_num_joints(), 0);
        float spin_angle = (M_PI/2.f + arm_status.at(0)) +
            (M_PI/2.f - target_obj_xyzrpy[5]);

        while (spin_angle > probcog_arm::get_joint_max(4))
        {
            spin_angle -= M_PI/2.f;
        }
        while (spin_angle < probcog_arm::get_joint_min(4))
        {
            spin_angle += M_PI/2.f;
        }

        spin.at(probcog_arm::get_num_joints()-1) = spin_angle;
        plan.push_back(spin);

        // 2. Open hand
        action hand_open(1, 1);
        plan.push_back(hand_open);

        // 3. Down onto obj
        point_3d end = probcog_arm::ee_xyz(start);
        end.at(2) = end.at(2) - (GRASP_TOP_OFFSET +
                                 GRASP_INTO_OBJ_OFFSET);
        action down = probcog_arm::solve_ik(start, end);
        plan.push_back(down);

        // 4. Close hand
        action hand_close(1, 0);
        plan.push_back(hand_close);

        // 5. Back up to grasp point
        action up = down;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
            up.at(i) *= -1;
        plan.push_back(up);

        // 6. Unspin wrist
        action unspin = spin;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
            unspin.at(i) *= -1;
        plan.push_back(unspin);
    }
    else std::cout << "You're doing something wrong!!" << std::endl;

#ifdef DEBUG_EXECUTION
    std::cout << "Made a grasp plan of " << plan.size()
              << " steps." << std::endl;
#endif

    return plan;
}

std::vector<action> planner_interface::plan_drop(pose start)
{
    std::vector<action> plan;

    // Straight down drop
    if (fabs(probcog_arm::ee_pitch(start) - -M_PI/2) < 0.1)
    {
        // 1. Down for gentler drop
        point_3d end = probcog_arm::ee_xyz(start);
        end.at(2) = end.at(2) - (GRASP_TOP_OFFSET +
                                 GRASP_INTO_OBJ_OFFSET);;
        action down = probcog_arm::solve_ik(start, end);
        plan.push_back(down);

        // 2. Open hand
        action hand_open(1, 1);
        plan.push_back(hand_open);

        // 3. Back up
        action up = down;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
            up.at(i) *= -1;
        plan.push_back(up);
    }
    else std::cout << "You're doing something wrong!!" << std::endl;

#ifdef DEBUG_EXECUTION
    std::cout << "Made a drop plan of " << plan.size()
              << " steps." << std::endl;
#endif
    return plan;
}

void planner_interface::process_new_plan_command(const planner_command_t* comm)
{
    if (comm->plan_type.compare("GRASP") == 0)
    {
        task = PLANNING_GRASP;
        for (std::vector<object_data_t>::iterator i = latest_objects.begin();
             i != latest_objects.end(); i++)
        {
            if (comm->target_object_id == i->id)
            {
                target_obj_dim = i->bbox_dim;
                target_obj_xyzrpy = i->bbox_xyzrpy;
                break;
            }
        }

        set_grasp_target(target_obj_dim, target_obj_xyzrpy);
    }
    else if (comm->plan_type.compare("DROP") == 0)
    {
        task = PLANNING_DROP;
        double drop_point[6];
        drop_point[0] = comm->target[0];
        drop_point[1] = comm->target[1];
        drop_point[2] = comm->target[2];
        for (int i = 3; i < 6; i++) drop_point[i] = 0;
        set_grasp_target(grasped_obj_dim, drop_point);
    }
    else
    {
        task = PLANNING_MOVE;
        point_3d goal;
        for (int i = 0; i < 3; i++)
        {
            goal.push_back(comm->target[i]);
        }
        arm_state::target = goal;
        arm_state::pitch_matters = false;
    }

    last_id_handled = comm->command_id;
    search_cmd_id = comm->command_id;

    if (comm->planning_algorithm.compare("ARA") == 0)
    {
        float big_prim_size = (comm->primitive_size)*
            (PRIMITIVE_SIZE_MAX - PRIMITIVE_SIZE_MIN) +
            PRIMITIVE_SIZE_MIN;
        std::cout << std::endl
                  << "[PLANNER] Primitive size is " << big_prim_size
                  << std::endl;
        probcog_arm::set_primitive_change(big_prim_size);

        std::cout << "[PLANNER] Initiating a search to "
                  << arm_state::target[0] << ", "
                  << arm_state::target[1] << ", "
                  << arm_state::target[2] << ", pitch "
                  << arm_state::target_pitch
                  << std::endl;
        latest_start_pose = arm_status;

        latest_search.clear();
        if (comm->time_limit < 0)
        {
            std::cout << "[PLANNER] Setting no time limit."
                      << std::endl;
        }
        else {
            std::cout << "[PLANNER] Setting a ";
            if (comm->hard_limit) std::cout << "hard";
            else std::cout << "soft";
            std::cout << " time limit of "
                      << comm->time_limit <<  "." << std::endl;
        }

        latest_request =
            search_request<arm_state, action>(arm_state(latest_start_pose),
                                              probcog_arm::big_primitives(),
                                              probcog_arm::small_primitives(),
                                              comm->time_limit,
                                              comm->hard_limit);
        current_plan_is_rrt = false;
        pthread_create(&thrd, NULL, &search_thread, this);
    }
    else if (comm->planning_algorithm.compare("RRT") == 0)
    {
        pose end_pose = compute_rrt_target_pose(arm_state::target,
                                                arm_state::target_pitch);
        std::vector<pose> pose_plan = rrtstar::plan(arm_status,
                                                    end_pose);
        pose prev;
        current_plan.clear();

        for (std::vector<pose>::iterator i = pose_plan.begin();
             i != pose_plan.end(); i++)
        {
            if (i == pose_plan.begin())
            {
                prev = *i;
                continue;
            }

            current_plan.push_back(subtract(*i, prev));
            prev = *i;
        }

        current_plan.push_back(subtract(end_pose, prev));
        current_plan_is_rrt = true;

        lcm::LCM lcm;
        planner_response_t resp;
        resp.response_type = "PLAN";

        resp.finished = true;
        resp.response_id = comm->command_id;
        resp.success = true;

        resp.plan_size = current_plan.size();

        lcm.publish("PLANNER_RESPONSES", &resp);
        last_response = resp;
    }

    // Keep track of what kind of plan we just made because we
    // can't add the grasp/drop plan onto it yet (shortcutting)
    if (task == PLANNING_GRASP) current_plan_type = GRASP;
    else if (task == PLANNING_DROP) current_plan_type = DROP;
    else current_plan_type = MOVE;

    if (current_plan_is_rrt) task = WAITING;
}

void planner_interface::forward_command()
{
    for (int i = 0;
         i < probcog_arm::get_num_joints(); i++)
    {
        current_command.at(i) +=
            current_plan.at(current_command_index).at(i);
    }
}

void planner_interface::handle_command_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const planner_command_t* comm)
{
    if (task == WAITING)
    {
        lcm::LCM lcm;
        lcm.publish("PLANNER_RESPONSES", &last_response);
    }

    if (comm->command_type.compare("PLAN") == 0)
    {
        if (latest_request.has_hard_limit() == false &&
            latest_request.check_paused())
        {
            paused_task = task;
            task = PAUSED;
            lcm::LCM lcm;
            planner_response_t resp;

            resp.response_type = "PAUSE";
            resp.response_id = search_cmd_id;
            resp.finished = false;

            latest_search = latest_request.copy_solutions();
            if (latest_search.size() > 0)
            {
                current_plan = latest_search.at(latest_search.size()-1).path;
                resp.plan_size = current_plan.size();
            }
            else
            {
                resp.plan_size = 0;
            }

            resp.success = (resp.plan_size > 0);

            lcm.publish("PLANNER_RESPONSES", &resp);
            last_response = resp;
        }
    }
    if (comm->command_type.compare("PLAN") == 0 &&
        comm->command_id > last_id_handled)
    {
        process_new_plan_command(comm);
    }
    else if (comm->command_type.compare("STOP") == 0 &&
             comm->command_id > last_id_handled)
    {
        std::cout << "[PLANNER] Stopping the search." << std::endl;
        latest_request.kill();
        last_id_handled = comm->command_id;
        // RESPONSE
    }
    else if (comm->command_type.compare("PAUSE") == 0 &&
             comm->command_id > last_id_handled)
    {
        std::cout << "[PLANNER] Pausing the search." << std::endl;
        latest_request.pause();
        last_id_handled = comm->command_id;
        paused_task = task;
        task = PAUSED;

        lcm::LCM lcm;
        planner_response_t resp;
        resp.response_type = "PAUSE";
        resp.response_id = comm->command_id;
        resp.finished = true;
        resp.success = true;

        latest_search = latest_request.copy_solutions();
        if (latest_search.size() > 0)
        {
            current_plan = latest_search.at(latest_search.size()-1).path;
            resp.plan_size = current_plan.size();
        }
        else
        {
            resp.plan_size = 0;
        }

        lcm.publish("PLANNER_RESPONSES", &resp);
        last_response = resp;
    }
    else if (comm->command_type.compare("CONTINUE") == 0 &&
             comm->command_id > last_id_handled)
    {
        std::cout << "[PLANNER] Resuming the search." << std::endl;
        latest_request.extend_time(comm->time_limit);
        latest_request.set_hard_limit(comm->hard_limit);
        latest_request.unpause();
        last_id_handled = comm->command_id;
        task = paused_task;
        paused_task = NONE;

        lcm::LCM lcm;
        planner_response_t resp;
        resp.response_type = "CONTINUE";
        resp.response_id = comm->command_id;
        resp.finished = true;
        resp.success = true;
        resp.plan_size = last_response.plan_size;

        lcm.publish("PLANNER_RESPONSES", &resp);
        last_response = resp;
    }
    else if (comm->command_type.compare("POSTPROCESS") == 0 &&
             comm->command_id > last_id_handled)
    {
        task = POSTPROCESSING;
        // SHORTCUT **Add actually using the parameter in the msg
        if (!current_plan_is_rrt)
        {
            int original = current_plan.size();
            std::cout << "[PLANNER] Smoothing a path of " << original;
            current_plan =
                shortcut<arm_state, action>(current_plan,
                                            arm_state(latest_start_pose));
            std::cout << " to " << current_plan.size() << std::endl;
        }
        last_id_handled = comm->command_id;

        lcm::LCM lcm;
        planner_response_t resp;
        resp.response_type = "POSTPROCESS";
        resp.response_id = comm->command_id;
        resp.finished = true;
        // XXX This isn't quite right.
        resp.success = true;
        resp.plan_size = current_plan.size();

        lcm.publish("PLANNER_RESPONSES", &resp);
        last_response = resp;

        task = WAITING;
    }
    else if (comm->command_type.compare("RESET") == 0 &&
             comm->command_id > last_id_handled)
    {
        task = EXECUTING;
        last_id_handled = comm->command_id;
        execute_cmd_id = comm->command_id;
        std::cout << "[PLANNER] Resetting the arm."
                  << std::endl << std::endl;
        current_command = pose(probcog_arm::get_num_joints(), 0);
        current_plan.clear();
        current_plan_type = MOVE;
        current_command_index = -1;
    }
    else if (comm->command_type.compare("EXECUTE") == 0 &&
             comm->command_id > last_id_handled)
    {
        last_id_handled = comm->command_id;
        execute_cmd_id = comm->command_id;
        current_command = arm_status;
        current_command_index = 0;
        forward_command();
        current_hand_command = 112.f*DEG_TO_RAD;
        requested_speed = comm->speed;

        task = EXECUTING;
        std::cout << "[PLANNER] Starting execution."
                  << std::endl;
    }
}

void planner_interface::handle_status_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const dynamixel_status_list_t* stats)
{
    pose np;
    for (int i = 0; i < probcog_arm::get_num_joints(); i++)
    {
        np.push_back(stats->statuses[i].position_radians);
    }
    float hand_status =
        stats->statuses[probcog_arm::get_num_joints()].position_radians;
    if (np != arm_status) arm_status = np;

    lcm::LCM lcm;

    if (!planning())
    {
#ifdef PUBLISH_COLLISION_MODEL
        arm_collision_boxes_t arm_msg = collision_world::arm_boxes(arm_status);
        lcm.publish("ARM_COLLISION_BOXES", &arm_msg);
#endif

        if (collision_world::collision(arm_status, true) &&
            !in_collision)
        {
            std::cout << "[CONTROLLER] "
                      << (collision_world::num_collisions())
                      << " collisions: ";
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
            in_collision = true;
        }
        else if (in_collision && !collision_world::collision(arm_status))
        {
            in_collision = false;
        }
    }


    if (task == WAITING_INITIAL)
    {
        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = 0;
            c.speed = probcog_arm::get_default_speed(i);
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
        }

        dynamixel_command_t hand;
        hand.position_radians = 112.f*DEG_TO_RAD;
        hand.speed = 0.15;
        hand.max_torque = 0.5;
        command.commands.push_back(hand);
        lcm.publish("ARM_COMMAND", &command);
    }

    if (task == EXECUTING || task == GRASPING)
    {
        bool done = true;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            if (fabs(arm_status[i] - current_command[i]) > 0.01)
            {
                done = false;
                break;
            }
        }
        if (task == GRASPING &&
            current_plan.at(current_command_index).size() == 1)
        {
            if (fabs(hand_status - current_hand_command) > 0.01)
                done = false;
        }
        if (current_command_index >= 0 &&
            current_plan_type == GRASP &&
            current_plan.at(current_command_index).size() == 1
            && fabs(stats->statuses[5].speed) < 0.01)
        {
#ifdef DEBUG_EXECUTION
            std::cout << "Probably grasped!" << std::endl;
#endif
            done = true;
        }

        if (done && current_command_index < current_plan.size()-1)
        {
#ifdef DEBUG_EXECUTION
            std::cout << "Finished command " << current_command_index
                      << " out of " << current_plan.size()
                      << std::endl;
#endif
            current_command_index++;
            if (task == GRASPING &&
                current_plan.at(current_command_index).size() == 1)
            {
                dynamixel_command_t hand;
                if (current_plan.at(current_command_index).at(0) == 0)
                {
                    current_hand_command = 112.f*DEG_TO_RAD;
                }
                else { current_hand_command = 45.f*DEG_TO_RAD; }
            }
            else
            {
                forward_command();
            }
        }
        else if (done &&
                 current_command_index == current_plan.size()-1)
        {
            if (task == EXECUTING && current_plan_type == GRASP)
            {
#ifdef DEBUG_EXECUTION
                std::cout << "Execution switching to GRASPING"
                          << std::endl;
#endif
                current_plan = plan_grasp(arm_status);
                current_command_index = 0;
                forward_command();
                grasped_obj_dim = target_obj_dim;
                collision_world::set_held_object(grasped_obj_dim);
                task = GRASPING;
            }
            else if (task == EXECUTING && current_plan_type == DROP)
            {
#ifdef DEBUG_EXECUTION
                std::cout << "Execution switching to DROPPING"
                          << std::endl;
#endif
                current_plan = plan_drop(arm_status);
                current_command_index = 0;
                forward_command();
                collision_world::clear_held_object();
                task = GRASPING;
            }
            else
            {
#ifdef DEBUG_EXECUTION
                std::cout << "Finished command "
                          << current_command_index
                          << " so the plan is done." << std::endl;
#endif

                current_command_index++;
                task = WAITING;
                lcm::LCM lcm;
                planner_response_t resp;
                resp.response_type = "EXECUTE";
                resp.finished = true;
                resp.success = true;
                resp.response_id = execute_cmd_id;
                resp.plan_size = current_plan.size();

                lcm.publish("PLANNER_RESPONSES", &resp);
                last_response = resp;
            }
        }

        dynamixel_command_list_t command;
        command.len = probcog_arm::get_num_joints() + 1;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            dynamixel_command_t c;
            c.position_radians = current_command.at(i);
#ifdef SLOW_SPEED // Will override Soar's requests
            c.speed = probcog_arm::get_default_speed(i)*0.1;
#else
            c.speed = (MIN_PROP_SPEED + (1.f - MIN_PROP_SPEED) *
                       requested_speed) *
                probcog_arm::get_default_speed(i);
#endif
            c.max_torque = probcog_arm::get_default_torque(i);
            command.commands.push_back(c);
        }

        dynamixel_command_t hand;
        hand.position_radians = current_hand_command;
#ifdef SLOW_SPEED // Will override Soar's requests
        hand.speed = 0.015;
#else
        hand.speed = (MIN_PROP_SPEED + (1.f - MIN_PROP_SPEED) *
                       requested_speed) * 0.15;
#endif

        hand.max_torque = 0.5;
        command.commands.push_back(hand);
        lcm.publish("ARM_COMMAND", &command);
    }
}

void planner_interface::handle_observations_message(
    const lcm::ReceiveBuffer* rbuf,
        const std::string& channel,
        const observations_t* obs)
{
    if (planning()) return;

    latest_objects = obs->observations;
    collision_world::clear();
    for (std::vector<object_data_t>::iterator i =
             latest_objects.begin();
         i != latest_objects.end(); i++)
    {
        object_data od;
        od.id = i->id;
        od.type = "block";
        for (int j = 0; j < i->num_cat; j++)
        {
            if (i->cat_dat[j].cat.cat == 1)
            {
                od.color = "not sure yet";
                for (int k = 0; k < i->cat_dat[j].len; k++)
                {
                    if (i->cat_dat[j].label[k].compare("red") == 0 &&
                        i->cat_dat[j].confidence[k] > 0.9)
                    {
                        od.color = "red";
                        break;
                    }
                    else if (i->cat_dat[j].label[k].compare("green") == 0 &&
                             i->cat_dat[j].confidence[k] > 0.9)
                    {
                        od.color = "green";
                        break;
                    }
                    else if (i->cat_dat[j].label[k].compare("blue") == 0 &&
                             i->cat_dat[j].confidence[k] > 0.4)
                    {
                        od.color = "blue";
                        break;
                    }
                }
            }
        }
        collision_world::add_object(i->bbox_dim, i->bbox_xyzrpy, od);
    }
}
