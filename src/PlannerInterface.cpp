#include "PlannerInterface.h"

#define PUBLISH_COLLISION_MODEL

pthread_t planner_interface::thrd = pthread_t();

float planner_interface::PRIMITIVE_SIZE_MIN = 2.f;
float planner_interface::PRIMITIVE_SIZE_MAX = 20.f;
float planner_interface::MIN_PROP_SPEED = 0.2f;

planner_interface::planner_interface() :
    arm_status(probcog_arm::get_num_joints(), 0),
    search_cmd_id(-1),
    execute_cmd_id(-1),
    add_grasp(false),
    add_drop(false),
    task(WAITING_INITIAL),
    current_command_index(0),
    requested_speed(1),
    last_id_handled(-1)
{
}

void* planner_interface::search_thread(void* arg)
{
    planner_interface* pi = static_cast<planner_interface*>(arg);
    std::cout << "Searching" << std::endl;
    arastar<arm_state, action>(pi->latest_request);
    pi->search_complete();
}

void planner_interface::search_complete()
{
    lcm::LCM lcm;
    planner_response_t resp;
    resp.response_type = "PLAN";
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
        arm_state::target[2] = xyzrpy[2] + dim[2]/2.f + 0.03;
    }
    else if (xyzrpy[2] < 0.2)
    {
        arm_state::target_pitch = -M_PI/2.f;
        arm_state::target[0] = xyzrpy[0];
        arm_state::target[1] = xyzrpy[1];
        arm_state::target[2] = xyzrpy[2] + dim[2]/2.f + 0.03;
    }
    else
    {
        arm_state::target_pitch = 0.f;
        arm_state::target[2] = xyzrpy[2];

        if (fabs(xyzrpy[0]) < 0.01)
        {
            arm_state::target[0] = 0;
            float offset = dim[1]/2.f + 0.04;
            if (xyzrpy[1] < 0) offset *= -1;
            arm_state::target[1] = xyzrpy[1] - offset;
        }
        else if (fabs(xyzrpy[1]) < 0.01)
        {
            arm_state::target[1] = 0;
            float offset = dim[0]/2.f + 0.04;
            if (xyzrpy[0] < 0) offset *= -1;
            arm_state::target[0] = xyzrpy[0] - offset;
        }
        else if (fabs(xyzrpy[0]) > fabs(xyzrpy[1]))
        {
            float offset = dim[0]/2.f + 0.04;
            if (xyzrpy[0] < 0) offset *= -1;
            arm_state::target[0] = xyzrpy[0] - offset;
            arm_state::target[1] = (arm_state::target[0] *
                                    xyzrpy[1]) / xyzrpy[0];
        }
        else
        {
            float offset = dim[1]/2.f + 0.04;
            if (xyzrpy[1] < 0) offset *= -1;
            arm_state::target[1] = xyzrpy[1] - offset;
            arm_state::target[0] = (arm_state::target[1] *
                                    xyzrpy[0]) / xyzrpy[1];
        }
    }
}

std::vector<action> planner_interface::plan_grasp(pose start,
                                                  bool is_drop)
{
    std::vector<action> plan;

    // Side grab
    if (fabs(probcog_arm::ee_pitch(start)) < 0.01)
    {
        // Spin wrist
        action spin(probcog_arm::get_num_joints(), 0);
        spin.at(probcog_arm::get_num_joints()-1) = M_PI/4.f;
        plan.push_back(spin);

        action hand_open(1, 1);
        plan.push_back(hand_open);

        // Forward into obj (?) XXX
        // This seems like I might want to store the grasp point
        // calculation instead of basically reversing it here...
    }
    // Straight down grab
    else if (fabs(probcog_arm::ee_pitch(start) - -M_PI/2) < 0.01)
    {
        // Spin wrist
        action spin(probcog_arm::get_num_joints(), 0);
        spin.at(probcog_arm::get_num_joints()-1) =
            target_obj_xyzrpy[5];
        plan.push_back(spin);

        action hand_open(1, 1);
        plan.push_back(hand_open);

        // Down onto obj (?)
        if (!is_drop)
        {
            point_3d end = probcog_arm::ee_xyz(start);
            end.at(2) = end.at(2) - 0.04;
            action down = probcog_arm::solve_ik(start, end);
            plan.push_back(down);
        }
    }

    if (!is_drop)
    {
        action hand_close(1, 0);
        plan.push_back(hand_close);
    }
    return plan;
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

    if (comm->command_type.compare("PLAN") == 0 &&
        comm->command_id > last_id_handled)
    {
        task = SEARCHING;
        if (comm->plan_type.compare("GRASP") == 0)
        {
            set_grasp_target(target_obj_dim, target_obj_xyzrpy);
            add_grasp = true;
            add_drop = false;
        }
        else if (comm->plan_type.compare("DROP") == 0)
        {
            double drop_point[6];
            drop_point[0] = comm->target[0];
            drop_point[1] = comm->target[1];
            drop_point[2] = comm->target[2];
            for (int i = 3; i < 6; i++) drop_point[i] = 0;
            set_grasp_target(grasped_obj_dim, drop_point);
            add_grasp = false;
            add_drop = true;
        }
        else
        {
            point_3d goal;
            for (int i = 0; i < 3; i++)
            {
                goal.push_back(comm->target[i]);
            }
            arm_state::target = goal;
            arm_state::pitch_matters = false;
            add_grasp = false;
            add_drop = false;
        }

        float big_prim_size = (comm->primitive_size)*
            (PRIMITIVE_SIZE_MAX - PRIMITIVE_SIZE_MIN) +
            PRIMITIVE_SIZE_MIN;
        std::cout << "Set the primitive to " << big_prim_size
                  << std::endl;
        probcog_arm::set_primitive_change(big_prim_size);

        std::cout << "Initiating a search to "
                  << arm_state::target[0] << ", "
                  << arm_state::target[1] << ", "
                  << arm_state::target[2] << ", pitch "
                  << arm_state::target_pitch
                  << std::endl;
        latest_start_pose = arm_status;

        latest_search.clear();
        latest_request =
            search_request<arm_state, action>(arm_state(latest_start_pose),
                                              probcog_arm::big_primitives(),
                                              probcog_arm::small_primitives());
        last_id_handled = comm->command_id;
        search_cmd_id = comm->command_id;
        pthread_create(&thrd, NULL, &search_thread, this);
    }
    else if (comm->command_type.compare("STOP") == 0 &&
             comm->command_id > last_id_handled)
    {
        std::cout << "Stopping a search!" << std::endl;
        latest_request.kill();
        last_id_handled = comm->command_id;
        // RESPONSE
    }
    else if (comm->command_type.compare("PAUSE") == 0 &&
             comm->command_id > last_id_handled)
    {
        std::cout << "Pausing the search!" << std::endl;
        latest_request.pause();
        last_id_handled = comm->command_id;
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
        std::cout << "Resuming the search!" << std::endl;
        latest_request.unpause();
        last_id_handled = comm->command_id;
        task = SEARCHING;

        lcm::LCM lcm;
        planner_response_t resp;
        resp.response_type = "PAUSE";
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
        std::cout << "Going to smooth the existing path!" << std::endl;
        task = POSTPROCESSING;
        // SHORTCUT **Add actually using the parameter in the msg
        int original = current_plan.size();
        std::cout << "Originally: " << original << std::endl;
        current_plan =
            shortcut<arm_state, action>(current_plan,
                                        arm_state(latest_start_pose));
        std::cout << "Ultimate path length: " << current_plan.size()
                  << std::endl;
        last_id_handled = comm->command_id;

        lcm::LCM lcm;
        planner_response_t resp;
        resp.response_type = "POSTPROCESS";
        resp.response_id = comm->command_id;
        resp.finished = true;
        resp.success = (current_plan.size() < original);
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
        std::cout << "Resetting the arm." << std::endl;
        current_command = pose(probcog_arm::get_num_joints(), 0);
        current_plan.clear();
        current_command_index = -1;
        add_grasp = false;
        add_drop = false;
    }
    else if (comm->command_type.compare("EXECUTE") == 0 &&
             comm->command_id > last_id_handled)
    {
        last_id_handled = comm->command_id;
        execute_cmd_id = comm->command_id;
        current_command = arm_status;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            current_command.at(i) += current_plan.at(0).at(i);
        }
        current_hand_command = 112.f*DEG_TO_RAD;
        current_command_index = 0;
        requested_speed = comm->speed;

        task = EXECUTING;
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
#ifdef PUBLISH_COLLISION_MODEL
    if (task != SEARCHING)
    {
        arm_collision_boxes_t arm_msg = collision_world::arm_boxes(arm_status);
        lcm.publish("ARM_COLLISION_BOXES", &arm_msg);
    }
#endif

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
        if (fabs(current_hand_command - hand_status) > 0.01
            && !add_drop)
        {
            done = false;
        }
        if (current_command_index >= 0 &&
            current_plan.at(current_command_index).size() == 1
            && fabs(stats->statuses[5].speed) < 0.01)
        {
            done = true;
        }

        if (done && current_command_index < current_plan.size()-1)
        {
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
                for (int i = 0;
                     i < probcog_arm::get_num_joints(); i++)
                {
                    current_command.at(i) +=
                        current_plan.at(current_command_index).at(i);
                }
            }
        }
        else if (done &&
                 current_command_index == current_plan.size()-1)
        {
            if (task == EXECUTING && add_grasp)
            {
                std::cout << "Execution switching to GRASPING"
                          << std::endl;
                current_plan = plan_grasp(arm_status, false);
                current_command_index = 0;
                grasped_obj_dim = target_obj_dim;
                collision_world::set_held_object(grasped_obj_dim);
                task = GRASPING;
            }
            else if (task == EXECUTING && add_drop)
            {
                std::cout << "Execution switching to DROPPING"
                          << std::endl;
                current_plan = plan_grasp(arm_status, true);
                current_command_index = 0;
                collision_world::clear_held_object();
                task = GRASPING;
            }
            else
            {
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
        hand.speed = (MIN_PROP_SPEED + (1.f - MIN_PROP_SPEED) *
                       requested_speed) * 0.15;
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
    if (task == SEARCHING) return;

    latest_objects = obs->observations;
    collision_world::clear();
    float height = 10;
    for (std::vector<object_data_t>::iterator i =
             latest_objects.begin();
         i != latest_objects.end(); i++)
    {
        collision_world::add_object(i->bbox_dim, i->bbox_xyzrpy);
        if (i->bbox_dim[2] < height)
        {
            height = i->bbox_dim[2];
            target_obj_dim = i->bbox_dim;
            target_obj_xyzrpy = i->bbox_xyzrpy;
        }
    }
}
