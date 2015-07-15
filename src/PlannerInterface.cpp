#include "PlannerInterface.h"

#define PUBLISH_COLLISION_MODEL

pthread_t planner_interface::thrd = pthread_t();

float planner_interface::PRIMITIVE_SIZE_MIN = 2.f;
float planner_interface::PRIMITIVE_SIZE_MAX = 20.f;
float planner_interface::MIN_PROP_SPEED = 0.2f;

planner_interface::planner_interface() :
    latest_plan_executed(false),
    latest_plan_smoothed(false),
    arm_status(probcog_arm::get_num_joints(), 0),
    task(WAITING_INITIAL),
    current_command_index(0),
    requested_speed(1)
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
    latest_plan_executed = false;
    latest_plan_smoothed = false;
    lcm::LCM lcm;
    planner_response_t resp;
    resp.response_type = "SEARCH";
    resp.finished = true;
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

    if (comm->command_type.compare("SEARCH") == 0 &&
        task != SEARCHING)
    {
        task = SEARCHING;
        point_3d goal;
        for (int i = 0; i < 3; i++)
        {
            goal.push_back(comm->target[i]);
        }
        arm_state::target = goal;

        float big_prim_size = (comm->primitive_size)*
            (PRIMITIVE_SIZE_MAX - PRIMITIVE_SIZE_MIN) +
            PRIMITIVE_SIZE_MIN;
        std::cout << "Set the primitive to " << big_prim_size
                  << std::endl;
        probcog_arm::set_primitive_change(big_prim_size);

        std::cout << "Initiating a search to " << goal[0]
                  << ", " << goal[1] << ", " << goal[2]
                  << std::endl;
        latest_start_pose = arm_status;

        latest_search.clear();
        latest_request =
            search_request<arm_state, action>(arm_state(latest_start_pose),
                                              probcog_arm::big_primitives(),
                                              probcog_arm::small_primitives());

        pthread_create(&thrd, NULL, &search_thread, this);
    }
    else if (comm->command_type.compare("STOP") == 0 &&
             task != WAITING)
    {
        std::cout << "Stopping a search!" << std::endl;
        latest_request.kill();
    }
    else if (comm->command_type.compare("PAUSE") == 0 &&
             task != PAUSED)
    {
        std::cout << "Pausing the search!" << std::endl;
        latest_request.pause();
        task = PAUSED;
    }
    else if (comm->command_type.compare("CONTINUE") == 0 &&
             task == PAUSED)
    {
        std::cout << "Resuming the search!" << std::endl;
        latest_request.unpause();
        task = SEARCHING;
    }
    else if (comm->command_type.compare("POSTPROCESS") == 0 &&
             task != POSTPROCESSING && !latest_plan_smoothed)
    {
        std::cout << "Going to smooth the existing path!" << std::endl;
        task = POSTPROCESSING;
        // SHORTCUT **Add actually using the parameter in the msg
        std::cout << "Originally: " << current_plan.size()
                  << std::endl;
        current_plan =
            shortcut<arm_state, action>(current_plan,
                                        arm_state(latest_start_pose));
        std::cout << "Ultimate path length: " << current_plan.size()
                  << std::endl;
        latest_plan_smoothed = true;
        task = WAITING;
    }
    else if (comm->command_type.compare("RESET") == 0 &&
             task != EXECUTING)
    {
        task = EXECUTING;
        std::cout << "Resetting the arm." << std::endl;
        current_command = pose(probcog_arm::get_num_joints(), 0);
        current_plan.clear();
        current_command_index = -1;
    }
    else if (comm->command_type.compare("EXECUTE") == 0 &&
             task != EXECUTING && !latest_plan_executed)
    {
        // current_plan = shortcut<arm_state, action>(current_plan,
        //                                            arm_state(status));
        // std::cout << "Shortcutted to " << current_plan.size()
        //           << std::endl;

        current_command = arm_status;
        for (int i = 0; i < probcog_arm::get_num_joints(); i++)
        {
            current_command.at(i) += current_plan.at(0).at(i);
        }
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

    if (task == EXECUTING)
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

        if (done && current_command_index < current_plan.size()-1)
        {
            current_command_index++;
            for (int i = 0; i < probcog_arm::get_num_joints(); i++)
            {
                current_command.at(i) +=
                    current_plan.at(current_command_index).at(i);
            }
        }
        if (done && current_command_index == current_plan.size()-1)
        {
            current_command_index++;
            task = WAITING;
            latest_plan_executed = true;
            lcm::LCM lcm;
            planner_response_t resp;
            resp.response_type = "EXECUTE";
            resp.finished = true;
            resp.success = true;
            if (!latest_search.empty()){
                resp.plan_size =
                    latest_search.at(latest_search.size()-1).path.size();
            }
            else resp.plan_size = 0;

            lcm.publish("PLANNER_RESPONSES", &resp);
            last_response = resp;
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
        hand.position_radians = 112.f*DEG_TO_RAD;
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
    for (std::vector<object_data_t>::iterator i =
             latest_objects.begin();
         i != latest_objects.end(); i++)
    {
        collision_world::add_object(i->bbox_dim, i->bbox_xyzrpy);
    }
}
