#include "PlannerInterface.h"

pthread_t planner_interface::thrd = pthread_t();

planner_interface::planner_interface() :
    arm_status(probcog_arm::get_num_joints(), 0),
    kill_search(false)
{
}

void* planner_interface::search_thread(void* arg)
{
    planner_interface* pi = static_cast<planner_interface*>(arg);
    arastar<arm_state, action>(&(pi->latest_search),
                               &(pi->kill_search),
                               arm_state(pi->latest_start_pose),
                               probcog_arm::big_primitives(),
                               probcog_arm::small_primitives(),
                               100.f);
    pi->search_complete();
}

void planner_interface::search_complete()
{
    lcm::LCM lcm;
    planner_response_t resp;
    if (latest_search.size() > 1)
    {
        resp.plan_found = true;
    }
    else
    {
        resp.plan_found = false;
    }
    resp.plan_size = latest_search.at(latest_search.size()-1).path.size();

    lcm.publish("PLANNER_RESPONSES", &resp);
}

void planner_interface::handle_command_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const planner_command_t* comm)
{
    std::cout << "Received a command message." << std::endl;
    if (comm->command_type.compare("SEARCH") == 0)
    {
        point_3d goal;
        for (int i = 0; i < 3; i++)
        {
            goal.push_back(comm->target[i]);
        }
        arm_state::target = goal;

        std::cout << "Initiating a search!" << std::endl;
        latest_start_pose = arm_status;
        kill_search = false;

        latest_search.clear();
        pthread_create(&thrd, NULL, &search_thread, this);
    }
    else if (comm->command_type.compare("STOP") == 0)
    {
        std::cout << "Stopping a search!" << std::endl;
        kill_search = true;
    }
    else if (comm->command_type.compare("EXECUTE") == 0)
    {
        std::cout << "Time to execute!" << std::endl;
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

#ifdef PUBLISH_COLLISION_MODEL
    lcm::LCM lcm;
    arm_collision_boxes_t arm_msg = collision_world::arm_boxes(status);
    lcm.publish("ARM_COLLISION_BOXES", &arm_msg);
#endif
}

void planner_interface::handle_observations_message(
    const lcm::ReceiveBuffer* rbuf,
        const std::string& channel,
        const observations_t* obs)
{
    latest_objects = obs->observations;
    collision_world::clear();
    for (std::vector<object_data_t>::iterator i =
             latest_objects.begin();
         i != latest_objects.end(); i++)
    {
        collision_world::add_object(i->bbox_dim, i->bbox_xyzrpy);
    }
}
