#include "PlannerInterface.h"

void* planner_interface::search_thread(void* arg)
{
    planner_interface* pi = static_cast<planner_interface*>(arg);
    arastar<arm_state, action>(&(pi->latest_search),
                               &(pi->kill_search),
                               arm_state(pi->latest_start_pose),
                               probcog_arm::big_primitives(),
                               probcog_arm::small_primitives(),
                               100.f);
}

void planner_interface::handle_command_message(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const planner_command_t* comm)
{
    if (comm->command_type.compare("SEARCH"))
    {
        latest_start_pose = arm_status;
    }
    else if (comm->command_type.compare("STOP"))
    {
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
