#pragma once

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <pthread.h>

#include "ProbCogSearchStates.h"
#include "Search.h"
#include "ProbCogArmCollision.h"
#include "Shortcut.h"

#include "planner_command_t.hpp"
#include "planner_response_t.hpp"
#include "dynamixel_status_list_t.hpp"
#include "observations_t.hpp"

typedef std::vector<search_result<arm_state, action> > arastar_result;

class planner_interface
{
public:
    planner_interface();

    void handle_command_message(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel,
                                const planner_command_t* comm);

    void handle_status_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const dynamixel_status_list_t* stats);
    void handle_observations_message(const lcm::ReceiveBuffer* rbuf,
                                     const std::string& channel,
                                     const observations_t* obs);

private:
    static void* search_thread(void* arg);
    void search_complete();

    static pthread_t thrd;
    std::vector<object_data_t> latest_objects;
    pose latest_start_pose;
    pose arm_status;
    arastar_result latest_search;
    bool kill_search;
};
