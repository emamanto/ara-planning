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
#include "dynamixel_command_list_t.hpp"
#include "observations_t.hpp"

typedef std::vector<search_result<arm_state, action> > arastar_result;

enum planner_status{ PLANNING_GRASP,
                     PLANNING_DROP,
                     PLANNING_MOVE,
                     POSTPROCESSING,
                     EXECUTING,
                     GRASPING,
                     WAITING,
                     PAUSED,
                     WAITING_INITIAL,
                     NONE };

enum plan_type{ MOVE, GRASP, DROP };

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
    void set_grasp_target(double dim[], double xyzrpy[]);
    std::vector<action> plan_grasp(pose start);
    std::vector<action> plan_drop(pose start);

    void forward_command();

    bool planning() {return (task == PLANNING_GRASP ||
                             task == PLANNING_DROP ||
                             task == PLANNING_MOVE); }

    static pthread_t thrd;
    planner_status task;
    planner_status paused_task;

    // Observations
    std::vector<object_data_t> latest_objects;
    double* target_obj_dim;
    double* grasped_obj_dim;
    double* target_obj_xyzrpy;

    // Control
    pose arm_status;
    int current_command_index;
    pose current_command;
    float current_hand_command;
    float requested_speed;

    // LCM
    planner_response_t last_response;
    int last_id_handled;
    int search_cmd_id;
    int execute_cmd_id;

    // Latest search/plan
    pose latest_start_pose;
    arastar_result latest_search;
    search_request<arm_state, action> latest_request;
    std::vector<action> current_plan;
    plan_type current_plan_type;

    static float PRIMITIVE_SIZE_MIN, PRIMITIVE_SIZE_MAX;
    static float MIN_PROP_SPEED;
    static float GRASP_TOP_OFFSET, GRASP_INTO_OBJ_OFFSET;
};
