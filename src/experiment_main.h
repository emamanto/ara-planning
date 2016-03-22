#include <boost/timer/timer.hpp>
#include "ProbCogSearchStates.h"
#include "Search.h"
#include "ProbCogArmCollision.h"
#include "Shortcut.h"
#include "dynamixel_status_list_t.hpp"
#include "dynamixel_command_list_t.hpp"
#include "observations_t.hpp"

enum stage{REACH, GRASP, MOVE, DROP};
enum status{SEARCH, EXECUTE, WAIT};

class experiment_handler
{
public:
    experiment_handler(int obj_id,
                       std::string color,
                       float drop_target_x,
                       float drop_target_y,
                       float time_lim,
                       float step_size_deg,
                       bool first_sol,
                       bool reset = true);
    ~experiment_handler() {};

    void handle_status_message(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const dynamixel_status_list_t* stats);
    void handle_observations_message(const lcm::ReceiveBuffer* rbuf,
                                     const std::string& channel,
                                     const observations_t* obs);
private:
    void check_collisions();
    void publish_command();
    void compute_next_plan();
    void compute_grasp_plan();
    void compute_drop_plan();
    void set_reach_point();

    bool motion_done();
    void request_hand_motion(bool opening);
    void print_stage(stage s);

    pose apos;
    pose dpos;
    float ahand;
    float dhand;
    float hand_speed;
    bool holding_object;
    std::vector<action> current_plan;
    int plan_index;

    std::vector<int> collision_ids;
    int stage_collision_time;
    collision_world ground_truth;
    collision_world observed;
    std::vector<object_data_t> latest_objects;
    std::vector<object_data_t> latest_observations;

    std::vector<double> current_distances;
    double current_min_distance;

    int target_obj_id;
    std::string target_obj_color;
    std::vector<float> target_obj_dim;
    float drop_x;
    float drop_y;
    float drop_z;

    float search_time_lim;
    float search_step_size;
    bool search_first_sol;

    stage current_stage;
    status current_status;

    boost::timer::cpu_timer execution_timer;

    int observe_time;
};
