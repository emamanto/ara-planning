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
                       bool reset);
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
    void set_reach_point();

    bool motion_done();
    void request_hand_motion(bool opening);
    void print_stage(stage s);

    pose apos;
    pose dpos;
    float ahand;
    float dhand;
    std::vector<pose> current_plan;
    int plan_index;
    int num_collisions;
    std::vector<object_data_t> latest_objects;

    int target_obj_id;
    std::string target_obj_color;
    float drop_x;
    float drop_y;

    stage current_stage;
    status current_status;

    int observe_time;
};
