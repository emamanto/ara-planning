#include "ProbCogSearchStates.h"
#include "Search.h"
#include "ProbCogArmCollision.h"
#include "Shortcut.h"
#include "dynamixel_status_list_t.hpp"
#include "dynamixel_command_list_t.hpp"
#include "observations_t.hpp"

class experiment_handler
{
public:
    experiment_handler(int obj_id,
                       std::string color,
                       float drop_target_x,
                       float drop_target_y);
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

    pose apos;
    pose dpos;
    float ahand;
    float dhand;
    std::vector<pose> current_plan;
    int plan_index;
    bool searching;
    int num_collisions;
    std::vector<object_data_t> latest_objects;

    int target_obj_id;
    std::string target_obj_color;
    float drop_x;
    float drop_y;

    bool picked_up;
    bool moved_drop;
    bool dropped_off;

    int observe_time;
};
