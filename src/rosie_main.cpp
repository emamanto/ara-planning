#include "PlannerInterface.h"

int main(int argc, char* argv[])
{
    lcm::LCM lcm;
    if (!lcm.good())
    {
        std::cout << "Failed to initialize LCM." << std::endl;
        return 1;
    }

    probcog_arm::INIT();

    planner_interface pi;
    lcm.subscribe("ARM_STATUS",
                  &planner_interface::handle_status_message,
                  &pi);
    lcm.subscribe("OBSERVATIONS",
                  &planner_interface::handle_observations_message,
                  &pi);

    while(0 == lcm.handle());

    return 0;
}
