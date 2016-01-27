#include "ProbCogArmCollision.h"
#include "ProbCogSearchStates.h"
#include <iostream>

// Parameter for fiddling
#define TIME_PER_STEP 0.01

template<>
bool subdivision_collision_check<arm_state, action>(arm_state start,
                                                    action action,
                                                    int depth)
{
    std::vector<float> joint_mvmt_times;
    float max_time = 0;
    int max_joint = 0;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        joint_mvmt_times.push_back(fabs(action.at(i))/
                                   fetch_arm::get_default_speed(i));
        if (joint_mvmt_times.at(i) > max_time)
        {
            max_time = joint_mvmt_times.at(i);
            max_joint = i;
        }
    }

    std::vector<float> chgs_per_step;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        chgs_per_step.push_back(TIME_PER_STEP*
                                fetch_arm::get_default_speed(i));
    }

    // END point of the motion
    pose chkpoint = fetch_arm::apply(start.position, action);
    float time = max_time;

    while(time > TIME_PER_STEP)
    {
        time -= TIME_PER_STEP;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            if (time > joint_mvmt_times.at(i)) continue;

            if ((time + TIME_PER_STEP) > joint_mvmt_times.at(i))
            {
                int ns = floor(joint_mvmt_times.at(i)/TIME_PER_STEP);
                float extra = (joint_mvmt_times.at(i) -
                               (TIME_PER_STEP*ns));
                if (action.at(i) > 0)
                {
                    chkpoint.at(i) -= extra;
                }
                else
                {
                    chkpoint.at(i) += extra;
                }
            }
            else
            {
                if (action.at(i) > 0)
                {
                    chkpoint.at(i) -= chgs_per_step.at(i);
                }
                else
                {
                    chkpoint.at(i) += chgs_per_step.at(i);
                }
            }
            if (!arm_state(chkpoint).valid())
            {
                return true;
            }
        }

    }

    return false;
}

