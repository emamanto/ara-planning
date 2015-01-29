#include "Arm.h"

using namespace std;

Arm::Arm(int num_joints) : num_joints(num_joints),
                           component_lengths(num_joints,
                                             100.f/num_joints),
                           current_angles(num_joints, 0.f)
{
    current_angles.at(0) = PI/6.f;
    current_angles.at(1) = 5*PI/3.f;
}

Arm::Arm(vector<float> components) : num_joints(components.size()),
                                     current_angles(num_joints, 0.f)
{
    float sum = 0;
    for(vector<float>::iterator i = components.begin();
        i != components.end(); i++)
    {
        sum += *i;
    }

    for(vector<float>::iterator i = components.begin();
        i != components.end(); i++)
    {
        component_lengths.push_back((*i/sum)*100);
    }
}

float Arm::get_joint(int joint_number)
{
    return current_angles.at(joint_number);
}

float Arm::get_component(int joint_number)
{
    return component_lengths.at(joint_number);
}

vector<float> Arm::get_joints()
{
    return current_angles;
}

vector<float> Arm::get_components()
{
    return component_lengths;
}
