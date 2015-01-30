#include "Arm.h"
#include <math.h>

#define DEG_TO_RAD M_PI/180.f

using namespace std;

Arm::Arm(int num_joints) : num_joints(num_joints),
                           component_lengths(num_joints,
                                             ARM_LENGTH/num_joints),
                           current_angles(num_joints, 0.f),
                           max_angles(num_joints, 180.f),
                           min_angles(num_joints, -180.f)
{
    min_angles.at(0) = 0.f;
}

Arm::Arm(vector<float> components) : num_joints(components.size()),
                                     current_angles(num_joints, 0.f),
                                     max_angles(num_joints, 180.f),
                                     min_angles(num_joints, -180.f)
{
    min_angles.at(0) = 0.f;
    float sum = 0;
    for(vector<float>::iterator i = components.begin();
        i != components.end(); i++)
    {
        sum += *i;
    }

    for(vector<float>::iterator i = components.begin();
        i != components.end(); i++)
    {
        component_lengths.push_back((*i/sum)*ARM_LENGTH);
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

float Arm::get_joint_max(int joint_number)
{
    return max_angles.at(joint_number);
}

float Arm::get_joint_min(int joint_number)
{
    return min_angles.at(joint_number);
}

vector<float> Arm::get_joints()
{
    return current_angles;
}

vector<float> Arm::get_components()
{
    return component_lengths;
}

vector<float> Arm::get_max_angles()
{
    return max_angles;
}

vector<float> Arm::get_min_angles()
{
    return min_angles;
}

bool Arm::set_joints(std::vector<float> angles)
{
    current_angles = angles;
}

bool Arm::set_joint(int joint_number, float angle)
{
    current_angles.at(joint_number) = angle;
}

float Arm::get_ee_x()
{
    float x = 0.f;
    float angle_sum = 0.f;
    for(int i = 0; i < num_joints; i++)
    {
        angle_sum += current_angles.at(i);
        x += component_lengths.at(i)*cos((angle_sum)*DEG_TO_RAD);
    }
    return x;
}

float Arm::get_ee_y()
{
    float y = 0.f;
    float angle_sum = 0.f;
    for(int i = 0; i < num_joints; i++)
    {
        angle_sum += current_angles.at(i);
        y += component_lengths.at(i)*sin((angle_sum)*DEG_TO_RAD);
    }
    return y;
}

bool Arm::is_valid(vector<float> joint_config)
{
    bool valid = true;
    for(int i = 0; i < num_joints; i++)
    {
        if (joint_config.at(i) > max_angles.at(i) ||
            joint_config.at(i) < min_angles.at(i))
        {
            valid = false;
            break;
        }
    }
    return valid;
}

bool Arm::is_currently_valid()
{
    bool valid = true;
    for(int i = 0; i < num_joints; i++)
    {
        if (current_angles.at(i) > max_angles.at(i) ||
            current_angles.at(i) < min_angles.at(i))
        {
            valid = false;
            break;
        }
    }
    return valid;
}
