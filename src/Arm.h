#pragma once

#include <vector>

#define ARM_LENGTH 300.f

typedef std::vector<float> pose;
typedef std::vector<float> length_config;
typedef std::vector<pose> plan;

class Arm
{
public:
    Arm(int num_joints);
    Arm(length_config components);

    int get_num_joints() {return num_joints;}
    float get_joint(int joint_number);
    float get_component(int component_number);
    float get_joint_max(int joint_number);
    float get_joint_min(int joint_number);
    pose get_joints();
    length_config get_components();
    pose get_max_angles();
    pose get_min_angles();

    bool set_joints(pose angles);
    bool set_joint(int joint_number, float angle);

    float get_ee_x();
    float get_ee_y();

    float get_ee_x_at(pose position);
    float get_ee_y_at(pose position);

    bool is_valid(pose joint_config);
    bool is_currently_valid();

private:
    int num_joints;
    length_config component_lengths;
    pose current_angles;
    pose max_angles;
    pose min_angles;
};
