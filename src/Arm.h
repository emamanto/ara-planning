#pragma once

#include <vector>

#define ARM_LENGTH 300.f

class Arm
{
public:
    Arm(int num_joints);
    Arm(std::vector<float> components);

    int get_num_joints() {return num_joints;}
    float get_joint(int joint_number);
    float get_component(int component_number);
    float get_joint_max(int joint_number);
    float get_joint_min(int joint_number);
    std::vector<float> get_joints();
    std::vector<float> get_components();
    std::vector<float> get_max_angles();
    std::vector<float> get_min_angles();

    bool set_joints(std::vector<float> angles);
    bool set_joint(int joint_number, float angle);

    float get_ee_x();
    float get_ee_y();

    bool is_valid(std::vector<float> joint_config);
    bool is_currently_valid();

private:
    int num_joints;
    std::vector<float> component_lengths;
    std::vector<float> current_angles;
    std::vector<float> max_angles;
    std::vector<float> min_angles;
};
