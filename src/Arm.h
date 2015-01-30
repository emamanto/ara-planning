#pragma once

#include <vector>

class Arm
{
public:
    Arm(int num_joints);
    Arm(std::vector<float> components);

    int get_num_joints() {return num_joints;}
    float get_joint(int joint_number);
    float get_component(int component_number);
    std::vector<float> get_joints();
    std::vector<float> get_components();

    float get_ee_x();
    float get_ee_y();

private:
    int num_joints;
    std::vector<float> component_lengths;
    std::vector<float> current_angles;
};
