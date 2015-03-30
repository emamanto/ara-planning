#pragma once

#include <vector>
#include <Eigen/Dense>

enum axis { X_AXIS, Y_AXIS, Z_AXIS };

struct joint
{
    axis around;
    float min; // deg
    float max; // deg
    float length; // m
    float width; // m
    float default_speed;
    float default_torque;
};

typedef std::vector<float> pose;
typedef std::vector<float> point_3d;
typedef std::vector<float> action;

class probcog_arm
{
public:
    static void INIT();

    static int get_num_joints() { return num_joints; }

    static point_3d joint_xyz(int joint_number, pose p);
    static point_3d ee_xyz(pose p);
    static float ee_dist_to(pose from, point_3d to);

    static action solve_ik(pose from, point_3d to);

    static float get_joint_max(int joint_number)
    { return configuration.at(joint_number).max; }
    static float get_joint_min(int joint_number)
    { return configuration.at(joint_number).min; }
    static axis get_joint_axis(int joint_number)
    { return configuration.at(joint_number).around; }
    static float get_component_length(int joint_number)
    { return configuration.at(joint_number).length; }
    static float get_component_width(int joint_number)
    { return configuration.at(joint_number).width; }
    static float get_default_speed(int joint_number)
    { return configuration.at(joint_number).default_speed; }
    static float get_default_torque(int joint_number)
    { return configuration.at(joint_number).default_torque; }

    static pose apply(pose from, action act);

    static std::vector<action>& big_primitives()
    { return big_prims; }
    static std::vector<action>& small_primitives()
    { return small_prims; }

    static bool is_valid(pose p);

private:
    probcog_arm() {};
    probcog_arm(probcog_arm const&) {};
    probcog_arm& operator=(probcog_arm const&) {};

    static Eigen::Matrix4f joint_transform(int joint_number, pose p);
    static Eigen::Matrix4f rotation_matrix(float angle, axis around);
    static Eigen::Matrix4f translation_matrix(float x,
                                              float y,
                                              float z);

    static int num_joints;
    static float base_height;
    static float hand_length;
    static std::vector<joint> configuration;
    static std::vector<action> big_prims;
    static std::vector<action> small_prims;
};
