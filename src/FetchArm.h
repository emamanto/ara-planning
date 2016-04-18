#pragma once

#include <vector>
#include <Eigen/Dense>

#define DEG_TO_RAD M_PI/180.f

enum axis { X_AXIS, Y_AXIS, Z_AXIS };
enum joint_type { REVOLUTE, CONTINUOUS, PRISMATIC };

struct joint
{
    joint_type type;
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
typedef std::vector<float> orientation;
typedef std::vector<float> action;

action subtract(pose& end, pose& begin);
point_3d translation_from_xform(Eigen::Matrix4f xform);
orientation eulers_from_xform(Eigen::Matrix4f xform);

class fetch_arm
{
public:
    static void INIT();

    static int get_num_joints() { return num_joints; }

    static point_3d joint_xyz(int joint_number, pose p);
    static Eigen::Matrix4f joint_transform(int joint_number, pose p);

    static Eigen::Matrix4f rotation_matrix(float angle, axis around);
    static Eigen::Matrix4f translation_matrix(float x,
                                              float y,
                                              float z);

    static point_3d ee_xyz(pose p);
    static orientation ee_rpy(pose p);

    static Eigen::Matrix4f ee_xform(pose p);

    static float ee_dist_to(pose from, point_3d to);

    static action solve_ik(pose from, Eigen::Matrix4f xform);

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

    static pose apply(pose& from, action& act);
    static pose apply(pose& from, std::vector<action>& plan);

    static void fast_apply(pose& from, action& act, pose& result);

    static void set_primitive_change(float big_rad);
    static std::vector<action>& big_primitives()
    { return big_prims; }
    static std::vector<action>& small_primitives()
    { return small_prims; }

    static bool is_valid(pose p);

    static float hand_length;
    static float hand_width;
    static float hand_height;
    static float finger_width;

    static point_3d base_offset;

private:
    fetch_arm() {};
    fetch_arm(fetch_arm const&) {};
    fetch_arm& operator=(fetch_arm const&) {};

    static int num_joints;
    static std::vector<joint> configuration;
    static std::vector<action> big_prims;
    static std::vector<action> small_prims;
};
