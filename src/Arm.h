#pragma once

#include <vector>

#define ARM_LENGTH 300.f

// Change joint by change degrees
struct action
{
    int joint;
    float change;

    action(int j, float c): joint(j), change(c) {}
    action(): joint(0), change(0) {}
    bool operator == (const action& other) const
    { return (joint == other.joint && change == other.change); }
    bool operator < (const action& other) const
    {return joint > other.joint; }
    bool operator > (const action& other) const
    {return joint < other.joint; }
};

struct line_segment
{
    float x1, y1;
    float x2, y2;

    line_segment() : x1(0), y1(0), x2(0), y2(0) {}
    line_segment(float a, float b, float c, float d) : x1 (a),
                                                       y1 (b),
                                                       x2 (c),
                                                       y2 (d) {}
};

typedef std::vector<float> pose;
typedef std::vector<float> length_config;
typedef std::vector<action> plan;

class Arm
{
public:
    static Arm* the_instance();

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

    float get_joint_x_at(int joint, pose position);
    float get_joint_y_at(int joint, pose position);

    bool is_valid(pose joint_config);
    bool is_currently_valid();

    line_segment arm_segment(int seg, pose position);
    bool intersect(int seg1, int seg2, pose position);
    bool intersect(line_segment seg1, line_segment seg2);

    bool apply(action a);
    bool apply(plan p);
    pose apply_at(action a, pose start);

    std::vector<action> get_primitives();
    void set_primitive_change(float c);

private:
    Arm();
    Arm(Arm const&) {};
    Arm& operator=(Arm const&) {};
    static Arm* instance;

    int num_joints;
    length_config component_lengths;
    pose current_angles;
    pose max_angles;
    pose min_angles;
    std::vector<action> primitives;
};
