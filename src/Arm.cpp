#include "Arm.h"
#include <math.h>
#include <iostream>

#define DEG_TO_RAD M_PI/180.f

std::vector<line_segment> obstacle::get_segments()
{
    std::vector<line_segment> segs;
    segs.push_back(line_segment(x, y, x+width, y));
    segs.push_back(line_segment(x, y, x, y-height));
    segs.push_back(line_segment(x+width, y, x+width, y-height));
    segs.push_back(line_segment(x, y-height, x+width, y-height));
    return segs;
}

Arm* Arm::instance = 0;

Arm* Arm::the_instance()
{
    if (!instance) instance = new Arm;
    return instance;
}

Arm::Arm() : num_joints(3),
             component_lengths(num_joints,
                               ARM_LENGTH/num_joints),
             current_angles(num_joints, 0.f),
             max_angles(num_joints, 180.f),
             min_angles(num_joints, -180.f)
{
    min_angles.at(0) = 0.f;
    set_primitive_change(8.f);
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

pose Arm::get_joints()
{
    return current_angles;
}

length_config Arm::get_components()
{
    return component_lengths;
}

pose Arm::get_max_angles()
{
    return max_angles;
}

pose Arm::get_min_angles()
{
    return min_angles;
}

bool Arm::set_joints(pose angles)
{
    if (angles.size() != num_joints)
    {
        std::cout << "ARM ERROR: Wrong number of joints."
                  << std::endl;
        return false;
    }
    current_angles = angles;
    return true;
}

bool Arm::set_joint(int joint_number, float angle)
{
    if (joint_number < 0 || joint_number >= num_joints)
    {
        std::cout << "ARM ERROR: Invalid joint number."
                  << std::endl;
        return false;
    }
    current_angles.at(joint_number) = angle;
    return true;
}

float Arm::get_ee_x_at(pose position)
{
    return get_joint_x_at(num_joints, position);
}

float Arm::get_ee_y_at(pose position)
{
    return get_joint_y_at(num_joints, position);
}

float Arm::get_joint_x_at(int joint, pose position)
{
    float x = 0.f;
    float angle_sum = 0.f;
    for(int i = 0; i < joint; i++)
    {
        angle_sum += position.at(i);
        x += component_lengths.at(i)*cos((angle_sum)*DEG_TO_RAD);
    }
    return x;
}

float Arm::get_joint_y_at(int joint, pose position)
{
    float y = 0.f;
    float angle_sum = 0.f;
    for(int i = 0; i < joint; i++)
    {
        angle_sum += position.at(i);
        y += component_lengths.at(i)*sin((angle_sum)*DEG_TO_RAD);
    }
    return y;
}

float Arm::get_ee_x()
{
    return get_ee_x_at(current_angles);
}

float Arm::get_ee_y()
{
    return get_ee_y_at(current_angles);
}

bool Arm::is_valid(pose joint_config)
{
    bool valid = true;

    for(int i = 0; i < num_joints; i++)
    {
        // Joints not over max or under min
        if (joint_config.at(i) > max_angles.at(i) ||
            joint_config.at(i) < min_angles.at(i))
        {
            return false;
        }

        // Table collision
        if (get_joint_y_at(i, joint_config) < 0)
        {
            return false;
        }

        //Self collision
        for (int j = 0; j < num_joints; j++)
        {
            if (j == i - 1 || j == i || j == i + 1) continue;
            if (intersect(i, j, joint_config))
            {
                return false;
            }
        }
    }

    if (get_ee_y_at(joint_config) < 0) return false;

    return true;
}

bool Arm::is_currently_valid()
{
    return is_valid(current_angles);
}

line_segment Arm::arm_segment(int seg, pose position)
{
    line_segment s;
    s.x1 = get_joint_x_at(seg, position);
    s.y1 = get_joint_y_at(seg, position);

    s.x2 = get_joint_x_at(seg+1, position);
    s.y2 = get_joint_y_at(seg+1, position);
    return s;
}

bool Arm::intersect(int seg1, int seg2, pose position)
{
    return intersect(arm_segment(seg1, position),
                     arm_segment(seg2, position));
}

bool Arm::collision(obstacle o, pose position)
{
    std::vector<line_segment> segs = o.get_segments();
    for (std::vector<line_segment>::iterator s = segs.begin();
         s != segs.end(); s++)
    {
        for (int j = 0; j < num_joints; j++)
        {
            if (intersect(*s, arm_segment(j, position)))
            {
                return true;
            }
        }
    }
    return false;
}

// http://geomalgorithms.com/a05-_intersect-1.html
bool Arm::intersect(line_segment seg1, line_segment seg2)
{
    float u_x = (seg1.x2 - seg1.x1);

    float u_y = (seg1.y2 - seg1.y1);

    float v_x = (seg2.x2 - seg2.x1);

    float v_y = (seg2.y2 - seg2.y1);

    float w_x = (seg1.x1 - seg2.x1);

    float w_y = (seg1.y1 - seg2.y1);

    float D = u_x*v_y - u_y*v_x;

    if (fabs(D) < 0.0000001) return false;

    float s = (v_x*w_y - v_y*w_x)/D;
    float t = (u_x*w_y - u_y*w_x)/D;

    if (s < 0 || s > 1) return false;
    if (t < 0 || t > 1) return false;

    return true;
}

bool Arm::apply(action a)
{
    for (change_vector::iterator it = a.changes.begin();
         it != a.changes.end(); it++)
    {
        current_angles.at(it->first) = (current_angles.at(it->first)
                                        + it->second);
    }
    return is_currently_valid();
}

bool Arm::apply(plan p)
{
    for (plan::iterator i = p.begin(); i != p.end(); i++)
    {
        apply(*i);
    }
    return is_currently_valid();
}

pose Arm::apply_at(action a, pose start)
{
    pose end = start;
    for (change_vector::iterator it = a.changes.begin();
         it != a.changes.end(); it++)
    {
        end.at(it->first) = (start.at(it->first)
                             + it->second);
    }
    return end;
}

// Pseudoinverse method
// http://en.wikipedia.org/wiki/Inverse_kinematics#The_Jacobian_inverse_technique
action Arm::solve_ik(float x, float y, pose position)
{
    action a;
    float scaling_factor = 0.1f;

    for (int i = 0; i < num_joints; i++)
    {
        a.changes[i] = 0;
    }

    Eigen::MatrixXf fk_jacobian(2, num_joints);
    pose cur_joints = position;
    Eigen::VectorXf joint_change;
    int i = 0;
    float fx, fy;

    while (true)
    {
        i++;
        fx = get_ee_x_at(cur_joints);
        fy = get_ee_y_at(cur_joints);
        if (sqrt(pow(x - fx, 2) + pow(y - fy, 2)) < 0.01 || i > 100)
        {
            break;
        }

        for (int i = 0; i < num_joints; i++)
        {
            float delta = cur_joints.at(i)*pow(10, -4);
            if ( delta < pow(10, -6)) delta = pow(10, -6);
            pose posd = cur_joints;
            posd.at(i) = cur_joints.at(i) + delta;
            fk_jacobian(0, i) = (get_ee_x_at(posd) - fx) / delta;
            fk_jacobian(1, i) = (get_ee_y_at(posd) - fy) / delta;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(fk_jacobian,
                                              (Eigen::ComputeFullU |
                                               Eigen::ComputeFullV));

        Eigen::MatrixXf sigma_pseudoinv =
            Eigen::MatrixXf::Zero(2, num_joints);
        sigma_pseudoinv.diagonal() = svd.singularValues();
        if (sigma_pseudoinv(0, 0) < 0.000001 ||
            sigma_pseudoinv(1, 1) < 0.000001) break;
        sigma_pseudoinv(0, 0) = 1.f/sigma_pseudoinv(0, 0);
        sigma_pseudoinv(1, 1) = 1.f/sigma_pseudoinv(1, 1);
        sigma_pseudoinv.transposeInPlace();

        Eigen::MatrixXf jacobian_plus =
            svd.matrixV()*sigma_pseudoinv*(svd.matrixU().transpose());

        Eigen::Vector2f dp;
        dp << (x - fx), (y - fy);
        joint_change = jacobian_plus*dp;

        for (int i = 0; i < num_joints; i++)
        {
            cur_joints.at(i) += scaling_factor*joint_change(i);
            a.changes[i] += scaling_factor*joint_change(i);
        }
    }

    if (sqrt(pow(x - fx, 2) + pow(y - fy, 2)) < 0.01) return a;
    std::cout << "Failed IK" << std::endl;
    return action(0, 0);
}

std::vector<action> Arm::get_big_primitives()
{
    return big_primitives;
}

std::vector<action> Arm::get_small_primitives()
{
    return small_primitives;
}

void Arm::set_primitive_change(float c)
{
    big_primitives.clear();
    small_primitives.clear();
    for (int i = 0; i < num_joints; i++)
    {
        big_primitives.push_back(action(i, c));
        big_primitives.push_back(action(i, -c));
        small_primitives.push_back(action(i, c/2.f));
        small_primitives.push_back(action(i, -c/2.f));
    }
}
