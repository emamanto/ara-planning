#include "FetchArm.h"
#include <iostream>

int fetch_arm::num_joints = 7;
point_3d fetch_arm::base_offset = point_3d();
float fetch_arm::hand_length = 0.1f;
float fetch_arm::hand_width = 0.11f;
float fetch_arm::hand_height = 0.02;
float fetch_arm::finger_width = 0.02;

std::vector<joint> fetch_arm::configuration = std::vector<joint>();
std::vector<action> fetch_arm::big_prims = std::vector<action>();
std::vector<action> fetch_arm::small_prims = std::vector<action>();

action subtract(pose end, pose begin)
{
    pose result;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        result.push_back(end.at(i) - begin.at(i));
    }
    return result;
}

point_3d translation_from_xform(Eigen::Matrix4f xform)
{
    point_3d t;
    t.push_back(xform(0, 3));
    t.push_back(xform(1, 3));
    t.push_back(xform(2, 3));
    return t;
}

// XZY ORDER--THIS IS WEIRD DON'T FORGET
// Ripped from threejs
point_3d eulers_from_xform(Eigen::Matrix4f xform)
{
    float m12 = xform(0, 1);
    if (m12 < -1) m12 = -1;
    if (m12 > 1) m12 = 1;
    float z = asin(-m12);

    float x, y;
    if (fabs(m12) < 0.99999)
    {
        x = atan2(xform(2, 1), xform(1, 1));
        y = atan2(xform(0, 2), xform(0, 0));
    }
    else
    {
        x = atan2(-xform(1, 2), xform(2, 2));
        y = 0;
    }

    orientation r;
    r.push_back(x);
    r.push_back(y);
    r.push_back(z);
    return r;
}

float mod_pi(float angle)
{
    while (angle > M_PI)
    {
        angle -= M_PI;
    }
    while (angle < -M_PI)
    {
        angle += M_PI;
    }
    return angle;
}

void fetch_arm::INIT()
{
    base_offset.push_back(-0.5);
    base_offset.push_back(0);
    base_offset.push_back(0.5);

    joint shoulder_pan;
    shoulder_pan.type = REVOLUTE;
    shoulder_pan.around = Z_AXIS;
    shoulder_pan.min = -92.f*DEG_TO_RAD;
    shoulder_pan.max = 92.f*DEG_TO_RAD;
    shoulder_pan.length = 0.117;
    shoulder_pan.width = 0.04;
    shoulder_pan.default_speed = 1.0;
    shoulder_pan.default_torque = 33.0;
    configuration.push_back(shoulder_pan);

    joint shoulder_lift;
    shoulder_lift.type = REVOLUTE;
    shoulder_lift.around = Y_AXIS;
    shoulder_lift.min = -70.f*DEG_TO_RAD;
    shoulder_lift.max = 87.f*DEG_TO_RAD;
    shoulder_lift.length = 0.219;
    shoulder_lift.width = 0.04;
    shoulder_lift.default_speed = 1.2;
    shoulder_lift.default_torque = 120.0;
    configuration.push_back(shoulder_lift);

    joint upperarm_roll;
    upperarm_roll.type = CONTINUOUS;
    upperarm_roll.around = X_AXIS;
    upperarm_roll.min = -M_PI;
    upperarm_roll.max = M_PI;
    upperarm_roll.length = 0.133;
    upperarm_roll.width = 0.04;
    upperarm_roll.default_speed = 1.5;
    upperarm_roll.default_torque = 70.0;
    configuration.push_back(upperarm_roll);

    joint elbow_flex;
    elbow_flex.type = REVOLUTE;
    elbow_flex.around = Y_AXIS;
    elbow_flex.min = -129.f*DEG_TO_RAD;
    elbow_flex.max = 129.f*DEG_TO_RAD;
    elbow_flex.length = 0.197;
    elbow_flex.width = 0.04;
    elbow_flex.default_speed = 1.4;
    elbow_flex.default_torque = 65.0;
    configuration.push_back(elbow_flex);

    joint forearm_roll;
    forearm_roll.type = CONTINUOUS;
    forearm_roll.around = X_AXIS;
    forearm_roll.min = -M_PI;
    forearm_roll.max = M_PI;
    forearm_roll.length = 0.1245;
    forearm_roll.width = 0.04;
    forearm_roll.default_speed = 1.5;
    forearm_roll.default_torque = 28.0;
    configuration.push_back(forearm_roll);

    joint wrist_flex;
    wrist_flex.type = REVOLUTE;
    wrist_flex.around = Y_AXIS;
    wrist_flex.min = -125.f*DEG_TO_RAD;
    wrist_flex.max = 125.f*DEG_TO_RAD;
    wrist_flex.length = 0.1245;
    wrist_flex.width = 0.04;
    wrist_flex.default_speed = 2.2;
    wrist_flex.default_torque = 25.0;
    configuration.push_back(wrist_flex);

    joint wrist_roll;
    wrist_roll.type = CONTINUOUS;
    wrist_roll.around = X_AXIS;
    wrist_roll.min = -M_PI;
    wrist_roll.max = M_PI;
    // Ignoring fixed gripper joint, adding here
    wrist_roll.length = 0.1485;
    wrist_roll.width = 0.04;
    wrist_roll.default_speed = 2.2;
    wrist_roll.default_torque = 7.0;
    configuration.push_back(wrist_roll);

    // Discounting hand joint for now, treating as stick on end

    set_primitive_change(M_PI/12);
}

Eigen::Matrix4f fetch_arm::joint_transform(int joint_number,
                                           pose p)
{
    Eigen::Matrix4f xform = translation_matrix(base_offset[0],
                                               base_offset[1],
                                               base_offset[2]);

    for (int i = 0; i <= joint_number; i++)
    {
        xform *= rotation_matrix(p.at(i),
                                 configuration.at(i).around);
        xform *= translation_matrix(configuration.at(i).length,
                                    0, 0);
    }
    return xform;
}
point_3d fetch_arm::joint_xyz(int joint_number, pose p)
{
    Eigen::Matrix4f xform = joint_transform(joint_number, p);
    point_3d xyz;
    xyz.push_back(xform(0,3));
    xyz.push_back(xform(1,3));
    xyz.push_back(xform(2,3));
    return xyz;
}

point_3d fetch_arm::ee_xyz(pose p)
{
    Eigen::Matrix4f xform = ee_xform(p);
    point_3d xyz;
    xyz.push_back(xform(0,3));
    xyz.push_back(xform(1,3));
    xyz.push_back(xform(2,3));
    return xyz;
}

Eigen::Matrix4f fetch_arm::ee_xform(pose p)
{
    Eigen::Matrix4f xform = (joint_transform(num_joints-1, p));
    xform *= translation_matrix(hand_length, 0, 0);

    return xform;
}

orientation fetch_arm::ee_rpy(pose p)
{
    Eigen::Matrix4f xform = (joint_transform(num_joints-1, p)*
                             translation_matrix(0, 0, hand_length));
    return eulers_from_xform(xform);
}

float fetch_arm::ee_dist_to(pose from, point_3d to)
{
    point_3d ee = ee_xyz(from);
    float xdiff = ee.at(0) - to.at(0);
    float ydiff = ee.at(1) - to.at(1);
    float zdiff = ee.at(2) - to.at(2);
    return sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zdiff, 2));
}

action fetch_arm::solve_ik(pose from, Eigen::Matrix4f xform)
{
    action a(num_joints, 0.f);

    point_3d target_pos = translation_from_xform(xform);
    orientation target_or = eulers_from_xform(xform);

    Eigen::MatrixXf jacobian(6, num_joints);
    pose cur_joints = from;

    point_3d cur_xyz;
    orientation cur_rpy;
    Eigen::VectorXf joint_change;
    int i = 0;

    while (true)
    {
        i++;
        cur_xyz = ee_xyz(cur_joints);
        cur_rpy = ee_rpy(cur_joints);

        if ((sqrt(pow(target_pos.at(0) - cur_xyz.at(0), 2) +
                  pow(target_pos.at(1) - cur_xyz.at(1), 2) +
                  pow(target_pos.at(2) - cur_xyz.at(2), 2)) < 0.001 &&
             fabs(cur_rpy.at(1) - target_or.at(1)) < 0.01))
        {
            for (int l = 0; l < a.size(); l++)
            {
                a[l] = mod_pi(a[l]);
            }
            return a;
        }
        if (i > 500) break;

        // Building the Jacobian
        for (int k = 0; k < num_joints; k++)
        {
            Eigen::Matrix4f joint_xform = joint_transform(k, cur_joints);

            Eigen::Vector4f axis_vec;
            axis a = fetch_arm::get_joint_axis(k);
            if (a == X_AXIS)
            {
                axis_vec << 1, 0, 0, 1;
            }
            else if (a == Y_AXIS)
            {
                axis_vec << 0, 1, 0, 1;
            }
            else // Z_AXIS
            {
                axis_vec << 0, 0, 1, 1;
            }

            Eigen::Vector4f axis_in_world_4 = joint_xform*axis_vec;
            Eigen::Vector3f axis_in_world;
            axis_in_world << axis_in_world_4(0),
                axis_in_world_4(1),
                axis_in_world_4(2);

            Eigen::Vector3f joint_origin;
            joint_origin << joint_xform(0, 3),
                joint_xform(1, 3),
                joint_xform(2, 3);

            Eigen::Vector3f o_diff;
            o_diff << cur_xyz.at(0) - joint_origin(0),
                cur_xyz.at(1) - joint_origin(1),
                cur_xyz.at(2) - joint_origin(2);

            Eigen::Vector3f linear = axis_in_world.cross(o_diff);
            for (int m = 0; m < 3; m++)
            {
                jacobian(m, k) = linear(m);
            }
            for (int n = 3; n < 6; n++)
            {
                jacobian(n, k) = axis_in_world(n-3);
            }
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jacobian,
                                              (Eigen::ComputeFullU |
                                               Eigen::ComputeFullV));

        Eigen::MatrixXf sigma_pseudoinv =
            Eigen::MatrixXf::Zero(6, num_joints);
        sigma_pseudoinv.diagonal() = svd.singularValues();

        bool stop = false;
        for (int k = 0; k < 6; k++)
        {
            if (sigma_pseudoinv(k, k) < 0.000001)
            {
                stop = true;
                break;
            }
            sigma_pseudoinv(k, k) = 1.f/sigma_pseudoinv(k, k);
        }
        if (stop)
        {
            break;
        }

        sigma_pseudoinv.transposeInPlace();

        Eigen::MatrixXf jacobian_plus =
                                    svd.matrixV()*sigma_pseudoinv*(svd.matrixU().transpose());

        Eigen::Matrix<float, 6, 1> dp;
        dp << (target_pos.at(0) - cur_xyz.at(0)),
            (target_pos.at(1) - cur_xyz.at(1)),
            (target_pos.at(2) - cur_xyz.at(2)),
            (target_or.at(0) - cur_rpy.at(0)),
            (target_or.at(1) - cur_rpy.at(1)),
            (target_or.at(2) - cur_rpy.at(2));

        joint_change = jacobian_plus*dp;
        joint_change = 0.1*joint_change;

        for (int k = 0; k < num_joints; k++)
        {
            cur_joints.at(k) += joint_change(k);
            a.at(k) += joint_change(k);
        }
    }

    //std::cout << "Failed IK" << std::endl;
    return action(num_joints, 0);
}

pose fetch_arm::apply(pose from, action act)
{
    pose end = from;
    for (int i = 0; i < num_joints; i++)
    {
        end.at(i) += act.at(i);
        // if (configuration.at(i).type == CONTINUOUS)
        //     end.at(i) = mod_pi(end.at(i));
    }
    return end;
}

pose fetch_arm::apply(pose from, std::vector<action> plan)
{
    pose end = from;
    for (std::vector<action>::iterator i = plan.begin();
         i != plan.end(); i++)
    {
        end = apply(end, *i);
    }
    return end;
}

void fetch_arm::set_primitive_change(float big_rad)
{
    big_prims.clear();
    small_prims.clear();

    // Ignore wrist roll
    for (int i = 0; i < num_joints-1; i++)
    {
        // change here if desired
        float big = big_rad;
        float small = big/2.f;

        action bp = action(num_joints, 0);
        bp.at(i) = big;
        big_prims.push_back(bp);

        action bn = action(num_joints, 0);
        bn.at(i) = -big;
        big_prims.push_back(bn);

        action sp = action(num_joints, 0);
        sp.at(i) = small;
        small_prims.push_back(sp);

        action sn = action(num_joints, 0);
        sn.at(i) = -small;
        small_prims.push_back(sn);
    }
}

bool fetch_arm::is_valid(pose p)
{
    // Joint range check
    for (int i = 0; i < num_joints; i++)
    {
        // if (configuration.at(i).type == CONTINUOUS) continue;
        if (p.at(i) < configuration.at(i).min ||
            p.at(i) > configuration.at(i).max) return false;
    }
    return true;
}

Eigen::Matrix4f fetch_arm::rotation_matrix(float angle,
                                           axis around)
{
    Eigen::Matrix4f rot;
    if ( around == X_AXIS )
    {
        rot << 1, 0, 0, 0,
            0, cos(angle), -sin(angle), 0,
            0, sin(angle), cos(angle), 0,
            0, 0, 0, 1;
    }
    else if ( around == Y_AXIS )
    {
        rot << cos(angle), 0, sin(angle), 0,
            0, 1, 0, 0,
            -sin(angle), 0, cos(angle), 0,
            0, 0, 0, 1;
    }
    else //Z
    {
        rot << cos(angle), -sin(angle), 0, 0,
            sin(angle), cos(angle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    }
    return rot;
}

Eigen::Matrix4f fetch_arm::translation_matrix(float x,
                                              float y,
                                              float z)
{
    Eigen::Matrix4f trans;
    trans << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return trans;
}
