#include "ProbCogArm.h"
#include <iostream>

int probcog_arm::num_joints = 5;
float probcog_arm::base_height = 0.075f;
float probcog_arm::hand_length = 0.111f;
float probcog_arm::hand_width = 0.05;
float probcog_arm::hand_height = 0.04;
std::vector<joint> probcog_arm::configuration = std::vector<joint>();
std::vector<action> probcog_arm::big_prims = std::vector<action>();
std::vector<action> probcog_arm::small_prims = std::vector<action>();

// This is a hard-coded mess but it describes the big probcog arm
void probcog_arm::INIT()
{
    joint shoulder_yaw;
    shoulder_yaw.around = Z_AXIS;
    shoulder_yaw.min = -180.f*DEG_TO_RAD;
    shoulder_yaw.max = 180.f*DEG_TO_RAD;
    shoulder_yaw.length = 0.051;
    shoulder_yaw.width = 0.065;
    shoulder_yaw.default_speed = 0.2;
    shoulder_yaw.default_torque = 0.8;
    configuration.push_back(shoulder_yaw);

    joint shoulder_pitch;
    shoulder_pitch.around = Y_AXIS;
    shoulder_pitch.min = -110.f*DEG_TO_RAD;
    shoulder_pitch.max = 110.f*DEG_TO_RAD;
    shoulder_pitch.length = 0.225;
    shoulder_pitch.width = 0.065;
    shoulder_pitch.default_speed = 0.075;
    shoulder_pitch.default_torque = 0.9;
    configuration.push_back(shoulder_pitch);

    joint elbow;
    elbow.around = Y_AXIS;
    elbow.min = -140.f*DEG_TO_RAD;
    elbow.max = 140.f*DEG_TO_RAD;
    elbow.length = 0.2;
    elbow.width = 0.065;
    elbow.default_speed = 0.10;
    elbow.default_torque = 0.85;
    configuration.push_back(elbow);

    joint wrist_pitch;
    wrist_pitch.around = Y_AXIS;
    wrist_pitch.min = -125.f*DEG_TO_RAD;
    wrist_pitch.max = 125.f*DEG_TO_RAD;
    wrist_pitch.length = 0.08;
    wrist_pitch.width = 0.04;
    wrist_pitch.default_speed = 0.125;
    wrist_pitch.default_torque = 0.7;
    configuration.push_back(wrist_pitch);

    joint wrist_yaw;
    wrist_yaw.around = Z_AXIS;
    wrist_yaw.min = -150.f*DEG_TO_RAD;
    wrist_yaw.max = 150.f*DEG_TO_RAD;
    wrist_yaw.length = 0.0;
    wrist_yaw.width = 0.03;
    wrist_yaw.default_speed = 0.25;
    wrist_yaw.default_torque = 0.6;
    configuration.push_back(wrist_yaw);

    // Discounting hand joint for now, treating as stick on end

    set_primitive_change(10.f);
}

Eigen::Matrix4f probcog_arm::joint_transform(int joint_number,
                                             pose p)
{
    Eigen::Matrix4f xform = translation_matrix(0, 0, base_height);
    for (int i = 0; i < joint_number; i++)
    {
        xform *= rotation_matrix(p.at(i),
                                 configuration.at(i).around);
        xform *= translation_matrix(0, 0,
                                    configuration.at(i).length);
    }
    return xform;
}
point_3d probcog_arm::joint_xyz(int joint_number, pose p)
{
    Eigen::Matrix4f xform = joint_transform(joint_number, p);
    point_3d xyz;
    xyz.push_back(xform(0,3));
    xyz.push_back(xform(1,3));
    xyz.push_back(xform(2,3));
    return xyz;
}

point_3d probcog_arm::ee_xyz(pose p)
{
    Eigen::Matrix4f xform = (joint_transform(num_joints-1, p)*
                             rotation_matrix(p.at(num_joints-1),
                                             get_joint_axis(num_joints-1))*
                             translation_matrix(0, 0, hand_length));

    point_3d xyz;
    xyz.push_back(xform(0,3));
    xyz.push_back(xform(1,3));
    xyz.push_back(xform(2,3));
    return xyz;
}

orientation probcog_arm::ee_rpy(pose p)
{
    Eigen::Matrix4f xform = (joint_transform(num_joints-1, p)*
                             translation_matrix(0, 0, hand_length));

    orientation rpy;
    float r = atan2(xform(2,1), xform(2,2));
    float pi = atan2(-xform(2,0), sqrt(xform(0,0)*xform(0,0) +
                                       xform(1,0)*xform(1,0)));
    float y = atan2(xform(1,0), xform(0,0));
    rpy.push_back(r);
    rpy.push_back(pi);
    rpy.push_back(y);
    return rpy;
}

float probcog_arm::ee_pitch(pose p)
{
    point_3d ee = ee_xyz(p);
    Eigen::Matrix4f xform = joint_transform(num_joints-1, p);
    float z_diff = ee.at(2)-xform(2,3);

    float pitch = asin(z_diff/hand_length);
    return pitch;
}

float probcog_arm::ee_dist_to(pose from, point_3d to)
{
    point_3d ee = ee_xyz(from);
    float xdiff = ee.at(0) - to.at(0);
    float ydiff = ee.at(1) - to.at(1);
    float zdiff = ee.at(2) - to.at(2);
    return sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zdiff, 2));
}

action probcog_arm::solve_ik(pose from, point_3d to)
{
    action a(num_joints, 0.f);

    Eigen::MatrixXf fk_jacobian(3, num_joints);
    pose cur_joints = from;
    Eigen::VectorXf joint_change;
    int i = 0;
    point_3d fxyz;

    while (true)
    {
        i++;
        fxyz = ee_xyz(cur_joints);
        if (sqrt(pow(to.at(0) - fxyz.at(0), 2) +
                 pow(to.at(1) - fxyz.at(1), 2) +
                 pow(to.at(2) - fxyz.at(2), 2)) < 0.001 || i > 100)
        {
            break;
        }

        for (int k = 0; k < num_joints; k++)
        {
            float delta = cur_joints.at(k)*pow(10, -2);
            if ( delta < pow(10, -4)) delta = pow(10, -4);
            pose posd = cur_joints;
            posd.at(k) = cur_joints.at(k) + delta;
            point_3d xyz_d = ee_xyz(posd);
            fk_jacobian(0, k) = (xyz_d.at(0) - fxyz.at(0)) / delta;
            fk_jacobian(1, k) = (xyz_d.at(1) - fxyz.at(1)) / delta;
            fk_jacobian(2, k) = (xyz_d.at(2) - fxyz.at(2)) / delta;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(fk_jacobian,
                                              (Eigen::ComputeFullU |
                                               Eigen::ComputeFullV));

        Eigen::MatrixXf sigma_pseudoinv =
            Eigen::MatrixXf::Zero(3, num_joints);
        sigma_pseudoinv.diagonal() = svd.singularValues();

        if (sigma_pseudoinv(0, 0) < 0.000001 ||
            sigma_pseudoinv(1, 1) < 0.000001 ||
            sigma_pseudoinv(2, 2) < 0.000001) break;

        sigma_pseudoinv(0, 0) = 1.f/sigma_pseudoinv(0, 0);
        sigma_pseudoinv(1, 1) = 1.f/sigma_pseudoinv(1, 1);
        sigma_pseudoinv(2, 2) = 1.f/sigma_pseudoinv(2, 2);
        sigma_pseudoinv.transposeInPlace();

        Eigen::MatrixXf jacobian_plus =
            svd.matrixV()*sigma_pseudoinv*(svd.matrixU().transpose());

        Eigen::Vector3f dp;
        dp << (to.at(0) - fxyz.at(0)), (to.at(1) - fxyz.at(1)),
            (to.at(2) - fxyz.at(2));
        joint_change = jacobian_plus*dp;

        for (int k = 0; k < num_joints; k++)
        {
            cur_joints.at(k) += joint_change(k);
            a.at(k) += joint_change(k);
        }
    }

    if (sqrt(pow(to.at(0) - fxyz.at(0), 2) +
             pow(to.at(1) - fxyz.at(1), 2) +
             pow(to.at(2) - fxyz.at(2), 2)) < 0.01) return a;
    //std::cout << "Failed IK" << std::endl;
    return action(num_joints, 0);
}

action probcog_arm::solve_gripper(pose from, float to_pitch)
{
    action a(num_joints, 0.f);

    Eigen::MatrixXf fk_jacobian(1, 3);
    pose cur_joints = from;
    Eigen::VectorXf joint_change;
    int i = 0;
    float fpitch;

    while (true)
    {
        i++;
        fpitch = ee_pitch(cur_joints);
        if (fabs(to_pitch-fpitch) < 0.01)
        {
            return a;
        }
        if (i > 100) break;

        for (int k = 1; k < 4; k++)
        {
            float delta = cur_joints.at(k)*pow(10, -2);
            if ( delta < pow(10, -4)) delta = pow(10, -4);
            pose posd = cur_joints;
            posd.at(k) = cur_joints.at(k) + delta;
            float pitch_d = ee_pitch(posd);
            fk_jacobian(0, k-1) = (pitch_d - fpitch) / delta;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(fk_jacobian,
                                              (Eigen::ComputeFullU |
                                               Eigen::ComputeFullV));

        Eigen::MatrixXf sigma_pseudoinv =
            Eigen::MatrixXf::Zero(1, 3);
        sigma_pseudoinv.diagonal() = svd.singularValues();

        if (sigma_pseudoinv(0, 0) < 0.000001) break;

        sigma_pseudoinv(0, 0) = 1.f/sigma_pseudoinv(0, 0);
        sigma_pseudoinv.transposeInPlace();

        Eigen::MatrixXf jacobian_plus =
            svd.matrixV()*sigma_pseudoinv*(svd.matrixU().transpose());

        Eigen::Matrix<float, 1, 1> dp;
        dp << (to_pitch - fpitch);
        joint_change = jacobian_plus*dp;

        for (int k = 0; k < 3; k++)
        {
            cur_joints.at(k) += joint_change(k);
            a.at(k) += joint_change(k);
        }
    }
    //std::cout << "Failed Gripper IK" << std::endl;
    return action(num_joints, 0);
}

pose probcog_arm::apply(pose from, action act)
{
    pose end = from;
    for (int i = 0; i < num_joints; i++)
    {
        end.at(i) += act.at(i);
    }
    return end;
}

pose probcog_arm::apply(pose from, std::vector<action> plan)
{
    pose end = from;
    for (std::vector<action>::iterator i = plan.begin();
         i != plan.end(); i++)
    {
        end = apply(end, *i);
    }
    return end;
}

void probcog_arm::set_primitive_change(float big_deg)
{
    big_prims.clear();
    small_prims.clear();

    // Skip the last joint, which is wrist yaw
    // This will be fixed during search from now on
    for (int i = 0; i < num_joints-1; i++)
    {
        // change here if desired
        float big = big_deg*M_PI/180.f;
        float small = big/2.f;

        if (i < 3)
        {
        action bp = action(num_joints, 0);
        bp.at(i) = big;
        big_prims.push_back(bp);

        action bn = action(num_joints, 0);
        bn.at(i) = -big;
        big_prims.push_back(bn);
        }

        action sp = action(num_joints, 0);
        sp.at(i) = small;
        small_prims.push_back(sp);

        action sn = action(num_joints, 0);
        sn.at(i) = -small;
        small_prims.push_back(sn);
    }
}

bool probcog_arm::is_valid(pose p)
{
    // Joint range check
    for (int i = 0; i < num_joints; i++)
    {
        if (p.at(i) < configuration.at(i).min ||
            p.at(i) > configuration.at(i).max) return false;
    }
    return true;
}

Eigen::Matrix4f probcog_arm::rotation_matrix(float angle,
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

Eigen::Matrix4f probcog_arm::translation_matrix(float x,
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
