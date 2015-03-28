#include "ProbCogArm.h"

int probcog_arm::num_joints = 5;
float probcog_arm::base_height = 0.075f;
std::vector<joint> probcog_arm::configuration = std::vector<joint>();
std::vector<action> probcog_arm::big_prims = std::vector<action>();
std::vector<action> probcog_arm::small_prims = std::vector<action>();

// This is a hard-coded mess but it describes the big probcog arm
void probcog_arm::INIT()
{
    joint shoulder_yaw;
    shoulder_yaw.around = Z_AXIS;
    shoulder_yaw.min = -180.f;
    shoulder_yaw.max = 180.f;
    shoulder_yaw.length = 0.051;
    shoulder_yaw.width = 0.065;
    shoulder_yaw.default_speed = 0.2;
    shoulder_yaw.default_torque = 0.8;
    configuration.push_back(shoulder_yaw);

    joint shoulder_pitch;
    shoulder_pitch.around = Y_AXIS;
    shoulder_pitch.min = -110.f;
    shoulder_pitch.max = 110.f;
    shoulder_pitch.length = 0.225;
    shoulder_pitch.width = 0.065;
    shoulder_pitch.default_speed = 0.075;
    shoulder_pitch.default_torque = 0.9;
    configuration.push_back(shoulder_pitch);

    joint elbow;
    elbow.around = Y_AXIS;
    elbow.min = -140.f;
    elbow.max = 140.f;
    elbow.length = 0.2;
    elbow.width = 0.065;
    elbow.default_speed = 0.10;
    elbow.default_torque = 0.85;
    configuration.push_back(elbow);

    joint wrist_pitch;
    wrist_pitch.around = Y_AXIS;
    wrist_pitch.min = -125.f;
    wrist_pitch.max = 125.f;
    wrist_pitch.length = 0.08;
    wrist_pitch.width = 0.04;
    wrist_pitch.default_speed = 0.125;
    wrist_pitch.default_torque = 0.7;
    configuration.push_back(wrist_pitch);

    joint wrist_yaw;
    wrist_yaw.around = Y_AXIS;
    wrist_yaw.min = -150.f;
    wrist_yaw.max = 150.f;
    // ?? 0.111
    wrist_yaw.length = 0.0;
    wrist_yaw.width = 0.03;
    wrist_yaw.default_speed = 0.25;
    wrist_yaw.default_torque = 0.6;
    configuration.push_back(wrist_yaw);

    for (int i = 0; i < num_joints; i++)
    {
        // change here if desired
        float big = 10;
        float small = 5;

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

point_3d probcog_arm::joint_xyz(int joint_number, pose p)
{
    Eigen::Matrix4f xform = translation_matrix(0, 0, base_height);
    for (int i = 0; i < joint_number; i++)
    {
        xform *= rotation_matrix(p.at(i),
                                 configuration.at(i).around);
        xform *= translation_matrix(0, 0,
                                    configuration.at(i).length);
    }
    point_3d xyz;
    xyz.push_back(xform(0,3));
    xyz.push_back(xform(1,3));
    xyz.push_back(xform(2,3));
    return xyz;
}

point_3d probcog_arm::ee_xyz(pose p)
{
    return joint_xyz(num_joints-1, p);
}

float probcog_arm::ee_dist_to(pose from, point_3d to)
{
    point_3d ee = ee_xyz(from);
    float xdiff = ee.at(0) - to.at(0);
    float ydiff = ee.at(1) - to.at(1);
    float zdiff = ee.at(2) - to.at(2);
    return sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zdiff, 2));
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

bool probcog_arm::is_valid(pose p)
{
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
