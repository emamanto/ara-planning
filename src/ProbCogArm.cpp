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
    wrist_yaw.length = 0.08;
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
