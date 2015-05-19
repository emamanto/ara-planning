#include "Shortcut.h"

bool subdivision_collision_check(pose p, action a, int depth)
{
    // Cutoff?
    if (depth > 20) return false;
    Arm* arm = Arm::the_instance();
    action half;
    for (int i = 0; i < arm->get_num_joints(); i++)
    {
        half.changes[i] = (a.changes[i]/2);
    }
    pose midpt = arm->apply_at(half, p);
    if (arm_state(midpt).valid())
    {
        bool before = subdivision_collision_check(p, half, depth+1);
        bool after = subdivision_collision_check(midpt, half, depth+1);
        return (before || after);
    }
    else return true;
}

plan shortcut(plan original, pose start)
{
    if (original.size() == 1) return original;

    Arm* arm = Arm::the_instance();
    action sc;
    for (plan::iterator i = original.begin();
         i != original.end(); i++)
    {
        for (int j = 0; j < arm->get_num_joints(); j++)
        {
            sc.changes[j] += i->changes[j];
        }
    }

    if (subdivision_collision_check(start, sc, 0))
    {
        // second to end
        plan p2;
        for(int i = 1; i < original.size(); i++)
        {
            p2.push_back(original.at(i));
        }
        pose new_start = arm->apply_at(original.at(0), start);
        plan p2_sc = shortcut(p2 , new_start);

        plan combined;
        combined.push_back(original.at(0));
        for(int i = 0; i < p2_sc.size(); i++)
        {
            combined.push_back(p2_sc.at(i));
        }
        return combined;
    }
    else
    {
        // shortcut
        plan shorter;
        shorter.push_back(sc);
        return shorter;
    }
}
