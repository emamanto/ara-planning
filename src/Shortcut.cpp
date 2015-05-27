#include "Shortcut.h"
#include <iostream>

bool subdivision_collision_check(pose p, action a, int depth)
{
    // Cutoff?
    if (depth > 6) return false;
    Arm* arm = Arm::the_instance();
    action half(arm->get_num_joints(), 0);
    for (int i = 0; i < arm->get_num_joints(); i++)
    {
        half.at(i) = (a.at(i)/2);
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

plan shortcut_partial(plan original, pose start)
{
    // std::cout << "Shortcutting a plan of length " << original.size()
    //          << std::endl;
    if (original.size() == 1) return original;

    Arm* arm = Arm::the_instance();
    action sc(arm->get_num_joints(), 0);
    for (plan::iterator i = original.begin();
         i != original.end(); i++)
    {
        for (int j = 0; j < arm->get_num_joints(); j++)
        {
            sc.at(j) += i->at(j);
        }
    }

    if (subdivision_collision_check(start, sc, 0))
    {
        // std::cout << "Collision" << std::endl;
        //second to end
        plan p2;
        for(int i = 0; i < original.size()-1; i++)
        {
             p2.push_back(original.at(i));
        }
        plan p2_sc = shortcut_partial(p2, start);

        plan combined;
        for(int i = 0; i < p2_sc.size(); i++)
        {
            combined.push_back(p2_sc.at(i));
        }
        combined.push_back(original.at(original.size()-1));

        return combined;
    }
    else
    {
        //std::cout << "Found a shortcut!" << std::endl;
        // shortcut
        plan shorter;
        shorter.push_back(sc);
        return shorter;
    }
}

plan shortcut(plan original, pose start)
{
    Arm* arm = Arm::the_instance();
    plan shortened;
    for (plan::iterator i = original.begin();
         i != original.end(); i++)
    {
        shortened.push_back(*i);
    }

    for (int i = 0; i < shortened.size(); i++)
    {
        // std::cout << "Shortening between " << i
        //          << " and end" << std::endl;
        plan before;
        for (int j = 0; j < i; j++)
        {
            before.push_back(shortened.at(j));
        }
        plan after;
        for (int j = i; j < shortened.size(); j++)
        {
            after.push_back(shortened.at(j));
        }

        plan sc = shortcut_partial(after, arm->apply_at(before, start));

        shortened.clear();
        for (int j = 0; j < i; j++)
        {
            shortened.push_back(before.at(j));
        }
        for (int j = 0; j < sc.size(); j++)
        {
            shortened.push_back(sc.at(j));
        }
    }

    return shortened;
}
