#pragma once

#include <vector>

template<typename S, typename P>
bool subdivision_collision_check(S start, P action, int depth)
{
    // Cutoff?
    if (depth > 6) return false;
    P half(action.size(), 0);
    for (int i = 0; i < action.size(); i++)
    {
        half.at(i) = (action.at(i)/2);
    }
    S midpt = start.apply(half);
    if (midpt.valid())
    {
        bool before =
            subdivision_collision_check<S, P>(start, half, depth+1);
        bool after =
            subdivision_collision_check<S, P>(midpt, half, depth+1);
        return (before || after);
    }
    else return true;
}

template<typename S, typename P>
std::vector<P> shortcut_partial(std::vector<P> original, S start)
{
    // std::cout << "Shortcutting a plan of length " << original.size()
    //          << std::endl;
    if (original.size() == 1) return original;

    P sh_ct(original.at(0).size(), 0);
    for (typename std::vector<P>::iterator i = original.begin();
         i != original.end(); i++)
    {
        for (int j = 0; j < original.at(0).size(); j++)
        {
            sh_ct.at(j) += i->at(j);
        }
    }

    if (subdivision_collision_check<S, P>(start, sh_ct, 0))
    {
        // std::cout << "\tCollision" << std::endl;
        //second to end
        std::vector<P> p2;
        for(int i = 0; i < original.size()-1; i++)
        {
             p2.push_back(original.at(i));
        }
        std::vector<P> p2_sc = shortcut_partial<S, P>(p2, start);

        std::vector<P> combined;
        for(int i = 0; i < p2_sc.size(); i++)
        {
            combined.push_back(p2_sc.at(i));
        }
        combined.push_back(original.at(original.size()-1));

        return combined;
    }
    else
    {
        // std::cout << "\tFound a shortcut!" << std::endl;
        // shortcut
        std::vector<P> shorter;
        shorter.push_back(sh_ct);
        return shorter;
    }
}

template<typename S, typename P>
std::vector<P> shortcut(std::vector<P> original, S start)
{
    std::vector<P> shortened;
    for (typename std::vector<P>::iterator i = original.begin();
         i != original.end(); i++)
    {
        shortened.push_back(*i);
    }

    for (int i = 0; i < shortened.size(); i++)
    {
        // std::cout << "Shortening between " << i
        //          << " and end" << std::endl;
        std::vector<P> before;
        for (int j = 0; j < i; j++)
        {
            before.push_back(shortened.at(j));
        }
        std::vector<P> after;
        for (int j = i; j < shortened.size(); j++)
        {
            after.push_back(shortened.at(j));
        }

        S nstart = start;
        for (typename std::vector<P>::iterator it = before.begin();
             it != before.end(); it++)
        {
            nstart = nstart.apply(*it);
        }

        std::vector<P> sc = shortcut_partial<S, P>(after, nstart);

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
