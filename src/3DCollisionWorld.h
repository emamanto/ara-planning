#pragma once

#include <vector>

#include "fcl/config.h"
#include "fcl/broadphase/broadphase.h"

class collision_world
{
public:
    static void add_object();

private:
    collision_world() {};
    collision_world(collision_world const&) {};
    collision_world& operator=(collision_world const&) {};

    static std::vector<fcl::CollisionObject*> objects;
};
