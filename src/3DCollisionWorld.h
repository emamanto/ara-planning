#pragma once

#include <vector>
#include <string>

#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "arm_collision_boxes_t.hpp"

#include "ProbCogArm.h"

bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata);

struct collision_data
{
    collision_data()
    {
        done = false;
    }

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    bool done;
};

struct object_data
{
    int id;
    std::string type;
    std::string color;
};

typedef std::pair<object_data, object_data> collision_pair;

class collision_world
{
public:
    static void add_object(std::vector<float> dim,
                           std::vector<float> xyzrpy,
                           object_data obj_info);
    static void add_object(double dim[],
                           double xyzrpy[],
                           object_data obj_info);

    static void clear();

    static bool collision(pose arm_position, bool details = false);
    static int num_collisions() {return colliding.size();}
    static collision_pair& get_collision_pair(int i)
    {return colliding.at(i);}

    static void set_held_object(double dims[]);
    static void clear_held_object();

    static arm_collision_boxes_t arm_boxes(pose arm_position);

private:
    collision_world() {};
    collision_world(collision_world const&) {};
    collision_world& operator=(collision_world const&) {};

    static fcl::BroadPhaseCollisionManager* world_objects_m;
    static fcl::BroadPhaseCollisionManager* arm_objects_m;
    static fcl::BroadPhaseCollisionManager* hand_objects_m;
    static fcl::BroadPhaseCollisionManager* base_objects_m;
    static bool has_held_object;
    static std::vector<float> held_object_dims;
    static std::vector<object_data> world_objects_info;
    static std::vector<collision_pair> colliding;
};
