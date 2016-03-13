#pragma once

#include <vector>
#include <string>

#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/broadphase/broadphase.h"

#include "FetchArm.h"

#define PUBLISH_COLLISION_MODEL

#ifdef PUBLISH_COLLISION_MODEL
#include <lcm/lcm-cpp.hpp>
#include "arm_collision_boxes_t.hpp"
#endif

bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata);
bool distance_function(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata, fcl::FCL_REAL& dist);

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

struct distance_data
{
    distance_data()
    {
        done = false;
    }

    fcl::DistanceRequest request;
    fcl::DistanceResult result;
    bool done;
};

struct object_data
{
    int id;
    std::string type;
    std::string color;
};

struct collision_info
{
    bool collision;
    bool computed_distance;
    double distance;
};

typedef std::pair<object_data, object_data> collision_pair;

class collision_world
{
public:
    collision_world();

    void add_object(std::vector<float> dim,
                    std::vector<float> xyzrpy,
                           object_data obj_info);
    void add_object(double dim[],
                    double xyzrpy[],
                    object_data obj_info);

    void clear();

    bool collision(pose arm_position,
                   float hand_position,
                   bool should_publish = false);

    collision_info collision(pose arm_position,
                             float hand_position,
                             bool compute_distance,
                             bool should_publish);

    int num_collisions() {return colliding.size();}
    collision_pair& get_collision_pair(int i)
    {return colliding.at(i);}

    static void set_held_object(std::vector<float> dims);
    static void clear_held_object();

private:
    collision_world(collision_world const&) {};
    collision_world& operator=(collision_world const&) {};

#ifdef PUBLISH_COLLISION_MODEL
    void publish_arm_boxes(pose arm_position);
#endif

    fcl::BroadPhaseCollisionManager* world_objects_m;
    fcl::BroadPhaseCollisionManager* arm_objects_m;
    fcl::BroadPhaseCollisionManager* hand_objects_m;
    fcl::BroadPhaseCollisionManager* base_objects_m;
    fcl::CollisionObject* base;
    fcl::CollisionObject* hand;
    std::vector<fcl::CollisionObject*> arm_parts;

    std::vector<object_data> world_objects_info;
    std::vector<collision_pair> colliding;

    static bool has_held_object;
    static std::vector<float> held_object_dims;
};
