#include "3DCollisionWorld.h"

std::vector<fcl::CollisionObject*> collision_world::objects =
    std::vector<fcl::CollisionObject*>();

void collision_world::add_object()
{
    fcl::Box* box = new fcl::Box(5, 10, 20);
    fcl::Transform3f tr = fcl::Transform3f();
    objects.push_back(
        new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box),
                                 tr));
}
