#include "3DCollisionWorld.h"

bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata)
{
    collision_data* cd = static_cast<collision_data*>(cdata);
    const fcl::CollisionRequest& request = cd->request;
    fcl::CollisionResult& result = cd->result;

    if(cd->done) return true;

    fcl::collide(o1, o2, request, result);

    if(!request.enable_cost &&
       (result.isCollision()) &&
       (result.numContacts() >= request.num_max_contacts))
        cd->done = true;

    return cd->done;
}

fcl::BroadPhaseCollisionManager* collision_world::world_objects_m = 0;
fcl::BroadPhaseCollisionManager* collision_world::arm_objects_m = 0;
fcl::BroadPhaseCollisionManager* collision_world::hand_objects_m = 0;
fcl::BroadPhaseCollisionManager* collision_world::base_objects_m = 0;

void collision_world::add_object(std::vector<float> dim,
                                 std::vector<float> xyzrpy)
{
    fcl::Box* box = new fcl::Box(dim[0], dim[1], dim[2]);

    // Translation vector
    fcl::Vec3f trans(xyzrpy[0], xyzrpy[1], xyzrpy[2]);

    // RPY matrix
    fcl::Matrix3f rmat(1,              0,               0,
                       0, cos(xyzrpy[3]), -sin(xyzrpy[3]),
                       0, sin(xyzrpy[3]), cos(xyzrpy[3]));
    fcl::Matrix3f pmat(cos(xyzrpy[4]),  0, sin(xyzrpy[4]),
                       0,               1,              0,
                       -sin(xyzrpy[4]), 0, cos(xyzrpy[4]));
    fcl::Matrix3f ymat(cos(xyzrpy[5]), -sin(xyzrpy[5]), 0,
                       sin(xyzrpy[5]),  cos(xyzrpy[5]), 0,
                                    0,               0, 1);
    fcl::Matrix3f rot = ymat * pmat * rmat;

    // Put translation and rotation together
    fcl::Transform3f p = fcl::Transform3f(rot, trans);

    if (!world_objects_m)
    {
        world_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    }

    world_objects_m->registerObject(
        new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box),
                                 p));
}

void collision_world::add_object(double dim[], double xyzrpy[])
{
    std::vector<float> dim_vec;
    for (int i = 0; i < 3; i++)
    {
        dim_vec.push_back(dim[i]);
    }
    std::vector<float> pos_vec;
    for (int i = 0; i < 6; i++)
    {
        pos_vec.push_back(xyzrpy[i]);
    }

    add_object(dim_vec, pos_vec);
}

void collision_world::clear()
{
    if (world_objects_m) world_objects_m->clear();
}

bool collision_world::collision(pose arm_position)
{
    if (!arm_objects_m)
    {
        arm_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    }
    if (!hand_objects_m)
    {
        hand_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    }
    if (!base_objects_m)
    {
        base_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    }

    arm_objects_m->clear();
    hand_objects_m->clear();
    base_objects_m->clear();

    for (int i = 0; i < probcog_arm::get_num_joints(); i++)
    {
        fcl::Box* box = new
            fcl::Box(probcog_arm::get_component_width(i),
                     probcog_arm::get_component_width(i),
                     probcog_arm::get_component_length(i));

        Eigen::Matrix4f trmat =
            probcog_arm::joint_transform(i+1, arm_position)*
            probcog_arm::translation_matrix(0,
                                            0,
                                            -probcog_arm::get_component_length(i)/2);

        fcl::Matrix3f rot(trmat(0, 0), trmat(0, 1), trmat(0, 2),
                          trmat(1, 0), trmat(1, 1), trmat(1, 2),
                          trmat(2, 0), trmat(2, 1), trmat(2, 2));
        fcl::Vec3f trans(trmat(0, 3), trmat(1, 3), trmat(2, 3));
        fcl::Transform3f p = fcl::Transform3f(rot, trans);

        fcl::CollisionObject* obj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box),
                                                             p);
        arm_objects_m->registerObject(obj);

        if (i < 2) base_objects_m->registerObject(obj);
        if (i > 2) hand_objects_m->registerObject(obj);
    }

    // HAND
    fcl::Box* box = new
        fcl::Box(probcog_arm::hand_width+0.04,
                 probcog_arm::hand_height*2+0.02,
                 probcog_arm::hand_length+0.02);

    int last_joint = probcog_arm::get_num_joints()-1;
    Eigen::Matrix4f trmat =
        probcog_arm::joint_transform(last_joint,
                                     arm_position)*
        probcog_arm::rotation_matrix(arm_position.at(last_joint),
                                     probcog_arm::get_joint_axis(last_joint))*
        probcog_arm::translation_matrix(0,
                                        0.02,
                                        probcog_arm::hand_length/2);

    fcl::Matrix3f rot(trmat(0, 0), trmat(0, 1), trmat(0, 2),
                      trmat(1, 0), trmat(1, 1), trmat(1, 2),
                      trmat(2, 0), trmat(2, 1), trmat(2, 2));
    fcl::Vec3f trans(trmat(0, 3), trmat(1, 3), trmat(2, 3));
    fcl::Transform3f p = fcl::Transform3f(rot, trans);

    fcl::CollisionObject* hobj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), p);

    arm_objects_m->registerObject(hobj);
    hand_objects_m->registerObject(hobj);
    // end HAND

    // BASE
    fcl::Box* base_box = new
        fcl::Box(0.065, 0.065, probcog_arm::base_height-0.02);

    fcl::Matrix3f brot(1, 0, 0,
                      0, 1, 0,
                      0, 0, 1);
    fcl::Vec3f btrans(0, 0, probcog_arm::base_height/2 + 0.011);
    fcl::Transform3f q = fcl::Transform3f(brot, btrans);

    fcl::CollisionObject* bobj = new
        fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(base_box), q);
    base_objects_m->registerObject(bobj);
    arm_objects_m->registerObject(bobj);

    // Self-collision check
    collision_data self_data;
    self_data.request = fcl::CollisionRequest();
    self_data.request.enable_contact = false;
    self_data.request.enable_cost = false;

    self_data.result = fcl::CollisionResult();

    hand_objects_m->collide(base_objects_m, &self_data,
                            collision_function);

    if (self_data.result.isCollision()) return true;

    // end BASE

    // TABLE
    std::vector<float> dims;
    dims.push_back(1);
    dims.push_back(1);
    dims.push_back(0.04);

    std::vector<float> pos;
    for (int i = 0; i < 6; i++) pos.push_back(0);

    add_object(dims, pos);
    // std::cout << "Checking for collision against "
    //           << world_objects_m->size() << " world objs"
    //           << std::endl;

    collision_data data;
    data.request = fcl::CollisionRequest();
    data.request.enable_contact = false;
    data.request.enable_cost = false;

    data.result = fcl::CollisionResult();

    arm_objects_m->collide(world_objects_m, &data,
                           collision_function);

    return data.result.isCollision();
}

arm_collision_boxes_t collision_world::arm_boxes(pose arm_position)
{
    collision(arm_position);
    std::vector<fcl::CollisionObject*> objs;
    arm_objects_m->getObjects(objs);

    arm_collision_boxes_t msg;
    msg.len = arm_objects_m->size();

    for (int i = 0; i <= probcog_arm::get_num_joints()+1; i++)
    {
        bbox_info_t cur;
        cur.joint = i;
        const fcl::Box* b = static_cast<const fcl::Box*>(objs[i]->getCollisionGeometry());
        cur.dim[0] = b->side[0];
        cur.dim[1] = b->side[1];
        cur.dim[2] = b->side[2];

        fcl::Quaternion3f q = objs[i]->getQuatRotation();
        cur.quat[0] = q.getW();
        cur.quat[1] = q.getX();
        cur.quat[2] = q.getY();
        cur.quat[3] = q.getZ();

        fcl::Vec3f v = objs[i]->getTranslation();
        cur.trans[0] = v[0];
        cur.trans[1] = v[1];
        cur.trans[2] = v[2];

        msg.boxes.push_back(cur);
    }

    return msg;
}
