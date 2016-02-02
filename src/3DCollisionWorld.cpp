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

bool collision_world::has_held_object = false;
std::vector<float> collision_world::held_object_dims = std::vector<float>(3, 0);

collision_world::collision_world()
{
    world_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    arm_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    hand_objects_m = new fcl::DynamicAABBTreeCollisionManager();
    base_objects_m = new fcl::DynamicAABBTreeCollisionManager();

    clear();
}

void collision_world::add_object(std::vector<float> dim,
                                 std::vector<float> xyzrpy,
                                 object_data obj_info)
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

    world_objects_info.push_back(obj_info);
    boost::shared_ptr<fcl::CollisionGeometry> cg =
        boost::shared_ptr<fcl::CollisionGeometry>(box);
    cg->setUserData((void*)(&world_objects_info[world_objects_info.size()-1]));

    world_objects_m->registerObject(new fcl::CollisionObject(cg,
                                                         p));
}

void collision_world::add_object(double dim[],
                                 double xyzrpy[],
                                 object_data obj_info)
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

    add_object(dim_vec, pos_vec, obj_info);
}

void collision_world::clear()
{
    world_objects_m->clear();
    world_objects_info.clear();
    colliding.clear();

    // TABLE
    std::vector<float> dims;
    dims.push_back(3);
    dims.push_back(3);
    dims.push_back(0.04);

    std::vector<float> pos;
    for (int i = 0; i < 6; i++) pos.push_back(0);

    object_data od;
    od.id = 0;
    od.type = "table";
    od.color = "none";

    add_object(dims, pos, od);
}

bool collision_world::collision(pose arm_position,
                                float hand_position,
                                bool should_publish)
{
    arm_objects_m->clear();
    hand_objects_m->clear();
    base_objects_m->clear();
    colliding.clear();
    arm_parts.clear();

    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        fcl::Cylinder* bounds = new
            fcl::Cylinder(fetch_arm::get_component_width(i),
                          fetch_arm::get_component_length(i));

        Eigen::Matrix4f trmat =
            fetch_arm::joint_transform(i, arm_position)*
            fetch_arm::translation_matrix(-fetch_arm::get_component_length(i)/2,
                                          0,
                                          0);
        trmat *= fetch_arm::rotation_matrix(M_PI/2, Y_AXIS);

        fcl::Matrix3f rot(trmat(0, 0), trmat(0, 1), trmat(0, 2),
                          trmat(1, 0), trmat(1, 1), trmat(1, 2),
                          trmat(2, 0), trmat(2, 1), trmat(2, 2));
        fcl::Vec3f trans(trmat(0, 3), trmat(1, 3), trmat(2, 3));
        fcl::Transform3f p = fcl::Transform3f(rot, trans);

        object_data* od = new object_data();
        od->id = -1;
        od->type = "arm";
        od->color = "none";
        boost::shared_ptr<fcl::CollisionGeometry> cg =
            boost::shared_ptr<fcl::CollisionGeometry>(bounds);
        cg->setUserData((void*)od);

        fcl::CollisionObject* obj = new fcl::CollisionObject(cg, p);
        arm_objects_m->registerObject(obj);

        arm_parts.push_back(obj);
        if (i > 1) hand_objects_m->registerObject(obj);
    }

    // HAND
    float width, len, height;
    if (has_held_object)
    {
        width = 2*hand_position + 2*fetch_arm::finger_width;
        len = fetch_arm::hand_length + held_object_dims[2] - 0.02;
        height = held_object_dims[1];
    }
    else
    {
        width = 2*hand_position + 2*fetch_arm::finger_width;
        len = fetch_arm::hand_length;
        height = fetch_arm::hand_height+0.01;
    }
    fcl::Box* box;
    box = new fcl::Box(len,
                       width,
                       height);

    int last_joint = fetch_arm::get_num_joints()-1;
    Eigen::Matrix4f trmat =
        fetch_arm::joint_transform(last_joint,
                                   arm_position);
    trmat *= fetch_arm::translation_matrix(fetch_arm::hand_length/2,
                                           0, 0);

    fcl::Matrix3f rot(trmat(0, 0), trmat(0, 1), trmat(0, 2),
                      trmat(1, 0), trmat(1, 1), trmat(1, 2),
                      trmat(2, 0), trmat(2, 1), trmat(2, 2));
    fcl::Vec3f trans(trmat(0, 3), trmat(1, 3), trmat(2, 3));
    fcl::Transform3f p = fcl::Transform3f(rot, trans);


    object_data* od = new object_data();
    od->id = -2;
    od->type = "hand";
    od->color = "none";
    boost::shared_ptr<fcl::CollisionGeometry> cg =
        boost::shared_ptr<fcl::CollisionGeometry>(box);
    cg->setUserData((void*)od);

    hand = new fcl::CollisionObject(cg, p);

    arm_objects_m->registerObject(hand);
    hand_objects_m->registerObject(hand);
    // end HAND

    // BASE
    fcl::Box* base_box = new
        fcl::Box(0.1, 0.3, fetch_arm::base_offset[2]-0.07);

    fcl::Matrix3f brot(1, 0, 0,
                      0, 1, 0,
                      0, 0, 1);
    fcl::Vec3f btrans(fetch_arm::base_offset[0],
                      fetch_arm::base_offset[1],
                      fetch_arm::base_offset[2]/2 + 0.05);
    fcl::Transform3f q = fcl::Transform3f(brot, btrans);

    object_data* od2 = new object_data();
    od2->id = -1;
    od2->type = "base";
    od2->color = "none";
    boost::shared_ptr<fcl::CollisionGeometry> cg2 =
        boost::shared_ptr<fcl::CollisionGeometry>(base_box);
    cg2->setUserData((void*)od2);

    base = new fcl::CollisionObject(cg2, q);

    base_objects_m->registerObject(base);
    arm_objects_m->registerObject(base);

#ifdef PUBLISH_COLLISION_MODEL
    if (should_publish)
    {
        publish_arm_boxes(arm_position);
    }
#endif

    // end BASE

    // std::cout << "Checking for collision of "
    //           << arm_objects_m->size()
    //           << " arm objs against "
    //           << world_objects_m->size() << " world objs"
    //           << std::endl;

    // Self-collision check
    collision_data self_data;
    self_data.request = fcl::CollisionRequest();
    self_data.request.num_max_contacts = 5;
    self_data.request.enable_contact = false;
    self_data.request.enable_cost = false;

    self_data.result = fcl::CollisionResult();

    hand_objects_m->collide(base_objects_m, &self_data,
                            collision_function);


    for (int i = 0; i < self_data.result.numContacts(); i++)
    {
        object_data* obj1 =
            (object_data*)(self_data.result.getContact(i).o1->getUserData());
        object_data* obj2 =
            (object_data*)(self_data.result.getContact(i).o2->getUserData());

        collision_pair pr = std::make_pair<object_data, object_data>(*obj1, *obj2);
        colliding.push_back(pr);
    }

    // World collision check
    collision_data data;
    data.request = fcl::CollisionRequest();
    data.request.num_max_contacts = 5;
    data.request.enable_contact = false;
    data.request.enable_cost = false;

    data.result = fcl::CollisionResult();

    arm_objects_m->collide(world_objects_m, &data,
                           collision_function);

    for (int i = 0; i < data.result.numContacts(); i++)
    {
        object_data* obj1 =
            (object_data*)(data.result.getContact(i).o1->getUserData());
        object_data* obj2 =
            (object_data*)(data.result.getContact(i).o2->getUserData());

        collision_pair pr = std::make_pair<object_data, object_data>(*obj1, *obj2);
        colliding.push_back(pr);
    }

    return (data.result.isCollision() ||
            self_data.result.isCollision());
}

void collision_world::set_held_object(std::vector<float> dims)
{
    has_held_object = true;
    held_object_dims.clear();
    for (int i = 0; i < 3; i++)
    {
        held_object_dims.push_back(dims[i]);
    }
}

void collision_world::clear_held_object()
{
    has_held_object = false;
    held_object_dims.clear();;
}

// I'm making these no longer boxes... cylinders match the arm's
// shapes better. The LCM type is going to stick with the same name though.
#ifdef PUBLISH_COLLISION_MODEL
void collision_world::publish_arm_boxes(pose arm_position)
{
    arm_collision_boxes_t cur_boxes;
    cur_boxes.len = fetch_arm::get_num_joints();
    cur_boxes.segments.clear();

    bbox_info_t base_info;
    base_info.joint = 0;
    const fcl::Box* bb = static_cast<const fcl::Box*>(base->getCollisionGeometry());
    for (int j = 0; j < 3; j++) base_info.dim[j] = bb->side[j];

    fcl::Quaternion3f qb = base->getQuatRotation();
    base_info.quat[0] = qb.getW();
    base_info.quat[1] = qb.getX();
    base_info.quat[2] = qb.getY();
    base_info.quat[3] = qb.getZ();

    fcl::Vec3f vb = base->getTranslation();
    base_info.trans[0] = vb[0];
    base_info.trans[1] = vb[1];
    base_info.trans[2] = vb[2];

    cur_boxes.base = base_info;

    bbox_info_t hand_info;
    hand_info.joint = fetch_arm::get_num_joints();
    const fcl::Box* bh = static_cast<const fcl::Box*>(hand->getCollisionGeometry());
    for (int j = 0; j < 3; j++) hand_info.dim[j] = bh->side[j];

    fcl::Quaternion3f qh = hand->getQuatRotation();
    hand_info.quat[0] = qh.getW();
    hand_info.quat[1] = qh.getX();
    hand_info.quat[2] = qh.getY();
    hand_info.quat[3] = qh.getZ();

    fcl::Vec3f vh = hand->getTranslation();
    hand_info.trans[0] = vh[0];
    hand_info.trans[1] = vh[1];
    hand_info.trans[2] = vh[2];

    cur_boxes.hand = hand_info;

    for (int i = 0; i < arm_parts.size(); i++)
    {
        cylinder_info_t cur;
        cur.joint = i;
        const fcl::Cylinder* b = static_cast<const fcl::Cylinder*>(arm_parts[i]->getCollisionGeometry());
        cur.radius = b->radius;
        cur.length = b->lz;

        fcl::Quaternion3f q = arm_parts[i]->getQuatRotation();
        cur.quat[0] = q.getW();
        cur.quat[1] = q.getX();
        cur.quat[2] = q.getY();
        cur.quat[3] = q.getZ();

        fcl::Vec3f v = arm_parts[i]->getTranslation();
        cur.trans[0] = v[0];
        cur.trans[1] = v[1];
        cur.trans[2] = v[2];

        cur_boxes.segments.push_back(cur);
    }

    lcm::LCM lcm;
    lcm.publish("ARM_COLLISION_BOXES", &cur_boxes);
}
#endif
