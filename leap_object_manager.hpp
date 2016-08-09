#ifndef LEAP_OBJECT_MANAGER_HPP_INCLUDED
#define LEAP_OBJECT_MANAGER_HPP_INCLUDED


struct leap_object
{
    objects_container* ctr;
    btRigidBody* kinematic;
};

///kinematic bullet
///gotta smooth out the hands, they're too jittery to directly interact with bullet
struct leap_object_manager
{
    object_context* context;
    leap_motion* motion;
    CommonRigidBodyBase* bullet_scene;
    btTransform start_transform;
    btCylinderShape* cylinder;

    float scale = 10.f;

    leap_object_manager(object_context* _context, leap_motion* _motion, CommonRigidBodyBase* _bullet_scene)
    {
        context = _context;
        motion = _motion;
        bullet_scene = _bullet_scene;

        start_transform.setIdentity();
        cylinder = bullet_scene->createCylinderShape(btVector3(scale, scale, scale));
    }

    std::vector<leap_object> objects;

    void tick()
    {
        std::vector<positional> bones = motion->get_smoothed_positionals();

        for(int i=objects.size(); i<bones.size(); i++)
        {
            objects_container* ctr = context->make_new();
            ctr->set_file("../openclrenderer/objects/high_cylinder_forward.obj");
            ctr->set_active(true);
            ///requesting scale will break caching, we cant have a bunch of differently scaled, yet cached objects
            ///yet. Its possible, i just need to figure it out cleanly
            ctr->request_scale(scale);

            context->load_active();
            context->build_request();

            btRigidBody* body = bullet_scene->createKinematicRigidBody(1.f, start_transform, cylinder);

            body->setFriction(3.f);
            body->setRollingFriction(3.f);

            //btBoxShape* col = body->getCollisionShape();

            //col->setLocalScaling(btVector3(ctr->get_final_scale(), ctr->get_final_scale(), ctr->get_final_scale()));

            leap_object obj;
            obj.ctr = ctr;
            obj.kinematic = body;

            objects.push_back(obj);
        }

        for(int i=0; i<bones.size(); i++)
        {
            mat3f mat = bones[i].rot.get_rotation_matrix();

            mat3f xrot = mat3f().XRot(M_PI/2);

            mat3f comb = mat * xrot;

            quat rquat;
            rquat.load_from_matrix(comb);

            btQuaternion btQuat;

            btQuat.setX(rquat.x());
            btQuat.setY(rquat.y());
            btQuat.setZ(rquat.z());
            btQuat.setW(rquat.w());

            objects[i].ctr->set_pos(conv_implicit<cl_float4>(bones[i].pos));
            objects[i].ctr->set_rot_quat(bones[i].rot);

            btTransform newTrans;

            newTrans.setOrigin(btVector3(bones[i].pos.v[0], bones[i].pos.v[1], bones[i].pos.v[2]));
            newTrans.setRotation(btQuat);

            objects[i].kinematic->getMotionState()->setWorldTransform(newTrans);

            if(!bullet_scene->bodyInScene(objects[i].kinematic))
            {
                bullet_scene->addRigidBody(objects[i].kinematic);
            }
        }

        for(int i=bones.size(); i<objects.size(); i++)
        {
            objects[i].ctr->hide();

            if(!bullet_scene->bodyInScene(objects[i].kinematic))
            {
                continue;
            }

            cl_float4 pos = objects[i].ctr->pos;


            btTransform newTrans;

            objects[i].kinematic->getMotionState()->getWorldTransform(newTrans);

            newTrans.setOrigin(btVector3(pos.x, pos.y, pos.z));

            objects[i].kinematic->getMotionState()->setWorldTransform(newTrans);

            bullet_scene->removeRigidBody(objects[i].kinematic);
        }
    }
};


#endif // LEAP_OBJECT_MANAGER_HPP_INCLUDED
