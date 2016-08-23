#ifndef PHYSICS_OBJECT_MANAGER_HPP_INCLUDED
#define PHYSICS_OBJECT_MANAGER_HPP_INCLUDED


///dynamic bullet
struct physics_object_manager
{
    object_context* context;
    std::vector<btRigidBody*>* bodies;

    std::vector<objects_container*> objects;

    CommonRigidBodyBase* bullet_scene;

    grabbable_manager* grab_manage;

    physics_object_manager(object_context* _context, std::vector<btRigidBody*>* _bodies, CommonRigidBodyBase* _bullet_scene, grabbable_manager* _grab_manage)
    {
        context = _context;
        bodies = _bodies;
        bullet_scene = _bullet_scene;
        grab_manage = _grab_manage;
    }

    void tick()
    {
        for(int i=objects.size(); i<bodies->size(); i++)
        {
            btRigidBody* body = (*bodies)[i];

            btVector3 aabbMin, aabbMax;

            body->getAabb(aabbMin,aabbMax);

            btVector3 diff = aabbMax - aabbMin;

            vec3f scale_3d = {diff.x(), diff.y(), diff.z()};
            scale_3d = scale_3d / 2.f;

            //printf("%f scale\n", cur_scale);

            objects_container* ctr = context->make_new();
            ctr->set_file("../openclrenderer/objects/cube.obj");
            ctr->set_active(true);
            ///requesting scale will break caching, we cant have a bunch of differently scaled, yet cached objects
            ///yet. It's possible, i just need to figure it out cleanly
            ctr->cache = false;

            context->load_active();

            ctr->scale(conv_implicit<cl_float3>(scale_3d));

            context->build_request();

            objects.push_back(ctr);

            grab_manage->add(ctr, body, false, true);
        }

        btTransform trans;

        //for(auto& i : bodies)
        for(int i=0; i<objects.size() && i < bodies->size(); i++)
        {
            objects_container* obj = objects[i];
            btRigidBody* body = (*bodies)[i];

            body->getMotionState()->getWorldTransform(trans);

            vec3f pos = {trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z()};

            obj->set_pos(conv_implicit<cl_float4>(pos));

            btQuaternion btQ = trans.getRotation();

            obj->set_rot_quat({{btQ.x(), btQ.y(), btQ.z(), btQ.w()}});
        }
    }
};

#endif // PHYSICS_OBJECT_MANAGER_HPP_INCLUDED
