#ifndef GRABBABLES_HPP_INCLUDED
#define GRABBABLES_HPP_INCLUDED

#include "leap_object_manager.hpp"

struct grabbable
{
    objects_container* ctr = nullptr;
    bbox b;
    btRigidBody* rigid_body = nullptr;
    int parent_id = -1;
    quaternion base_diff;
    bool self_owned = false;

    bool should_hand_collide = true;
    bool does_hand_collide = true;
    float time_elapsed_since_release_ms = 3000;
    ///with kinematic objects
    float time_needed_since_release_to_recollide_ms = 2000;

    int frame_wait_restore = 0;

    btVector3 vel_back = btVector3(0,0,0);

    bool is_kinematic = false;

    float last_ftime = 0;

    vec3f avg_vel = {0,0,0};

    uint32_t last_world_id = -1;

    vec3f offset = {0,0,0};

    bool is_parented = false;

    ///try decreasing max history, and using exponential averages etc
    ///or perhaps even a more explicit jitter removal algorithm
    std::deque<vec3f> history;
    int max_history = 8;

    void init(objects_container* _ctr, btRigidBody* _rigid_body)
    {
        ctr = _ctr;
        rigid_body = _rigid_body;

        //if(ctr)
        //    b = get_bbox(ctr);
        //else
        {
            btVector3 bmin;
            btVector3 bmax;

            btTransform none;
            none.setOrigin(btVector3(0,0,0));
            none.setRotation(btQuaternion().getIdentity());

            rigid_body->getCollisionShape()->getAabb(none, bmin, bmax);

            b.min = {bmin.x(), bmin.y(), bmin.z()};
            b.max = {bmax.x(), bmax.y(), bmax.z()};
        }

        base_diff = base_diff.identity();
    }

    void set_self_owned()
    {
        self_owned = true;
    }

    bool inside(vec3f pos)
    {
        mat3f rot;

        //if(ctr == nullptr)
        {
            btTransform trans;

            rigid_body->getMotionState()->getWorldTransform(trans);

            btVector3 bpos = trans.getOrigin();
            btQuaternion bq = trans.getRotation();

            quaternion q = {{bq.x(), bq.y(), bq.z(), bq.w()}};

            rot = q.get_rotation_matrix();

            vec3f v = {bpos.x(), bpos.y(), bpos.z()};

            vec3f rel = pos - v;

            return within(b, rot.transp() * rel);
        }

        /*rot = ctr->rot_quat.get_rotation_matrix();

        vec3f my_pos = xyz_to_vec(ctr->pos);

        return within(b, rot.transp() * (pos - my_pos));*/
    }

    ///grabbed
    void parent(CommonRigidBodyBase* bullet_scene, int id, quaternion base, quaternion current_parent, vec3f position_offset)
    {
        parent_id = id;

        base_diff = current_parent.get_difference(base);

        should_hand_collide = false;

        make_kinematic(bullet_scene);

        mat3f hand_rot = current_parent.get_rotation_matrix();

        offset = hand_rot.transp() * position_offset;

        is_parented = true;

        //printf("%f %f %f %f base\n", base_diff.x(), base_diff.y(), base_diff.z(), base_diff.w());
    }

    ///released for whatever reason
    void unparent(CommonRigidBodyBase* bullet_scene)
    {
        parent_id = -1;

        should_hand_collide = true;

        time_elapsed_since_release_ms = 0;

        make_dynamic(bullet_scene);

        is_parented = false;
    }

    vec3f get_pos()
    {
        btTransform newTrans;

        rigid_body->getMotionState()->getWorldTransform(newTrans);

        btVector3 bq = newTrans.getOrigin();

        return {bq.x(), bq.y(), bq.z()};
    }

    quaternion get_quat()
    {
        btTransform newTrans;

        rigid_body->getMotionState()->getWorldTransform(newTrans);

        btQuaternion bq = newTrans.getRotation();

        return {{bq.x(), bq.y(), bq.z(), bq.w()}};
    }

    void set_trans(cl_float4 clpos, quaternion m)
    {
        mat3f mat_diff = base_diff.get_rotation_matrix();

        mat3f current_hand = m.get_rotation_matrix();
        mat3f my_rot = current_hand * mat_diff;

        quaternion n;
        n.load_from_matrix(my_rot);


        vec3f absolute_pos = {clpos.x, clpos.y, clpos.z};

        ///current hand does not take into account the rotation offset when grabbing
        ///ie we'll double rotate
        vec3f offset_rot = current_hand * offset;

        vec3f pos = absolute_pos + offset_rot;

        btTransform newTrans;

        rigid_body->getMotionState()->getWorldTransform(newTrans);

        newTrans.setOrigin(btVector3(pos.v[0], pos.v[1], pos.v[2]));
        newTrans.setRotation(btQuaternion(n.x(), n.y(), n.z(), n.w()));

        rigid_body->getMotionState()->setWorldTransform(newTrans);

        if(ctr)
            ctr->set_pos(conv_implicit<cl_float4>(pos));
    }

    void make_dynamic(CommonRigidBodyBase* bullet_scene)
    {
        if(!is_kinematic)
            return;

        //rigid_body->saveKinematicState(1/60.f);

        toggleSaveMotion();

        bullet_scene->makeDynamic(rigid_body);

        is_kinematic = false;
    }

    void make_kinematic(CommonRigidBodyBase* bullet_scene)
    {
        if(is_kinematic)
            return;

        toggleSaveMotion();

        bullet_scene->makeKinematic(rigid_body);

        is_kinematic = true;
    }

    ///blender appears to calculate a kinematic velocity, but i have no idea how to get it ;_;
    void toggleSaveMotion()
    {
        frame_wait_restore = 2;

        vel_back = rigid_body->getLinearVelocity();
    }

    void make_no_collide_hands(CommonRigidBodyBase* bullet_scene)
    {
        if(!does_hand_collide)
            return;

        bullet_scene->changeGroup(rigid_body, collision_masks::NORMAL, collision_masks::NORMAL);

        does_hand_collide = false;
    }

    void make_collide_hands(CommonRigidBodyBase* bullet_scene)
    {
        if(does_hand_collide)
            return;

        bullet_scene->changeGroup(rigid_body, collision_masks::NORMAL, collision_masks::ALL);

        does_hand_collide = true;
    }

    void tick(float ftime, CommonRigidBodyBase* bullet_scene)
    {
        if(time_elapsed_since_release_ms >= time_needed_since_release_to_recollide_ms && should_hand_collide)
        {
            make_collide_hands(bullet_scene);
        }

        if(!should_hand_collide)
        {
            make_no_collide_hands(bullet_scene);
        }

        if(is_kinematic && last_world_id != bullet_scene->info.internal_step_id)
        {
            //rigid_body->saveKinematicState(1/60.f);

            btVector3 vel = rigid_body->getLinearVelocity();
            btVector3 angular = rigid_body->getAngularVelocity();

            //vec3f pos = get_pos();

            //printf("pos %f %f %f\n", pos.v[0], pos.v[1], pos.v[2]);

            avg_vel = avg_vel + (vec3f){vel.x(), vel.y(), vel.z()};

            avg_vel = avg_vel / 2.f;

            history.push_back({vel.x(), vel.y(), vel.z()});

            if(history.size() > max_history)
                history.pop_front();

            //printf("vel %f %f %f\n", vel.x(), vel.y(), vel.z());
        }


        time_elapsed_since_release_ms += ftime;

        //printf("%f timeelapsed\n", time_elapsed_since_release_ms);

        if(frame_wait_restore > 0)
        {
            frame_wait_restore--;

            if(frame_wait_restore == 0)
            {
                vec3f avg = {0,0,0};

                for(auto& i : history)
                {
                    avg += i;
                }

                if(history.size() > 0)
                    avg = avg / (float)history.size();

                rigid_body->setLinearVelocity({avg.v[0], avg.v[1], avg.v[2]});
            }
        }

        if(self_owned)
        {
            btTransform trans;
            rigid_body->getMotionState()->getWorldTransform(trans);

            vec3f pos = xyzf_to_vec(trans.getOrigin());
            quaternion qrot = convert_from_bullet_quaternion(trans.getRotation());

            ctr->set_pos(conv_implicit<cl_float4>(pos));
            ctr->set_rot_quat(qrot);
        }

        last_ftime = ftime;
        last_world_id = bullet_scene->info.internal_step_id;
    }

    /*vec3f get_pos()
    {
        if(ctr)
        {
            return xyz_to_vec(ctr->pos);
        }
        else
        {
            btTransform trans;

            rigid_body->getMotionState()->getWorldTransform(trans);

            btVector3 pos = trans.getOrigin();
        }
    }*/

    bool is_grabbed()
    {
        return is_parented;
    }
};

struct grab_info
{
    std::map<pinch, std::vector<grabbable>> pinches_grabbed;
};

struct by_id
{
    bool operator()(const pinch& lhs, const pinch& rhs) const
    {
        return lhs.hand_id < rhs.hand_id;
    }
};

///need to disable finger -> collider collisions for x seconds after launch
///may be easier to simply disable whole collider -> hand collisions
///need to pass in leap motion object manager, give the leap object class knowledge of which fingerbone it is
///then disable collisions for the fingertips when we're above a threshold
struct grabbable_manager
{
    std::vector<grabbable*> grabbables;
    leap_motion* motion;
    CommonRigidBodyBase* bullet_scene;
    leap_object_manager* leap_object_manage;

    void init(leap_motion* leap, CommonRigidBodyBase* _bullet_scene, leap_object_manager* _leap_object_manage)
    {
        motion = leap;
        bullet_scene = _bullet_scene;
        leap_object_manage = _leap_object_manage;
    }

    grabbable* add(objects_container* ctr, btRigidBody* rigid_body, bool override_kinematic = false)
    {
        if(rigid_body->isStaticOrKinematicObject() && !override_kinematic)
            return nullptr;

        grabbable* g = new grabbable;
        g->init(ctr, rigid_body);

        grabbables.push_back(g);

        return g;
    }

    ///if grabbed by multiple hands -> take the average
    ///implement the above now m8
    void tick(float ftime)
    {
        std::vector<pinch> pinches = motion->get_pinches();

        ///ok this needs to be hysteresis now, where we grab if > .8, but only let go if < 0.2
        float pinch_strength_to_release = 0.1f;
        float pinch_strength_to_grab = 0.8f;
        float pinch_strength_to_disable_collisions = 0.1f;

        for(pinch& p : pinches)
        {
            if(p.pinch_strength < pinch_strength_to_release)
            {
                for(grabbable* g : grabbables)
                {
                    if(p.hand_id == g->parent_id)
                        g->unparent(bullet_scene);
                }

                continue;
            }

            ///tips
            leap_object* index_object = leap_object_manage->get_leap_object(p.hand_id, 1, 3);
            leap_object* thumb_object = leap_object_manage->get_leap_object(p.hand_id, 0, 3);

            if(index_object != nullptr && thumb_object != nullptr)
            {
                bool in_scene = bullet_scene->bodyInScene(index_object->kinematic);
                in_scene |= bullet_scene->bodyInScene(thumb_object->kinematic);

                if(p.pinch_strength > pinch_strength_to_disable_collisions)
                {
                    if(in_scene)
                    {
                        bullet_scene->removeRigidBody(index_object->kinematic);
                        bullet_scene->removeRigidBody(thumb_object->kinematic);
                    }
                }
                else
                {
                    if(!in_scene)
                    {
                        bullet_scene->addRigidBody(index_object->kinematic);
                        bullet_scene->addRigidBody(thumb_object->kinematic);
                    }
                }
            }

            if(p.pinch_strength < pinch_strength_to_grab)
                continue;

            vec3f pinch_pos = p.pos;

            for(grabbable* g : grabbables)
            {
                bool within = g->inside(pinch_pos);

                if(!within)
                    continue;

                if(g->parent_id != -1)
                    continue;

                /*cl_float4 gpos = g->ctr->pos;

                vec3f gfpos = {gpos.x, gpos.y, gpos.z};*/

                vec3f gfpos = g->get_pos();

                g->parent(bullet_scene, p.hand_id, g->get_quat(), p.hand_rot, gfpos - pinch_pos);
            }
        }

        ///for some reason the inertia works if we do this here
        ///but no in tick
        ///but if down the bottom it works in tick
        ///but not overall
        ///this works for some reason now, something funky is going on with bullet kinematics
        for(grabbable* g : grabbables)
        {
            g->tick(ftime, bullet_scene);
        }

        for(grabbable* g : grabbables)
        {
            if(g->parent_id == -1)
                continue;

            bool unparent = true;

            for(pinch& p : pinches)
            {
                if(p.hand_id == g->parent_id)
                {
                    g->set_trans(conv_implicit<cl_float4>(p.pos), p.hand_rot);

                    unparent = false;
                }
            }

            if(unparent)
                g->unparent(bullet_scene);
        }
    }

    std::map<pinch, std::vector<btRigidBody*>, by_id> get_all_pinched(const std::vector<btRigidBody*>& check_bodies, float pinch_threshold)
    {
        std::vector<pinch> pinches = motion->get_pinches();

        std::map<pinch, std::vector<btRigidBody*>, by_id> ret;

        for(pinch& i : pinches)
        {
            float pinch_strength = i.pinch_strength;

            if(pinch_strength < pinch_threshold)
                continue;

            vec3f pos = i.pos;

            for(btRigidBody* body : check_bodies)
            {
                btCollisionShape* col = body->getCollisionShape();

                btVector3 center;
                btScalar radius;

                col->getBoundingSphere(center, radius);

                btTransform trans;

                body->getMotionState()->getWorldTransform(trans);

                center = center + trans.getOrigin();

                vec3f cen = {center.x(), center.y(), center.z()};

                if((cen - pos).length() > radius)
                    continue;

                ///we're a match! Hooray!
                ret[i].push_back(body);
            }
        }

        return ret;
    }
};


#endif // GRABBABLES_HPP_INCLUDED
