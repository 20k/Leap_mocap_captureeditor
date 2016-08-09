#include "../openclrenderer/proj.hpp"

#include <Leap/LeapC.h>
#include <thread>
#include <atomic>
#include <mutex>

#include "BasicExample.h"

#include "CommonExampleInterface.h"
#include "CommonGUIHelperInterface.h"
#include <LinearMath/btTransform.h>
#include "CommonRigidBodyBase.h"


///todo eventually
///split into dynamic and static objects

///todo
///fix memory management to not be atrocious

///we're completely hampered by memory latency

///rift head movement is wrong

struct positional
{
    vec3f pos;
    quat rot;
};

struct finger : positional
{
    int hand_id;
};

struct bone : positional
{
    int hand_id;
};

struct pinch
{
    quat hand_rot;
    vec3f pos;
    float pinch_strength;
    int hand_id;
};

struct leap_motion
{
    LEAP_CONNECTION connection;
    LEAP_DEVICE* device;
    LEAP_CALIBRATION calibration;

    LEAP_DEVICE_REF my_device_ref;

    LEAP_CONNECTION_CONFIG cfg;

    bool device_init = false;
    bool connection_init = false;

    std::map<uint32_t, LEAP_HAND> hand_map_mt;
    std::map<uint32_t, LEAP_HAND> hand_map;

    std::deque<std::map<uint32_t, LEAP_HAND>> hand_history_mt;
    std::deque<std::map<uint32_t, LEAP_HAND>> hand_history;
    ///1/110 = 9ms
    ///9ms * 5 = 45ms delay of smoothing
    int max_history = 5;

    int64_t start_time_us = 0;
    int64_t current_time_offset_ms = 0;

    bool start_init = false;

    LEAP_CLOCK_REBASER* rebase = new LEAP_CLOCK_REBASER;

    LEAP_TRACKING_EVENT* pEvent = (LEAP_TRACKING_EVENT*)calloc(99999, sizeof(char));


    sf::Clock clk;

    std::thread thr;

    std::atomic_int quit;

    leap_motion()
    {
        quit = 0;

        eLeapRS state = LeapCreateConnection(nullptr, &connection);

        connection_init = true;

        lg::log("LEAP\n\n\n\n\n\n\n");

        if(state != eLeapRS_Success)
        {
            lg::log("Leap not connected probly", state);
            return;
        }

        state = LeapOpenConnection(connection);

        if(state != eLeapRS_Success)
        {
            lg::log("Leap service not started probly", state);
            return;
        }

        LEAP_CONNECTION_MESSAGE* evt = new LEAP_CONNECTION_MESSAGE;

        bool connected = false;

        sf::Clock timeout_clock;
        float timeout_time_ms = 1000.f;

        while(!connected && timeout_clock.getElapsedTime().asMilliseconds() < timeout_time_ms)
        {
            state = LeapPollConnection(connection, 0, evt);

            /*if(state != eLeapRS_Success)
            {
                //lg::log("Poll error", state);
                //printf("poll %x\n", state);
                //return;
            }*/

            LEAP_CONNECTION_INFO* connection_info = new LEAP_CONNECTION_INFO;

            state = LeapGetConnectionInfo(connection, connection_info);

            int len = connection_info->size;

            //printf("Connection len %i\n", len);

            if(connection_info->status != eLeapConnectionStatus_Connected)
            {
                //printf("Not connected %x\n", connection_info->status);
                //lg::log("Leap not connected ", connection_info->status);
            }
            else
            {
                printf("Connected\n");
                connected = true;
            }
        }

        state = LeapPollConnection(connection, 1, evt);
        state = LeapPollConnection(connection, 1, evt);

        Sleep(100);

        state = LeapPollConnection(connection, 1, evt);

        uint32_t num = 0;

        state = LeapGetDeviceList(connection, nullptr, &num);

        if(num == 0)
        {
            lg::log("0 leaps connected 1 ", state);
            return;
        }

        LEAP_DEVICE_REF* device_ref = new LEAP_DEVICE_REF[num];

        state = LeapGetDeviceList(connection, device_ref, &num);

        if(num == 0)
        {
            lg::log("0 leaps connected 2");
            return;
        }

        my_device_ref = device_ref[0];

        device = new LEAP_DEVICE;

        state = LeapOpenDevice(my_device_ref, device);

        device_init = true;

        LeapCreateClockRebaser(rebase);

        thr = std::thread(&poll, this);

        if(state != eLeapRS_Success)
        {
            lg::log("device error ", state);
            return;
        }
    }

    std::mutex hand_excluder;

    void poll()
    {
        while(quit == 0)
        {
            LEAP_CONNECTION_MESSAGE evt;

            LeapPollConnection(connection, 1000, &evt);

            if(evt.type == eLeapEventType_Tracking)
            {
                hand_excluder.lock();

                hand_map_mt.clear();

                const LEAP_TRACKING_EVENT* track = evt.tracking_event;

                for(int i=0; i<track->nHands; i++)
                {
                    hand_map_mt[track->pHands[i].id] = track->pHands[i];
                }

                hand_history_mt.push_back(hand_map_mt);

                if(hand_history_mt.size() > max_history)
                    hand_history_mt.pop_front();

                hand_excluder.unlock();
            }

            Sleep(1);
        }
    }

    void tick()
    {
        if(!device_init)
            return;

        /*int64_t now = LeapGetNow();


        LeapUpdateRebase(*rebase, clk.getElapsedTime().asMicroseconds(), now);
        LeapRebaseClock(*rebase, clk.getElapsedTime().asMicroseconds(), &now);

        eLeapRS res = LeapInterpolateFrame(connection, now, pEvent, 99999);

        if(res != eLeapRS_Success)
            printf("res %x\n", res);

        hand_map.clear();

        for(int i=0; i<pEvent->nHands; i++)
        {
            hand_map[pEvent->pHands[i].id] = pEvent->pHands[i];
        }*/

        hand_excluder.lock();

        hand_map = hand_map_mt;
        hand_history = hand_history_mt;

        hand_excluder.unlock();



        int64_t now = LeapGetNow();

        //LeapUpdateRebase(*rebase, clk.getElapsedTime().asMicroseconds(), now);
        //LeapRebaseClock(*rebase, clk.getElapsedTime().asMicroseconds(), &now);

        eLeapRS res = LeapInterpolateFrame(connection, now - 1000*2, pEvent, 99999);

        if(res != eLeapRS_Success)
            printf("res %x\n", res);

        hand_map.clear();

        for(int i=0; i<pEvent->nHands; i++)
        {
            hand_map[pEvent->pHands[i].id] = pEvent->pHands[i];
        }
    }

    vec3f get_index_tip()
    {
        if(hand_map.size() == 0)
            return {0,0,0};

        LEAP_HAND first = hand_map.begin()->second;

        LEAP_VECTOR lvec = first.digits[1].stabilized_tip_position;

        printf("tv %f %f %f\n", lvec.x, lvec.y, lvec.z);

        return {lvec.x, lvec.y, -lvec.z};
    }

    std::vector<bone> get_bones(LEAP_DIGIT d)
    {
        std::vector<bone> ret;

        for(int i=0; i<4; i++)
        {
            bone b;

            LEAP_BONE lb = d.bones[i];

            vec3f prev, next;

            prev = xyz_to_vec(lb.prev_joint);
            next = xyz_to_vec(lb.next_joint);

            b.pos = (next + prev) / 2.f;

            quat q;
            q.from_vec({lb.rotation.x, lb.rotation.y, lb.rotation.z, lb.rotation.w});

            vec4f aa = q.to_axis_angle();

            aa.v[0] = -aa.v[0];
            aa.v[1] = -aa.v[1];

            q.load_from_axis_angle(aa);

            b.rot = q;

            ret.push_back(b);
        }

        return ret;
    }

    quaternion transform_leap(quaternion q)
    {
        vec4f aa = q.to_axis_angle();

        aa.v[0] = -aa.v[0];
        aa.v[1] = -aa.v[1];

        q.load_from_axis_angle(aa);

        return q;
    }

    std::vector<finger> get_fingers()
    {
        std::vector<finger> ret;

        for(auto& i : hand_map)
        {
            LEAP_HAND first = i.second;

            vec3f palm_pos = xyz_to_vec(first.palm.stabilized_position);
            palm_pos.v[2] = -palm_pos.v[2];

            for(int j=0; j<5; j++)
            {
                LEAP_DIGIT d = first.digits[j];

                finger f;
                f.pos = xyz_to_vec(d.stabilized_tip_position);
                f.pos.v[2] = -f.pos.v[2];

                f.hand_id = first.id;

                std::vector<bone> bones = get_bones(d);

                f.rot = bones.back().rot;

                ret.push_back(f);
            }
        }

        return ret;
    }

    std::vector<positional> hands_to_positional(std::map<uint32_t, LEAP_HAND>& hands)
    {
        std::vector<positional> ret;

        for(auto& i : hands)
        {
            LEAP_HAND first = i.second;

            for(int jj=0; jj<5; jj++)
            {
                LEAP_DIGIT d = first.digits[jj];

                for(int bb = 0; bb < 4; bb++)
                {
                    LEAP_BONE b = d.bones[bb];

                    positional p;

                    vec3f prev = xyz_to_vec(b.prev_joint);
                    vec3f next = xyz_to_vec(b.next_joint);

                    p.pos = (prev + next) / 2.f;
                    p.pos.v[2] = -p.pos.v[2];

                    quaternion q;
                    q.from_vec(xyzw_to_vec(b.rotation));

                    q = transform_leap(q);

                    p.rot = q;

                    ret.push_back(p);
                }
            }
        }

        return ret;
    }

    std::vector<positional> get_positionals()
    {
       return hands_to_positional(hand_map);
    }

    std::vector<positional> get_smoothed_positionals()
    {
        std::vector<positional> ret;

        if(hand_history.size() == 0)
            return get_positionals();

        int num = 0;

        for(std::map<uint32_t, LEAP_HAND>& hand : hand_history)
        {
            std::vector<positional> this_hand = hands_to_positional(hand);

            if(this_hand.size() != ret.size())
            {
                ret = this_hand;

                num = 0;
            }
            else
            {
                for(int i=0; i<ret.size(); i++)
                {
                    ret[i].pos += this_hand[i].pos;
                }
            }

            num++;
        }

        for(auto& i : ret)
        {
            i.pos = i.pos / (float)num;
        }

        return ret;
    }

    std::vector<pinch> get_pinches()
    {
        std::vector<pinch> ret;

        for(auto& i : hand_map)
        {
            LEAP_HAND& h = i.second;

            vec3f thumb_pos = xyz_to_vec(h.thumb.stabilized_tip_position);
            vec3f index_pos = xyz_to_vec(h.index.stabilized_tip_position);

            pinch p;
            p.pos = (thumb_pos*1 + index_pos) / 2.f;
            p.pos.v[2] = -p.pos.v[2];
            p.pinch_strength = h.pinch_strength;
            p.hand_id = h.id;


            LEAP_QUATERNION q = h.middle.bones[0].rotation;

            p.hand_rot = convert_leap_quaternion({{q.x, q.y, q.z, q.w}});

            ret.push_back(p);
        }

        return ret;
    }

    ~leap_motion()
    {
        quit = 1;

        thr.join();

        if(device_init)
        {
            LeapCloseDevice(*device);
            LeapDestroyClockRebaser(*rebase);

        }
        if(connection_init)
        {
            LeapDestroyConnection(connection);
        }

        free(pEvent);
    }
};

struct bbox
{
    vec3f min;
    vec3f max;
};

bool within(bbox& b, vec3f test_relative)
{
    for(int i=0; i<3; i++)
        if(test_relative.v[i] < b.min.v[i]) ///lower than minimum point, not inside cube
            return false;

    for(int i=0; i<3; i++)
        if(test_relative.v[i] >= b.max.v[i]) ///greater than maximum point, not inside cube
            return false;

    ///must be within cube
    return true;
}

bbox get_bbox(objects_container* obj)
{
    vec3f tl = {FLT_MAX, FLT_MAX, FLT_MAX}, br = {FLT_MIN, FLT_MIN, FLT_MIN};

    for(object& o : obj->objs)
    {
        for(triangle& t : o.tri_list)
        {
            for(vertex& v : t.vertices)
            {
                vec3f pos = {v.get_pos().x, v.get_pos().y, v.get_pos().z};

                tl = min(pos, tl);
                br = max(pos, br);
            }
        }
    }

    return {tl, br};
}

struct grabbable
{
    objects_container* ctr = nullptr;
    bbox b;
    btRigidBody* rigid_body = nullptr;
    int parent_id = -1;
    quaternion base_diff;

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

    CommonRigidBodyBase* bullet_scene;

    ///try decreasing max history, and using exponential averages etc
    ///or perhaps even a more explicit jitter removal algorithm
    std::deque<vec3f> history;
    int max_history = 8;

    void init(objects_container* _ctr, btRigidBody* _rigid_body, CommonRigidBodyBase* _bullet_scene)
    {
        ctr = _ctr;
        rigid_body = _rigid_body;

        b = get_bbox(ctr);

        base_diff = base_diff.identity();

        bullet_scene = _bullet_scene;
    }

    bool inside(vec3f pos)
    {
        vec3f my_pos = xyz_to_vec(ctr->pos);

        return within(b, pos - my_pos);
    }

    ///grabbed
    void parent(CommonRigidBodyBase* bullet_scene, int id, quaternion base, quaternion current_parent)
    {
        parent_id = id;

        base_diff = current_parent.get_difference(base);

        should_hand_collide = false;

        make_kinematic(bullet_scene);

        //printf("%f %f %f %f base\n", base_diff.x(), base_diff.y(), base_diff.z(), base_diff.w());
    }

    ///released for whatever reason
    void unparent(CommonRigidBodyBase* bullet_scene)
    {
        parent_id = -1;

        should_hand_collide = true;

        time_elapsed_since_release_ms = 0;

        make_dynamic(bullet_scene);
    }

    vec3f get_pos()
    {
        //btTransform newTrans;

        //rigid_body->getMotionState()->getWorldTransform(newTrans);

        //btVector3 bq = newTrans.getOrigin();

        //return {bq.x(), bq.y(), bq.z()};


        btVector3 pos = bullet_scene->getScaledPos(rigid_body);

        return {pos.x(), pos.y(), pos.z()};
    }

    quaternion get_quat()
    {
        btTransform newTrans;

        rigid_body->getMotionState()->getWorldTransform(newTrans);

        btQuaternion bq = newTrans.getRotation();

        return {{bq.x(), bq.y(), bq.z(), bq.w()}};
    }

    void set_trans(cl_float4 pos, quaternion m)
    {
        mat3f mat_diff = base_diff.get_rotation_matrix();

        mat3f current_hand = m.get_rotation_matrix();
        mat3f my_rot = current_hand * mat_diff;

        quaternion n;
        n.load_from_matrix(my_rot);


        btTransform newTrans;

        //rigid_body->getMotionState()->getWorldTransform(newTrans);

        newTrans.setOrigin(btVector3(pos.x, pos.y, pos.z));
        newTrans.setRotation(btQuaternion(n.x(), n.y(), n.z(), n.w()));

        //rigid_body->getMotionState()->setWorldTransform(newTrans);

        bullet_scene->setScaledTransform(rigid_body, newTrans);

        ctr->set_pos(pos);
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

        last_ftime = ftime;
        last_world_id = bullet_scene->info.internal_step_id;
    }
};

///need to disable finger -> collider collisions for x seconds after launch
///may be easier to simply disable whole collider -> hand collisions
struct grabbable_manager
{
    std::vector<grabbable> grabbables;
    leap_motion* motion;
    CommonRigidBodyBase* bullet_scene;

    void init(leap_motion* leap, CommonRigidBodyBase* _bullet_scene)
    {
        motion = leap;
        bullet_scene = _bullet_scene;
    }

    void add(objects_container* ctr, btRigidBody* rigid_body)
    {
        if(rigid_body->isStaticOrKinematicObject())
            return;

        grabbable g;
        g.init(ctr, rigid_body, bullet_scene);

        grabbables.push_back(g);
    }

    void tick(float ftime)
    {
        std::vector<pinch> pinches = motion->get_pinches();

        float pinch_strength = 0.2f;

        for(pinch& p : pinches)
        {
            if(p.pinch_strength < pinch_strength)
            {
                for(grabbable& g : grabbables)
                {
                    if(p.hand_id == g.parent_id)
                        g.unparent(bullet_scene);
                }

                continue;
            }

            vec3f pinch_pos = p.pos;

            for(grabbable& g : grabbables)
            {
                bool within = g.inside(pinch_pos);

                if(!within)
                    continue;

                if(g.parent_id != -1)
                    continue;

                g.parent(bullet_scene, p.hand_id, g.get_quat(), p.hand_rot);
            }
        }

        ///for some reason the inertia works if we do this here
        ///but no in tick
        ///but if down the bottom it works in tick
        ///but not overall
        for(grabbable& g : grabbables)
        {
            g.tick(ftime, bullet_scene);
        }

        for(grabbable& g : grabbables)
        {
            if(g.parent_id == -1)
                continue;

            bool unparent = true;

            for(pinch& p : pinches)
            {
                if(p.hand_id == g.parent_id)
                {
                    g.set_trans(conv_implicit<cl_float4>(p.pos), p.hand_rot);

                    unparent = false;
                }
            }

            if(unparent)
                g.unparent(bullet_scene);
        }

    }
};

struct renderable
{
    vec3f pos;
    quaternion rot;
};

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

        //scale = bullet_scene->scale;

        float mscale = scale / bullet_scene->scale;

        start_transform.setIdentity();
        cylinder = bullet_scene->createCylinderShape(btVector3(mscale, mscale, mscale));
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

            btVector3 bone_pos = btVector3(bones[i].pos.v[0], bones[i].pos.v[1], bones[i].pos.v[2]);

            newTrans.setOrigin(bone_pos);
            newTrans.setRotation(btQuat);

            bullet_scene->setScaledTransform(objects[i].kinematic, newTrans);

            //objects[i].kinematic->getMotionState()->setWorldTransform(newTrans);
        }

        for(int i=bones.size(); i<objects.size(); i++)
        {
            objects[i].ctr->hide();
        }
    }
};

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

            scale_3d = scale_3d * bullet_scene->scale;

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

            grab_manage->add(ctr, body);
        }

        btTransform trans;

        //for(auto& i : bodies)
        for(int i=0; i<objects.size() && i < bodies->size(); i++)
        {
            objects_container* obj = objects[i];
            btRigidBody* body = (*bodies)[i];

            body->getMotionState()->getWorldTransform(trans);

            //vec3f pos = {trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z()};

            btVector3 bpos = bullet_scene->getScaledPos(body);

            vec3f pos = {bpos.x(), bpos.y(), bpos.z()};

            obj->set_pos(conv_implicit<cl_float4>(pos));

            btQuaternion btQ = trans.getRotation();

            obj->set_rot_quat({{btQ.x(), btQ.y(), btQ.z(), btQ.w()}});
        }
    }
};

/*void spawn_cubes(object_context& context, grabbable_manager& grab)
{
    std::vector<objects_container*> ctr;

    for(int i=0; i<20; i++)
    {
        for(int j=0; j<20; j++)
        {
            vec3f pos = {i, 200, j};

            pos = pos * (vec3f){10, 1, 10};

            objects_container* sponza = context.make_new();
            sponza->set_file("../openclrenderer/objects/cube.obj");

            sponza->set_active(true);

            sponza->request_scale(4.f);

            sponza->set_pos(conv_implicit<cl_float4>(pos));

            sponza->cache = false;

            ctr.push_back(sponza);
        }
    }

    context.load_active();
    context.build_request();

    for(auto& i : ctr)
    {
        grab.add(i);
    }
}*/

///gamma correct mipmap filtering
///7ish pre tile deferred
///try first bounce in SS, then go to global if fail
int main(int argc, char *argv[])
{
    lg::set_logfile("./logging.txt");

    std::streambuf* b1 = std::cout.rdbuf();

    std::streambuf* b2 = lg::output->rdbuf();

    std::ios* r1 = &std::cout;
    std::ios* r2 = lg::output;

    r2->rdbuf(b1);


    sf::Clock load_time;

    object_context context;
    context.set_clear_colour({0.25, 0.25, 0.25});

    objects_container* sponza = context.make_new();
    sponza->set_file("../openclrenderer/objects/cube.obj");
    //sponza->set_load_cube_blank({10, 10, 10});

    sponza->set_active(true);


    //sponza->hide();

    engine window;

    window.load(1680,1050,1000, "turtles", "../openclrenderer/cl2.cl", true);

    window.set_camera_pos({0, 108, -169});

    //window.set_camera_pos((cl_float4){-800,150,-570});

    ///we need a context.unload_inactive
    context.load_active();

    //sponza->scale(10.f);
    //sponza->set_specular(0.f);

    context.build(true);

    ///if that is uncommented, we use a metric tonne less memory (300mb)
    ///I do not know why
    ///it might be opencl taking a bad/any time to reclaim. Investigate

    auto object_dat = context.fetch();
    window.set_object_data(*object_dat);

    //window.set_tex_data(context.fetch()->tex_gpu);

    sf::Event Event;

    light l;
    l.set_col((cl_float4){1.0f, 1.0f, 1.0f, 0.0f});
    l.set_shadow_casting(0);
    l.set_brightness(1);
    l.radius = 100000;
    l.set_pos((cl_float4){-200, 1000, -100, 0});

    light::add_light(&l);

    auto light_data = light::build();
    ///

    window.set_light_data(light_data);
    window.construct_shadowmaps();

    printf("load_time %f\n", load_time.getElapsedTime().asMicroseconds() / 1000.f);

    sf::Keyboard key;

    float avg_ftime = 6000;

    leap_motion leap;

    //spawn_cubes(context, grab_manager);

	DummyGUIHelper noGfx;

	CommonExampleOptions options(&noGfx);
	CommonRigidBodyBase*    example = BasicExampleCreateFunc(options);

    example->setScale(1000.f);
	example->initPhysics();
	example->setScaleGravity(1000.f);


    grabbable_manager grab_manager;
    grab_manager.init(&leap, example);

    leap_object_manager leap_object_spawner(&context, &leap, example);

	physics_object_manager phys(&context, example->getBodies(), example, &grab_manager);


    ///use event callbacks for rendering to make blitting to the screen and refresh
    ///asynchronous to actual bits n bobs
    ///clSetEventCallback
    while(window.window.isOpen())
    {
        sf::Clock c;

        while(window.window.pollEvent(Event))
        {
            if(Event.type == sf::Event::Closed)
                window.window.close();
        }

        example->stepSimulation(window.get_frametime_ms() / 1000.f);

        example->tick();
        phys.tick();

        compute::event event;

        {
            ///do manual async on thread
            ///make a enforce_screensize method, rather than make these hackily do it
            event = window.draw_bulk_objs_n(*context.fetch());

            //event = window.do_pseudo_aa();

            //event = window.draw_godrays(*context.fetch());

            window.increase_render_events();

            context.fetch()->swap_depth_buffers();
        }

        leap.tick();
        leap_object_spawner.tick();
        grab_manager.tick(c.getElapsedTime().asMicroseconds() / 1000.f);

        //sponza->set_pos(conv_implicit<cl_float4>(leap.get_index_tip()));

        window.set_render_event(event);

        window.blit_to_screen(*context.fetch());

        window.flip();

        window.render_block();

        context.build_tick();
        context.flush_locations();

        avg_ftime += c.getElapsedTime().asMicroseconds();

        avg_ftime /= 2;

        if(key.isKeyPressed(sf::Keyboard::M))
            std::cout << c.getElapsedTime().asMicroseconds() << std::endl;

        if(key.isKeyPressed(sf::Keyboard::Comma))
            std::cout << avg_ftime << std::endl;
    }

    example->exitPhysics();
    delete example;

    ///if we're doing async rendering on the main thread, then this is necessary
    window.render_block();
    cl::cqueue.finish();
    cl::cqueue2.finish();
    glFinish();
}
