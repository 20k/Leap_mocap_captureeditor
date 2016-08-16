#ifndef HAND_TO_HAND_INTERACT_HPP_INCLUDED
#define HAND_TO_HAND_INTERACT_HPP_INCLUDED

struct hand_to_hand_interactor
{
    leap_object_manager* leap_object_manage;
    grabbable_manager* grab_manage;
    leap_motion* motion;
    object_context* context;
    CommonRigidBodyBase* bullet_scene;

    float scale = 10.f;

    float cylinder_length = scale * 2;

    float cylinder_separation = cylinder_length / 4;

    void init(leap_object_manager* _leap_object_manage, grabbable_manager* _grab_manage, leap_motion* _motion, object_context* _context, CommonRigidBodyBase* _bullet_scene)
    {
        leap_object_manage = _leap_object_manage;
        grab_manage = _grab_manage;
        motion = _motion;
        context = _context;
        bullet_scene = _bullet_scene;
    }

    struct slide_in_progress
    {
        int reference_hand_id = -1;
        int grabber_hand_id = -1;
        float saved_len = 0;

        std::vector<objects_container*> objs;
    };

    struct finished_slide
    {
        int reference_hand_id = -1;
        int grabber_hand_id = -1;
        float len = 0;
        grabbable* g = nullptr;
        bool suppress = false;

        std::vector<objects_container*> objs;
    };

    std::map<int, bool> hand_to_finger_is_extended;
    std::vector<slide_in_progress> slides;
    std::vector<finished_slide> finished;

    void tick()
    {
        ///kinematic hands
        ///if we simply cut down these objects to the valid ones
        ///might make this simpler
        std::vector<leap_object> objects = leap_object_manage->get_objects();

        std::vector<btRigidBody*> bodies;
        std::map<btRigidBody*, leap_object*> rigid_to_leap;

        for(auto& i : objects)
        {
            bodies.push_back(i.kinematic);

            rigid_to_leap[i.kinematic] = &i;
        }

        std::map<pinch, std::vector<btRigidBody*>, by_id> hand_pinches = grab_manage->get_all_pinched(bodies, 0.2f);

        //printf("ssadfasdf %i\n", hand_pinches.size());

        //std::vector<int> pincher_ids;

        for(auto& ii : hand_pinches)
        {
            pinch p = ii.first;
            std::vector<btRigidBody*>& kinematic = ii.second;

            //pincher_ids.push_back(p.hand_id);

            for(auto& kk : kinematic)
            {
                leap_object* lobj = rigid_to_leap[kk];

                if(lobj->current_positional.bone_num != 3 || lobj->current_positional.finger_num != 1)
                    continue;

                ///we grabbed ourself
                if(p.hand_id == lobj->current_positional.hand_id)
                    continue;

                if(hand_to_finger_is_extended[lobj->current_positional.hand_id])
                    continue;

                ///we've grabbed the tip of our first finger

                bool good = true;

                for(auto& jj : slides)
                {
                    ///one of the pinched hands is the stored reference hand
                    if(jj.reference_hand_id == lobj->current_positional.hand_id)
                    {
                        good = false;
                        break;
                    }
                }

                ///slide already exists
                if(!good)
                    continue;

                slide_in_progress s;
                s.grabber_hand_id = p.hand_id;
                s.reference_hand_id = lobj->current_positional.hand_id;

                slides.push_back(s);

                printf("add pinch\n");
            }
        }

        std::vector<pinch> pinches = motion->get_pinches();

        for(int i=0; i<slides.size(); i++)
        {
            slide_in_progress slide = slides[i];

            bool in_progress = false;

            for(auto& ii : pinches)
            {
                const pinch& p = ii;

                if(p.hand_id == slide.grabber_hand_id && p.pinch_strength > 0.1f)
                {
                    in_progress = true;
                    break;
                }
            }

            if(!in_progress)
            {
                terminate_slide(slides[i]);

                slides.erase(slides.begin() + i);
                i--;
            }
            ///update slider model
            else
            {

            }
        }

        for(auto& i : slides)
        {
            tick_slide(i);
        }

        for(auto& i : finished)
        {
            if(i.suppress)
                continue;

            tick_finished(i);
        }

        for(int i=0; i<finished.size(); i++)
        {
            finished_slide& fs = finished[i];

            if(fs.g->is_grabbed() && !fs.suppress && fs.g->parent_id != fs.reference_hand_id)
            {
                lg::log("HO HO");

                hand_to_finger_is_extended[fs.reference_hand_id] = false;

                objects_container* base = context->make_new();
                base->isloaded = true;
                base->cache = false; //?

                fs.g->ctr = base;

                int n = get_num_from_len(fs.len);

                vec3f avg = get_avg_pos(fs.objs, n);

                btTransform trans;
                fs.g->rigid_body->getMotionState()->getWorldTransform(trans);

                quaternion world_rot = convert_from_bullet_quaternion(trans.getRotation());

                vec3f dir = {0, 0, 1};

                float gap = (cylinder_length + cylinder_separation);

                vec3f base_pos = {0, 0, -fs.len/2.f + cylinder_separation + cylinder_length};

                for(int i=0; i<fs.objs.size() && i < n; i++)
                {
                    objects_container* obj = fs.objs[i];

                    vec3f offset = base_pos;

                    obj->set_pos(conv_implicit<cl_float4>(offset));
                    obj->set_rot_quat({{0,0,0,1}});

                    obj->set_parent(base);

                    base_pos = base_pos + dir * gap;
                }

                for(int i=n; i<fs.objs.size(); i++)
                {
                    fs.objs[i]->set_active(false);
                }

                base->set_pos(conv_implicit<cl_float4>(avg));
                base->set_rot_quat(world_rot);

                fs.g->is_kinematic = true;
                fs.g->set_self_owned();

                //finished.erase(finished.begin() + i);
                //i--;

                fs.suppress = true;
            }

            if(fs.suppress)
            {
                /*for(auto& i : fs.g->ctr->transform_children)
                {
                    vec3f pos = xyz_to_vec(i->pos);

                    printf("%f %f %f lpos\n", pos.v[0], pos.v[1], pos.v[2]);
                }*/

                vec3f pos = xyz_to_vec(fs.g->ctr->pos);

                btTransform trans;
                fs.g->rigid_body->getMotionState()->getWorldTransform(trans);

                vec3f physpos = xyzf_to_vec(trans.getOrigin());

                //printf("%f %f %f lpos\n", pos.v[0], pos.v[1], pos.v[2]);
                //printf("%f %f %f phys\n", physpos.v[0], physpos.v[1], physpos.v[2]);
            }
        }
    }

    int get_num_from_len(float len)
    {
        return floor(len / (cylinder_length + cylinder_separation)) - 1.f;
    }

    ///I guess keep ticking until we try to grab it? we just want to
    ///disable the extension mechanic if we're not pinching
    ///wrong terminate should do nothing, except wiat for a enw pinch to spawn the object
    void tick_slide(slide_in_progress& s)
    {
        leap_object* base_obj = leap_object_manage->get_leap_object(s.reference_hand_id, 1, 3);
        leap_object* grab_obj = leap_object_manage->get_leap_object(s.grabber_hand_id, 1, 3);

        if(base_obj == nullptr || grab_obj == nullptr)
            return;

        vec3f base_pos = xyz_to_vec(base_obj->ctr->pos);

        mat3f base_rot = base_obj->ctr->rot_quat.get_rotation_matrix();

        vec3f grab_pos = xyz_to_vec(grab_obj->ctr->pos);

        ///do proper perpendicular check later
        vec3f ddir = (grab_pos - base_pos);

        float len = ddir.length();

        s.saved_len = len;

        vec3f dir = base_rot * (vec3f){0, 0, 1};

        base_pos = base_pos + dir.norm() * (cylinder_length + cylinder_separation);

        int n = get_num_from_len(len);

        if(n < 0)
            return;

        int batch = 10;

        int n_rounded = n;

        if((n % batch) != 0)
            n_rounded = n - (n % batch) + batch;


        for(int i=s.objs.size(); i <= n_rounded; i++)
        {
            objects_container* ctr = context->make_new();
            ctr->set_file("../openclrenderer/objects/high_cylinder_forward.obj");
            ctr->set_active(true);
            ///requesting scale will break caching, we cant have a bunch of differently scaled, yet cached objects
            ///yet. Its possible, i just need to figure it out cleanly
            ctr->request_scale(10.f);

            context->build_request();

            s.objs.push_back(ctr);
        }

        context->load_active();

        for(int i=0; i<n; i++)
        {
            objects_container* ctr = s.objs[i];

            ctr->set_pos({base_pos.v[0], base_pos.v[1], base_pos.v[2]});
            ctr->set_rot_quat(base_obj->ctr->rot_quat);

            base_pos = base_pos + dir.norm() * (cylinder_length + cylinder_separation);
        }

        for(int i=n; i<s.objs.size(); i++)
        {
            s.objs[i]->hide();
        }
    }

    void tick_finished(finished_slide& fs)
    {
        int n = get_num_from_len(fs.len);

        if(n <= 0)
            return;

        leap_object* base_obj = leap_object_manage->get_leap_object(fs.reference_hand_id, 1, 3);

        if(base_obj == nullptr)
            return;

        vec3f base_pos = xyz_to_vec(base_obj->ctr->pos);

        quaternion base_rot_quat = base_obj->ctr->rot_quat;

        mat3f base_rot = base_rot_quat.get_rotation_matrix();

        vec3f dir = base_rot * (vec3f){0, 0, 1};

        base_pos = base_pos + dir.norm() * (cylinder_length + cylinder_separation);

        for(int i=0; i<n; i++)
        {
            objects_container* ctr = fs.objs[i];

            ctr->set_pos({base_pos.v[0], base_pos.v[1], base_pos.v[2]});
            ctr->set_rot_quat(base_obj->ctr->rot_quat);

            base_pos = base_pos + dir.norm() * (cylinder_length + cylinder_separation);
        }

        vec3f avg_pos = get_avg_pos(fs.objs, n);

        btRigidBody* body = fs.g->rigid_body;
        body->setFriction(2.f);
        body->setRollingFriction(2.f);

        btTransform trans;
        trans.setOrigin(btVector3(avg_pos.v[0], avg_pos.v[1], avg_pos.v[2]));
        trans.setRotation(btQuaternion(base_rot_quat.x(), base_rot_quat.y(), base_rot_quat.z(), base_rot_quat.w()));

        body->getMotionState()->setWorldTransform(trans);

        fs.g->make_kinematic(bullet_scene);

        //printf("T %f %f %f\n", EXPAND_3(avg_pos));
    }

    vec3f get_avg_pos(const std::vector<objects_container*>& ctr, int nmax = INT_MAX)
    {
        if(ctr.size() == 0 || nmax == 0)
            return {0,0,0};

        vec3f avg = {0,0,0};

        int num = std::min((int)ctr.size(), nmax);

        for(int i=0; i<ctr.size() && i < nmax; i++)
        {
            avg = avg + xyz_to_vec(ctr[i]->pos);
        }

        return avg / num;
    }

    void terminate_slide(slide_in_progress& s)
    {
        lg::log("Slide terminated");

        leap_object* base_obj = leap_object_manage->get_leap_object(s.reference_hand_id, 1, 3);

        if(base_obj == nullptr)
            return;

        finished_slide fs;
        fs.reference_hand_id = s.reference_hand_id;
        fs.grabber_hand_id = s.grabber_hand_id;
        fs.objs = std::move(s.objs);
        fs.len = s.saved_len;

        int n = get_num_from_len(fs.len);

        float real_len = n * (cylinder_length + cylinder_separation) - cylinder_separation;

        ///z forward reference
        btBoxShape* box = bullet_scene->createBoxShape(btVector3(scale, scale, real_len/2));

        vec3f center = get_avg_pos(fs.objs, n);
        quaternion rot_quat = base_obj->ctr->rot_quat;

        btTransform start;
        start.setOrigin(btVector3(center.x(), center.y(), center.z()));
        start.setRotation(btQuaternion(rot_quat.x(), rot_quat.y(), rot_quat.z(), rot_quat.w()));

        btRigidBody* body = bullet_scene->createRigidBody(1.f * n, start, box);

        bullet_scene->makeKinematic(body);

        fs.g = grab_manage->add(nullptr, body, true);

        hand_to_finger_is_extended[fs.reference_hand_id] = true;

        finished.push_back(fs);

        lg::log("End of slide terminated");

        ///push to finished slide queue
        ///convert objects to regular old rigid body
        ///push to grabbables
    }
};

#endif // HAND_TO_HAND_INTERACT_HPP_INCLUDED
