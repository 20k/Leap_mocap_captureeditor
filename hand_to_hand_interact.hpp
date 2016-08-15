#ifndef HAND_TO_HAND_INTERACT_HPP_INCLUDED
#define HAND_TO_HAND_INTERACT_HPP_INCLUDED

struct hand_to_hand_interactor
{
    leap_object_manager* leap_object_manage;
    grabbable_manager* grab_manage;
    leap_motion* motion;
    object_context* context;

    void init(leap_object_manager* _leap_object_manage, grabbable_manager* _grab_manage, leap_motion* _motion, object_context* _context)
    {
        leap_object_manage = _leap_object_manage;
        grab_manage = _grab_manage;
        motion = _motion;
        context = _context;
    }

    struct slide_in_progress
    {
        int reference_hand_id = -1;
        int grabber_hand_id = -1;

        std::vector<objects_container*> objs;
    };

    struct finished_slide
    {
        btRigidBody* body;

        std::vector<objects_container*> objs;
    };

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
    }

    void tick_slide(slide_in_progress& s)
    {
        float scale = 10.f;

        float cylinder_length = scale * 2;

        float cylinder_separation = cylinder_length / 4;

        std::vector<leap_object> objects = leap_object_manage->get_objects();

        leap_object* base_obj = nullptr;
        leap_object* grab_obj = nullptr;

        for(leap_object& obj : objects)
        {
            if(obj.current_positional.hand_id == s.reference_hand_id && obj.current_positional.bone_num == 3 && obj.current_positional.finger_num == 1)
                base_obj = &obj;

            if(obj.current_positional.hand_id == s.grabber_hand_id && obj.current_positional.bone_num == 3 && obj.current_positional.finger_num == 1)
                grab_obj = &obj;
        }

        if(base_obj == nullptr || grab_obj == nullptr)
            return;

        vec3f base_pos = xyz_to_vec(base_obj->ctr->pos);

        mat3f base_rot = base_obj->ctr->rot_quat.get_rotation_matrix();

        vec3f grab_pos = xyz_to_vec(grab_obj->ctr->pos);

        ///do proper perpendicular check later
        vec3f ddir = (grab_pos - base_pos);

        float len = ddir.length();

        vec3f dir = base_rot * (vec3f){0, 0, 1};

        base_pos = base_pos + dir.norm() * (cylinder_length + cylinder_separation);

        int n = ceil(len / (cylinder_length + cylinder_separation)) - 1.f;

        if(n <= 0)
            return;

        for(int i=s.objs.size(); i <= n; i++)
        {
            objects_container* ctr = context->make_new();
            ctr->set_file("../openclrenderer/objects/high_cylinder_forward.obj");
            ctr->set_active(true);
            ///requesting scale will break caching, we cant have a bunch of differently scaled, yet cached objects
            ///yet. Its possible, i just need to figure it out cleanly
            ctr->request_scale(10.f);

            context->load_active();
            context->build_request();

            s.objs.push_back(ctr);
        }

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

    void terminate_slide(slide_in_progress& s)
    {
        lg::log("Slide terminated");

        ///push to finished slide queue
        ///convert objects to regular old rigid body
        ///push to grabbables
    }
};

#endif // HAND_TO_HAND_INTERACT_HPP_INCLUDED
