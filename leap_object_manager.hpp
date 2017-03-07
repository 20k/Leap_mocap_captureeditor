#ifndef LEAP_OBJECT_MANAGER_HPP_INCLUDED
#define LEAP_OBJECT_MANAGER_HPP_INCLUDED


struct leap_object
{
    objects_container* ctr;

    positional current_positional;
};

///kinematic bullet
///gotta smooth out the hands, they're too jittery to directly interact with bullet
struct leap_object_manager
{
    object_context* context;
    leap_motion* motion;

    float scale = 10.f;

    leap_object_manager(object_context* _context, leap_motion* _motion)
    {
        context = _context;
        motion = _motion;
    }

    std::vector<leap_object> objects;

    leap_object* get_leap_object(int hand_id, int finger_num, int bone_num)
    {
        for(auto& i : objects)
        {
            positional& p = i.current_positional;

            if(p.hand_id == hand_id && p.finger_num == finger_num && p.bone_num == bone_num)
                return &i;
        }

        ///shouldn't happen
        return nullptr;
    }

    leap_object* get_hand(int type, int finger_num, int bone_num)
    {
        for(auto& i : objects)
        {
            positional& p = i.current_positional;

            if(p.type == type && p.finger_num == finger_num && p.bone_num == bone_num)
                return &i;
        }

        return nullptr;
    }

    std::vector<leap_object> get_hand_objects(int hand_id)
    {
        std::vector<leap_object> ret;

        for(auto& i : objects)
        {
            if(i.current_positional.hand_id == hand_id)
            {
                ret.push_back(i);
            }
        }

        return ret;
    }

    ///need to be able to scale down the entire thing
    void tick(float dynamic_scale = 1.f)
    {
        ///hand_history not interpolated, but hand_map is
        ///this may cause some of the jitteryness. Try experimenting with this
        ///turns out its prolly not that, although its more responsive
        std::vector<positional> bones = motion->get_smoothed_positionals(1);

        //std::vector<positional> bones = motion->get_positionals();

        for(int i=objects.size(); i<bones.size(); i++)
        {
            objects_container* ctr = context->make_new();
            ctr->set_file("../openclrenderer/objects/high_cylinder_forward.obj");
            ctr->set_active(true);
            ///requesting scale will break caching, we cant have a bunch of differently scaled, yet cached objects
            ///yet. Its possible, i just need to figure it out cleanly
            //ctr->request_scale(scale);

            context->load_active();
            ctr->set_dynamic_scale(scale);
            context->build_request();

            leap_object obj;
            obj.ctr = ctr;

            objects.push_back(obj);
        }

        ///bones.size() strictly <= objects.size()
        for(int i=0; i<bones.size(); i++)
        {
            leap_object* lobj = get_leap_object(bones[i].hand_id, 2, 0);

            vec3f middle_of_hand = {0,0,0};

            if(lobj)
                middle_of_hand = lobj->current_positional.pos;

            vec3f cpos = bones[i].pos;

            vec3f middle_to_outside = cpos - middle_of_hand;

            middle_to_outside = middle_to_outside * dynamic_scale;

            vec3f final_pos = middle_to_outside + middle_of_hand;

            objects[i].ctr->set_pos(conv_implicit<cl_float4>(final_pos));
            objects[i].ctr->set_rot_quat(bones[i].rot);
            objects[i].ctr->set_dynamic_scale(scale * dynamic_scale);

            objects[i].current_positional = bones[i];
        }

        for(int i=bones.size(); i<objects.size(); i++)
        {
            objects[i].ctr->hide();
        }
    }

    void handle_ctr(leap_object* obj, objects_container* ctr1, int which = 1)
    {
        mat3f r = mat3f().XRot(-M_PI/2) * mat3f().ZRot(M_PI);

        quaternion q;
        q.load_from_matrix(r);

        ctr1->set_rot_quat(q);

        float h_fov = motion->device_info.h_fov;
        float v_fov = motion->device_info.v_fov;


        vec3f hand_pos = obj->current_positional.pos;

        //vec3f right_camera_position = {motion->camera_offset_x_mm * which, 0, 0};

        ///take distance to camera, correct for depth projection (rough)

        ctr1->set_pos({motion->camera_offset_x_mm*2 * which, obj->current_positional.pos.y() - 50, 0});

        float height = obj->current_positional.pos.y();


        float half_dim = height * tan(motion->image_half_angle_rad);

        half_dim *= 2;

        float base = 400;

        float scale = half_dim / base;

        ctr1->set_dynamic_scale(scale);
    }

    void tick_image_processing()
    {
        objects_container* ctr1 = motion->ctr_1;
        objects_container* ctr2 = motion->ctr_2;

        if(ctr1 == nullptr || ctr2 == nullptr)
            return;

        if(!motion->should_get_images)
            return;

        leap_object* rhand = get_hand(eLeapHandType_Right, 2, 0);

        if(!rhand)
        {
            ctr1->set_pos({0,0,0});
            //return;
        }
        else
            handle_ctr(rhand, ctr1);

        leap_object* lhand = get_hand(eLeapHandType_Left, 2, 0);

        if(!lhand)
            ctr2->set_pos({0,0,0});
        else
            handle_ctr(lhand, ctr2, -1);

        //static float f = 1;
        //f += 0.1f;

        //ctr1->set_dynamic_scale(f);

        //printf("%f\n", h_fov / v_fov);

        //printf("h %f %f\n", h_fov, v_fov);
    }

    std::vector<leap_object> get_objects()
    {
        return objects;
    }
};


#endif // LEAP_OBJECT_MANAGER_HPP_INCLUDED
