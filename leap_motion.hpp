#ifndef LEAP_MOTION_H_INCLUDED
#define LEAP_MOTION_H_INCLUDED


struct positional
{
    vec3f pos;
    quat rot;

    ///0 -> 3
    int bone_num = -1;
    ///0 -> 4
    int finger_num = -1;
    ///who knows. Well, the leap motion people do
    int hand_id = -1;
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
    vec3f hand_pos;
    quat hand_rot;
    vec3f pos;
    float pinch_strength;
    float derived_pinch_strength;
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
    int max_history = 20;

    int64_t start_time_us = 0;
    int64_t current_time_offset_ms = 0;

    ///for image
    uint64_t required_buffer_len = 0;
    bool buffer_required_reallocate = false;
    void* buffer_for_image = nullptr;
    void* returned_buffer_mt = nullptr; ///these two have the same value, but because its returned from leap, cannot be optimised away
    int64_t last_frame_id_mt = 0;
    int64_t last_frame_id = 0;
    bool new_frame_available_mt = true; ///so we can make sure we dont request the same two images in a row
    bool request_finished_mt = true;
    bool request_available_for_processing_mt = false;

    uint32_t bpp_mt = 0, width_mt = 0, height_mt = 0;
    float xscale_mt = 0, yscale_mt = 0;
    uint32_t width = 0, height = 0;
    int current_width = 0, current_height = 0;

    bool start_init = false;
    bool should_get_images = false;
    objects_container* ctr = nullptr;

    LEAP_CLOCK_REBASER* rebase = new LEAP_CLOCK_REBASER;

    LEAP_TRACKING_EVENT* pEvent = (LEAP_TRACKING_EVENT*)calloc(99999, sizeof(char));

    LEAP_IMAGE_FRAME_REQUEST_TOKEN token;
    LEAP_IMAGE_FRAME_DESCRIPTION descrip;

    sf::Clock clk;

    std::thread thr;

    std::atomic_int quit;

    void enable_images(object_context& ctx)
    {
        ctr = ctx.make_new();

        should_get_images = true;
    }

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
    std::mutex buffer_excluder;
    std::mutex frame_excluder;

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

                if(last_frame_id_mt != track->info.frame_id)
                    new_frame_available_mt = true;

                last_frame_id_mt = track->info.frame_id;

                hand_excluder.unlock();
            }

            if(evt.type == eLeapEventType_ImageComplete)
            {
                const LEAP_IMAGE_COMPLETE_EVENT* image_complete_event = evt.image_complete_event;

                buffer_excluder.lock();
                request_finished_mt = true;
                request_available_for_processing_mt = true;

                bpp_mt = image_complete_event->properties->bpp;
                width_mt = image_complete_event->properties->width;
                height_mt = image_complete_event->properties->height;
                xscale_mt = image_complete_event->properties->x_scale;
                yscale_mt = image_complete_event->properties->y_scale;
                returned_buffer_mt = image_complete_event->pfnData;

                //printf("got image with id %i\n", image_complete_event->info.frame_id);
                buffer_excluder.unlock();
            }

            if(evt.type == eLeapEventType_ImageRequestError)
            {
                const LEAP_IMAGE_FRAME_REQUEST_ERROR_EVENT* image_request_error_event = evt.image_request_error_event;

                buffer_excluder.lock();

                if(image_request_error_event->error == eLeapImageRequestError_InsufficientBuffer)
                {
                    if(required_buffer_len != image_request_error_event->required_buffer_len)
                        buffer_required_reallocate = true;

                    required_buffer_len = image_request_error_event->required_buffer_len;
                }

                request_finished_mt = true;

                buffer_excluder.unlock();
            }

            Sleep(1);
        }
    }

    void handle_image_processing(uint32_t len)
    {
        ///same thread as reallocation so no async needed
        buffer_excluder.lock();
        uint8_t* ptr = (uint8_t*)returned_buffer_mt;

        uint8_t* upper_half = (uint8_t*)malloc(sizeof(uint8_t)*len/2);
        uint8_t* lower_half = (uint8_t*)malloc(sizeof(uint8_t)*len/2);

        memcpy(upper_half, ptr, sizeof(uint8_t)*len/2);
        memcpy(lower_half, ptr + len/2, sizeof(uint8_t)*len/2);
        //buffer_excluder.unlock();

        //buffer_excluder.lock();
        int w = width_mt;
        int h = height_mt;
        int bpp = bpp_mt;
        int xscale = xscale_mt;
        int yscale = yscale_mt;
        buffer_excluder.unlock();

        object_context& ctx = *ctr->parent;
        texture_context& tex_ctx = ctx.tex_ctx;

        //h *= 2;

        ///won't realloc texture atm
        if(!ctr->isactive)
        {
            texture* ntex = tex_ctx.make_new();
            ///will need to be twice the width I think due to stereoscopy
            ntex->set_create_colour(sf::Color(255, 0, 255), w, h);

            printf("Made new with %i %i\n", w, h);

            //cl_float2 dim = {w, h};

            cl_float2 dim = {200, 200};

            ctr->set_load_func(std::bind(obj_rect, std::placeholders::_1, *ntex, dim));

            ctr->set_active(true);

            ctx.load_active();

            ctr->set_two_sided(true);
            ctr->patch_stretch_texture_to_full();

            ctr->scale({xscale, yscale/2, 1.f});

            ctx.build(true); ///we need the gpu data there now
        }

        //printf("bpp %i\n", bpp);

        ///should have at least one object as per above
        texture* tex = tex_ctx.id_to_tex(ctr->objs[0].tid);

        if(tex == nullptr)
        {
            lg::log("Wtf texture == nullptr");
            return;
        }

        tex->update_gpu_texture_mono(ctx.get_current_gpu()->tex_gpu_ctx, upper_half, len/2, w, h, false);

        ctr->set_rot({0,0,M_PI});
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
        last_frame_id = last_frame_id_mt;

        hand_excluder.unlock();

        uint64_t len = 0;
        bool reallocate = false;

        buffer_excluder.lock();
        len = required_buffer_len;
        reallocate = buffer_required_reallocate;
        buffer_excluder.unlock();

        if(reallocate)
        {
            frame_excluder.lock();

            //if(buffer_for_image)
            //    delete [] buffer_for_image;

            ///its easier to leak on resize as
            ///1. itll never happen
            ///2. if we free the image, leap motion will try to write into it and crash
            buffer_for_image = malloc(sizeof(uint8_t)*len);

            frame_excluder.unlock();
        }

        descrip.frame_id = last_frame_id;
        descrip.buffer_len = len;
        descrip.pBuffer = buffer_for_image;
        descrip.type = eLeapImageType_Default;

        bool is_finished;

        buffer_excluder.lock();
        is_finished = request_finished_mt;
        buffer_excluder.unlock();

        bool new_frame_available;

        hand_excluder.lock();
        new_frame_available = new_frame_available_mt;
        hand_excluder.unlock();

        ///process images
        bool image_available;

        buffer_excluder.lock();
        image_available = request_available_for_processing_mt;
        request_available_for_processing_mt = false;
        buffer_excluder.unlock();

        if(image_available)
        {
            handle_image_processing(len);
        }
        ///end process

        if(is_finished && new_frame_available && should_get_images)
        {
            hand_excluder.lock();
            buffer_excluder.lock();

            new_frame_available_mt = false;
            request_finished_mt = false;

            LeapRequestImages(connection, &descrip, &token);

            buffer_excluder.unlock();
            hand_excluder.unlock();

            /*hand_excluder.lock();
            new_frame_available_mt = false;
            hand_excluder.unlock();

            buffer_excluder.lock();
            request_finished_mt = false;
            buffer_excluder.unlock();*/
        }

        //printf("pr %i %i\n", is_finished, new_frame_available);

        int64_t now = LeapGetNow();

        //LeapUpdateRebase(*rebase, clk.getElapsedTime().asMicroseconds(), now);
        //LeapRebaseClock(*rebase, clk.getElapsedTime().asMicroseconds(), &now);

        ///-1000*10 is perfectly smooth, but higher latency obvs
        eLeapRS res = LeapInterpolateFrame(connection, now - 1000*0, pEvent, 99999);

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

                    p.bone_num = bb;
                    p.finger_num = jj;
                    p.hand_id = first.id;

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

    std::vector<positional> get_smoothed_positionals(int n = 8)
    {
        std::vector<positional> ret;

        if(hand_history.size() == 0)
            return get_positionals();

        int num = 0;

        //for(std::map<uint32_t, LEAP_HAND>& hand : hand_history)
        for(int kk=hand_history.size()-1; kk >= 0; kk--)
        {
            std::map<uint32_t, LEAP_HAND>& hand = hand_history[kk];

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

            if(num >= n)
                break;
        }

        for(auto& i : ret)
        {
            i.pos = i.pos / (float)num;
        }

        return ret;
    }

    LEAP_HAND get_smoothed_hand_history(uint32_t id, int offset_from_size, int either_side = 2)
    {
        if(offset_from_size <= 0 || offset_from_size > hand_history.size())
            return LEAP_HAND();

        LEAP_HAND ret = hand_history[hand_history.size() - offset_from_size][id];

        int num = 1;

        for(int i=-either_side; i<=either_side; i++)
        {
            if(i == 0)
                continue;

            int offset = hand_history.size() - (i + offset_from_size);

            if(offset < 0 || offset >= hand_history.size())
                continue;

            LEAP_HAND& h = hand_history[offset][id];

            for(int ff = 0; ff < 5; ff++)
            {
                for(int bb = 0; bb < 4; bb++)
                {
                    LEAP_VECTOR vn = h.digits[ff].bones[bb].next_joint;
                    LEAP_VECTOR vp = h.digits[ff].bones[bb].prev_joint;

                    LEAP_VECTOR von = ret.digits[ff].bones[bb].next_joint;
                    LEAP_VECTOR vop = ret.digits[ff].bones[bb].prev_joint;

                    vec3f vns = {vn.v[0], vn.v[1], vn.v[2]};
                    vec3f vps = {vp.v[0], vp.v[1], vp.v[2]};

                    vec3f vons = {von.v[0], von.v[1], von.v[2]};
                    vec3f vops = {vop.v[0], vop.v[1], vop.v[2]};

                    vns += vons;
                    vps += vops;

                    ret.digits[ff].bones[bb].next_joint = {vns.v[0], vns.v[1], vns.v[2]};
                    ret.digits[ff].bones[bb].prev_joint = {vps.v[0], vps.v[1], vps.v[2]};
                }
            }

            num++;
        }

        for(int ff = 0; ff < 5; ff++)
        {
            for(int bb = 0; bb < 4; bb++)
            {
                LEAP_VECTOR von = ret.digits[ff].bones[bb].next_joint;
                LEAP_VECTOR vop = ret.digits[ff].bones[bb].prev_joint;

                vec3f vons = {von.v[0], von.v[1], von.v[2]};
                vec3f vops = {vop.v[0], vop.v[1], vop.v[2]};

                vons = vons / (float)num;
                vops = vops / (float)num;

                ret.digits[ff].bones[bb].next_joint = {vons.v[0], vons.v[1], vons.v[2]};
                ret.digits[ff].bones[bb].prev_joint = {vops.v[0], vops.v[1], vops.v[2]};
            }
        }

        return ret;
    }

    std::map<uint32_t, LEAP_HAND> get_smoothed_all_hands_history(int offset_from_size, int either_side = 2)
    {
        std::map<uint32_t, LEAP_HAND> ret;

        for(auto& i : hand_map)
        {
            ret[i.first] = get_smoothed_hand_history(i.first, offset_from_size, either_side);
        }

        return ret;
    }

    std::vector<pinch> get_pinches()
    {
        std::vector<pinch> ret;

        if(hand_history.size() == 0)
            return ret;

        for(auto& i : hand_map)
        //for(auto& i : hand_history.back())
        {
            LEAP_HAND& h = i.second;

            vec3f thumb_pos = xyz_to_vec(h.thumb.stabilized_tip_position);
            vec3f index_pos = xyz_to_vec(h.index.stabilized_tip_position);

            pinch p;

            p.pos = (thumb_pos*1 + index_pos) / 2.f;
            p.pos.v[2] = -p.pos.v[2];

            p.hand_pos = xyz_to_vec(h.palm.position);
            p.hand_pos.v[2] = -p.hand_pos.v[2];

            p.pinch_strength = h.pinch_strength;
            p.hand_id = h.id;

            float derived_distance = (thumb_pos - index_pos).length();

            ///8cm
            p.derived_pinch_strength = std::max(1.f - (derived_distance / 75.f), 0.f);
            //printf("Pdist %f %f\n", p.pinch_strength, (thumb_pos - index_pos).length());


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


#endif // LEAP_MOTION_H_INCLUDED
