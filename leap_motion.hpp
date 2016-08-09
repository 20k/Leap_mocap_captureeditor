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
    int max_history = 2;

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

        eLeapRS res = LeapInterpolateFrame(connection, now, pEvent, 99999);

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


#endif // LEAP_MOTION_H_INCLUDED
