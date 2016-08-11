#ifndef HAND_FIRER_HPP_INCLUDED
#define HAND_FIRER_HPP_INCLUDED

struct fire_state
{
    bool fired = false;
    bool fire_this_frame = false;
    uint32_t id = -1;
    vec3f pos = {0,0,0};
    vec3f dir = {0,0,0};

    sf::Clock suppression_clock;
    float suppression_time_ms = 400.f;
};

struct pvec
{
    vec3f pos;
    vec3f dir;
};

struct hand_firer
{
    leap_motion* motion;

    ///leap motion frames
    int frame_history = 4;

    bool in_fire_state = false;

    std::map<uint32_t, fire_state> hand_fire_state;

    void init(leap_motion* _motion)
    {
        motion = _motion;
    }

    quaternion hand_rotation(LEAP_HAND& h)
    {
        LEAP_QUATERNION q = h.digits[2].bones[0].rotation;

        return {{q.x, q.y, q.z, q.w}};
    }

    void tick()
    {
        if(motion->hand_history.size() <= frame_history)
            return;

        if(frame_history <= 0)
        {
            lg::log("Frame history cannot be <= 0");
            return;
        }

        ///.size()-1 == latest, lower is earlier

        //std::map<uint32_t, LEAP_HAND>& latest = motion->hand_history[motion->hand_history.size()-1];
        //std::map<uint32_t, LEAP_HAND>& earliest = motion->hand_history[motion->hand_history.size()-frame_history];

        auto latest = motion->get_smoothed_all_hands_history(1, 1);
        auto earliest = motion->get_smoothed_all_hands_history(frame_history, 1);

        std::vector<uint32_t> ids;

        for(auto& i : latest)
            ids.push_back(i.first);

        for(auto& id : ids)
        {
            LEAP_HAND& h2 = latest[id];
            LEAP_HAND& h1 = earliest[id];

            quaternion q2 = hand_rotation(h2);
            quaternion q1 = hand_rotation(h1);

            mat3f m2 = q2.get_rotation_matrix();
            mat3f m1 = q1.get_rotation_matrix();

            vec3f v2 = m2 * (vec3f){0, 1, 0};
            vec3f v1 = m1 * (vec3f){0, 1, 0};

            float angle = angle_between_vectors(v1, v2);

            fire_state& hand_state = hand_fire_state[id];

            hand_state.id = id;

            LEAP_VECTOR l1 = h1.index.bones[3].prev_joint;
            LEAP_VECTOR l2 = h1.index.bones[3].next_joint;
            LEAP_VECTOR ls = h1.index.bones[1].prev_joint;
            LEAP_VECTOR le = h1.index.bones[1].next_joint;

            vec3f d1 = {l1.x, l1.y, -l1.z};
            vec3f d2 = {l2.x, l2.y, -l2.z};
            vec3f ds = {ls.x, ls.y, -ls.z};
            vec3f de = {le.x, le.y, -le.z};

            vec3f offset = (de - ds).norm() * 100;

            vec3f avg_pos = (ds + de)/2.f;

            quaternion rquat = convert_from_leap_quaternion(h1.index.bones[1].rotation);

            mat3f fquat = rquat.get_rotation_matrix();

            vec3f dir = fquat * (vec3f){0, 0, 1};

            //vec3f pos = (d1 + d2)/2.f;

            bool primary_extended = h1.index.is_extended;

            bool others_extended = false;

            for(int i=2; i<5; i++)
            {
                others_extended |= h1.digits[i].is_extended;
            }

            if(angle > M_PI/8 && primary_extended && !others_extended)
            {
                hand_state.pos = d2 + offset;
                hand_state.dir = dir.norm();
                //hand_state.dir = (de - ds).norm();
                //hand_state.dir = (d1 - ds).norm();

                if(!hand_state.fired)
                {
                    hand_state.fire_this_frame = true;
                }

                hand_state.fired = true;
            }
            else
            {
                hand_state.fire_this_frame = false;
                hand_state.fired = false;
            }
        }
    }

    bool hand_should_fire(uint32_t id)
    {
        return hand_fire_state[id].fire_this_frame;
    }

    std::vector<pvec> get_fire_positions()
    {
        std::vector<pvec> ret;

        for(auto& ifs : hand_fire_state)
        {
            fire_state& fs = ifs.second;

            uint32_t id = fs.id;

            vec3f pos = fs.pos;
            vec3f dir = fs.dir;

            if(fs.fire_this_frame && fs.suppression_clock.getElapsedTime().asMilliseconds() >= fs.suppression_time_ms)
            {
                fs.suppression_clock.restart();

                ret.push_back({pos, dir});
            }

            fs.fire_this_frame = false;
        }

        return ret;
    }
};

#endif // HAND_FIRER_HPP_INCLUDED
