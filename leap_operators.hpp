#ifndef LEAP_OPERATORS_HPP_INCLUDED
#define LEAP_OPERATORS_HPP_INCLUDED

using bone_fptr = JBONE (*)(const JBONE&, const JBONE&);

using bone_stdfunc = std::function<JBONE(const JBONE&, const JBONE&)>;

using bone_stdsingle = std::function<JBONE(const JBONE&, float)>;

///ok. we're also going to need divide and then mult
inline
JBONE bone_sub(const JBONE& one, const JBONE& two)
{
    JBONE ret = one;

    ret.prev_joint = one.prev_joint - two.prev_joint;
    ret.next_joint = one.next_joint - two.next_joint;

    ret.width = one.width - two.width;

    ///one * ret.rotation = two.rotation
    ret.rotation = one.rotation.get_difference(two.rotation);

    return ret;
}

inline
JBONE bone_add(const JBONE& one, const JBONE& two)
{
    JBONE ret = one;

    ret.prev_joint = one.prev_joint + two.prev_joint;
    ret.next_joint = one.next_joint + two.next_joint;

    ret.width = one.width + two.width;

    ///one * ret.rotation = two.rotation
    ret.rotation = one.rotation * two.rotation;

    return ret;
}

/// < 1 or uuh.. oh well, rotations would be interesting
inline
JBONE bone_div(const JBONE& one, float two)
{
    JBONE ret = one;

    ret.prev_joint = one.prev_joint / two;
    ret.next_joint = one.next_joint / two;

    ret.width = one.width / two;

    ///when two is large, we get 0
    ///when two -> 1 we get 1
    float interpolate_frac = 1.f / two;

    ///when 0 we get identity
    ///when 1 we get one.rotation
    ret.rotation = quat::slerp(quat().identity(), one.rotation, interpolate_frac);

    return ret;
}

///change this to take a bone_op so we don't have to damn reduplicate all these functions
inline
JDIGIT digit_op(const JDIGIT& one, const JDIGIT& two, bone_stdfunc bone_func)
{
    JDIGIT ret = one;

    for(int i=0; i<4; i++)
    {
        ret.bones[i] = bone_func(one.bones[i], two.bones[i]);
    }

    return ret;
}

inline
JDIGIT digit_sop(const JDIGIT& one, float two, bone_stdsingle bone_func)
{
    JDIGIT ret = one;

    for(int i=0; i<4; i++)
    {
        ret.bones[i] = bone_func(one.bones[i], two);
    }

    return ret;
}

inline
JHAND hand_op(const JHAND& one, const JHAND& two, bone_stdfunc bone_func)
{
    JHAND ret = one;

    for(int i=0; i<5; i++)
    {
        ret.digits[i] = digit_op(one.digits[i], two.digits[i], bone_func);
    }

    return ret;
}

inline
JHAND hand_sop(const JHAND& one, float two, bone_stdsingle bone_func)
{
    JHAND ret = one;

    for(int i=0; i<5; i++)
    {
        ret.digits[i] = digit_sop(one.digits[i], two, bone_func);
    }

    return ret;
}

///lets be real we'll never have 3 hands, and l/r is a very useful distinction
struct LRHAND
{
    JHAND hands[2] = {};
    int32_t hand_ids[2] = {-1, -1};
};

inline
LRHAND get_lrhand(const leap_motion_capture_frame& frame)
{
    LRHAND ret;

    for(auto& i : frame.frame_data)
    {
        ret.hands[i.second.type] = i.second;
        ret.hand_ids[i.second.type] = i.first;
    }

    return ret;
}

///more complex, we're essentially merging the frames
///ok. We need to properly merge frames here, this is the issue i think
inline
leap_motion_capture_frame frame_op(const leap_motion_capture_frame& one, const leap_motion_capture_frame& two, bone_stdfunc bone_func)
{
    LRHAND lrone = get_lrhand(one);
    LRHAND lrtwo = get_lrhand(two);

    leap_motion_capture_frame ret = one;

    std::map<uint32_t, JHAND> ret_frame;

    for(int i=0; i<2; i++)
    {
        if(lrone.hand_ids[i] == -1 && lrtwo.hand_ids[i] == -1)
            continue;

        uint32_t forced_id = lrone.hand_ids[i];

        ///if one doesn't exist, we'll just get all 0s which is acceptable, although potentially a hazard
        JHAND& h1 = lrone.hands[i];
        JHAND& h2 = lrtwo.hands[i];

        ret_frame[forced_id] = hand_op(h1, h2, bone_func);
    }

    ret.frame_data = ret_frame;

    return ret;
}

inline
leap_motion_capture_frame frame_sop(const leap_motion_capture_frame& one, float two, bone_stdsingle bone_func)
{
    leap_motion_capture_frame ret = one;

    for(auto& i : ret.frame_data)
    {
        i.second = hand_sop(i.second, two, bone_func);
    }

    return ret;
}

#endif // LEAP_OPERATORS_HPP_INCLUDED
