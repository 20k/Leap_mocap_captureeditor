#ifndef LEAP_OPERATORS_HPP_INCLUDED
#define LEAP_OPERATORS_HPP_INCLUDED

///ok. we're also going to need divide and then mult
inline
JBONE bone_sub(const JBONE& one, const JBONE& two)
{
    JBONE ret = one;

    ret.prev_joint = one.prev_joint - two.prev_joint;
    ret.next_joint = one.next_joint - two.next_joint;

    ret.width = one.width - two.width;
    //ret.next_joint = one.next_joint - two.next_joint;

    ///one * ret.rotation = two.rotation
    ret.rotation = one.rotation.get_difference(two.rotation);

    return ret;
}

///change this to take a bone_op so we don't have to damn reduplicate all these functions
inline
JDIGIT digit_sub(const JDIGIT& one, const JDIGIT& two)
{
    JDIGIT ret = one;

    for(int i=0; i<4; i++)
    {
        ret.bones[i] = bone_sub(one.bones[i], two.bones[i]);
    }

    return ret;
}

inline
JHAND hand_sub(const JHAND& one, const JHAND& two)
{
    JHAND ret = one;

    for(int i=0; i<5; i++)
    {
        ret.digits[i] = digit_sub(one.digits[i], two.digits[i]);
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
inline
leap_motion_capture_frame frame_sub(const leap_motion_capture_frame& one, const leap_motion_capture_frame& two)
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

        ret_frame[forced_id] = hand_sub(h1, h2);
    }

    ret.frame_data = ret_frame;

    return ret;
}

#endif // LEAP_OPERATORS_HPP_INCLUDED
