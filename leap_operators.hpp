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

inline
bool valid_hand_id(int id, const std::vector<int32_t>& hand_ids)
{
    for(auto& i : hand_ids)
    {
        if(i == id)
            return true;
    }

    return false;
}

inline
int side_hand_to_id(int side, int32_t old_id, const std::vector<int32_t>& valid_ids, const std::vector<int>& sides)
{
    for(int i=0; i<sides.size(); i++)
    {
        if(sides[i] == side)
        {
            return valid_ids[i];
        }
    }

    ///could be that we have a scene with both a left and right hand in, which is stupid
    ///ie anim 1 has left hand, anim 2 only right
    ///or if anim 1 has left hand, and anim 2 has left AND right
    ///this is actually a legit use case, for testing at least
    //lg::log("SEMI ERROR ANIM STATE HAND CONFLICT ARE YOU BEING AN IDIOT");

    //assert(false);

    return old_id;
}

inline
leap_motion_capture_frame patch_frame_hand_ids(const leap_motion_capture_frame& start, const leap_motion_capture_frame& next)
{
    std::vector<int32_t> hand_ids;
    std::vector<int> hand_sides; ///0 = left, 1 = right

    //for(leap_motion_capture_frame& frame : ret.mocap.data)

    const leap_motion_capture_frame& check_frame = start;

    {
        for(auto& i : check_frame.frame_data)
        {
            hand_ids.push_back(i.first);
            hand_sides.push_back(i.second.type);
        }
    }

    if(hand_ids.size() > 2)
        hand_ids.resize(2);

    if(hand_sides.size() > 2)
        hand_sides.resize(2);

    //leap_motion_replay next_copy = next;

    ///START PATCHING HAND IDS
    //for(leap_motion_capture_frame& frame : next_copy.mocap.data)

    leap_motion_capture_frame frame = next;

    {
        std::vector<uint32_t> ids_to_erase;
        std::vector<JHAND> replacement_hands;

        ///if we want more hands, we'll have to only patch if the hand id isn't present
        ///ie we might have two right hands and one left, with one right hand appearing, but the second right hand will currently
        ///be overwritten to the first. I have no idea if this is a really problematic issue or not, but I guess its worth noting
        for(auto& i : frame.frame_data)
        {
            JHAND current_hand = i.second;

            int32_t id = i.first;

            if(!valid_hand_id(id, hand_ids))
            {
                int side = current_hand.type;

                int new_id = side_hand_to_id(side, id, hand_ids, hand_sides);

                ids_to_erase.push_back(id);
                current_hand.id = new_id;

                replacement_hands.push_back(current_hand);
            }
        }

        for(auto& era : ids_to_erase)
        {
            frame.frame_data.erase(era);
        }

        for(auto& i : replacement_hands)
        {
            frame.frame_data[i.id] = i;
        }
    }

    return frame;
}

///more complex, we're essentially merging the frames
///ok. We need to properly merge frames here, this is the issue i think
///nope. Map consistently sorts them, I don't know why I believed this could possibly be an issue
inline
leap_motion_capture_frame frame_op(const leap_motion_capture_frame& one, const leap_motion_capture_frame& two, bone_stdfunc bone_func)
{
    //leap_motion_capture_frame patched_two = patch_frame_hand_ids(one, two);
    leap_motion_capture_frame patched_two = two;

    leap_motion_capture_frame ret = one;

    for(auto& i : one.frame_data)
    {
        const JHAND h1 = i.second;
        const JHAND h2 = patched_two.frame_data[i.first];

        ret.frame_data[i.first] = hand_op(h1, h2, bone_func);
    }

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
