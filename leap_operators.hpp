#ifndef LEAP_OPERATORS_HPP_INCLUDED
#define LEAP_OPERATORS_HPP_INCLUDED

inline
JBONE bone_sub(JBONE& one, JBONE& two)
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

inline
JDIGIT digit_sub(JDIGIT& one, JDIGIT& two)
{
    JDIGIT ret = one;

    for(int i=0; i<4; i++)
    {
        ret.bones[i] = bone_sub(one.bones[i], two.bones[i]);
    }

    return ret;
}

inline
JHAND hand_sub(JHAND& one, JHAND& two)
{
    JHAND ret = one;

    for(int i=0; i<5; i++)
    {
        ret.digits[i] = digit_sub(one.digits[i], two.digits[i]);
    }

    return ret;
}

///more complex, we're essentially merging the frames
inline
leap_motion_capture_frame frame_sub(leap_motion_capture_frame& one, leap_motion_capture_frame& two)
{

}

#endif // LEAP_OPERATORS_HPP_INCLUDED
