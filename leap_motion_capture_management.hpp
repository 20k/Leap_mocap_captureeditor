#ifndef LEAP_MOTION_CAPTURE_MANAGEMENT_HPP_INCLUDED
#define LEAP_MOTION_CAPTURE_MANAGEMENT_HPP_INCLUDED

#include <vec/vec.hpp>
#include <string>
#include <vector>
#include <map>
#include <SFML/System.hpp>
#include <leap/leapC.h>
#include "../openclrenderer/objects_container.hpp"

struct objects_container;
struct object_context;


struct JBONE
{
    vec3f prev_joint = {0,0,0};
    vec3f next_joint = {0,0,0};

    float width = 0;

    quat rotation;

    vec3f get_pos()
    {
        return (prev_joint + next_joint)/2.f;
    }
};

struct JDIGIT
{
    int32_t finger_id = -1;

    JBONE bones[4] = {};
};

struct JHAND
{
    uint32_t id = -1;

    ///0 = left, 1 = right
    int type = 0;

    JDIGIT digits[5] = {};
};

inline
JHAND leap_hand_to_engine(const LEAP_HAND& hand)
{
    uint32_t hid = hand.id;

    JHAND jhand;
    jhand.id = hand.id;
    jhand.type = hand.type;

    for(int digit_id = 0; digit_id < 5; digit_id++)
    {
        LEAP_DIGIT dig1 = hand.digits[digit_id];

        jhand.digits[digit_id].finger_id = dig1.finger_id;

        for(int bone_id = 0; bone_id < 4; bone_id++)
        {
            LEAP_BONE b1 = dig1.bones[bone_id];

            JBONE& jb = jhand.digits[digit_id].bones[bone_id];

            jb.next_joint = xyz_to_vec(b1.next_joint);
            jb.prev_joint = xyz_to_vec(b1.prev_joint);

            jb.next_joint.v[2] = -jb.next_joint.v[2];
            jb.prev_joint.v[2] = -jb.prev_joint.v[2];

            jb.width = b1.width;
            jb.rotation = convert_from_leap_quaternion(b1.rotation);
        }
    }

    return jhand;
}

struct leap_motion_capture_frame
{
    float time_s = 0.f;

    std::map<uint32_t, JHAND> frame_data;
};

#include "leap_operators.hpp"

struct leap_motion_capture_data
{
    std::vector<leap_motion_capture_frame> data;

    sf::Clock clk;

    void start_capture()
    {
        clk.restart();
    }

    void save_frame(std::map<uint32_t, LEAP_HAND>& dat)
    {
        leap_motion_capture_frame frame;

        //frame.frame_data = leap_hand_to_engine(dat);
        for(auto& i : dat)
        {
            frame.frame_data[i.first] = leap_hand_to_engine(dat[i.first]);
        }

        frame.time_s = (clk.getElapsedTime().asMicroseconds() / 1000.f) / 1000.f;

        data.push_back(frame);
    }
};

///include option to pad beginning and end frames for dynamic blending
struct leap_motion_replay
{
    char name[20] = {0};
    leap_motion_capture_data mocap;

    ///if is exiting, we distribute whole animation error across this replay
    //bool is_exciting = false;
    bool exciting_start = false;
    bool exciting_end = false;

    //int smooth_level = 0;

    sf::Clock clk;
    bool going = false;
    int last_frame = 0; ///INTERNAL frame
    bool can_terminate = true;

    void set_replay_data(const leap_motion_capture_data& dat);
    void start_playblack();


    float get_time_s();

    int get_frames_remaining() const;
    leap_motion_capture_frame get_current_frame();
    leap_motion_capture_frame get_next_frame();

    void trim_all_frames_before_current();

    bool should_advance_frame();

    ///0 = closest to current frame, 1 = closest to next frame
    float get_frame_frac();
    void conditionally_advance_frame();
    bool finished();


    JBONE interpolate_bones(JBONE b1, JBONE b2, float a);

    leap_motion_capture_frame get_interpolated_frame();
    leap_motion_capture_frame position_containers(std::vector<objects_container*>& containers);

    leap_motion_replay smooth();
};

struct current_replay
{
    std::vector<objects_container*> containers;
    leap_motion_replay replay;
    leap_motion_capture_frame last_interpolated;
};

///have a get_free_containers method
///keep track of in use containers
///have a start_replay_with_internal_containers and etc, then this class can manage their lifetime
struct leap_motion_capture_manager
{
    std::vector<leap_motion_replay> replays;

    ///in progress capture
    leap_motion_capture_data in_progress;

    int gid = 0;

    bool fix_clipping = true;

    std::map<int, current_replay> currently_replaying_map;

    //std::vector<current_replay> currently_replaying;

    bool going = false;
    bool snap = false;

    void add_capture(const leap_motion_capture_data& capture, const std::string& name = "", bool active_start = false, bool active_end = false);
    void start_capture();

    void set_capture_data(std::map<uint32_t, LEAP_HAND>& hands);
    void finish_capture();

    void snap_one_frame_capture();

    int start_external_replay(const leap_motion_replay& replay, std::vector<objects_container*>& containers);
    int start_replay(int id, std::vector<objects_container*>& containers);
    void tick_replays();

    object_context* ctx;

    std::vector<objects_container*> ctrs;

    float scale = 10.f;

    void init_manual_containers(object_context& context, int num_hands);
    void init_manual_containers_with_col(object_context& context, int num_hands, vec3f col);
    void destroy_manual_containers();
    void hide_manual_containers();
    void tick_ui();

    void save(const std::string& prefix);

    void load(const std::string& prefix);
};

///don't move, its vision is based on motion
///Ok. Works for left hand. Implement right hand as well. Where do we put it on the sword?? We may have to redesign sword
///Ok. I have an angle for the right hand that I'm ok with. Next up is figuring out wtf we're going to do with the hand placement eh
///hand placement
///works fine now
inline
void attach_replays_to_fighter_sword(leap_motion_capture_manager& capture_manager, objects_container* sword_ctr, float dyn_scale = 0.6f, float base_scale = 10.f)
{
    vec3f sword_up = {0, 1, 0};

    vec3f sword_pos = xyz_to_vec(sword_ctr->pos);
    quat sword_rot = sword_ctr->rot_quat;

    //printf("Sword pos %f %f %f   %f %f %f %f\n", EXPAND_3(sword_pos), sword_rot.q.x(), sword_rot.q.y(), sword_rot.q.z(), sword_rot.q.w());

    ///ie in direction of point
    vec3f sword_current_up = sword_rot.get_rotation_matrix() * sword_up;

    //for(current_replay& replay : capture_manager.currently_replaying)
    for(auto& rmap : capture_manager.currently_replaying_map)
    {
        current_replay& replay = rmap.second;

        leap_motion_capture_frame frame = replay.last_interpolated;

        int cid = 0;

        for(auto& hand_data : frame.frame_data)
        {
            JHAND& hand = hand_data.second;

            ///nans seem to originate in underlying data, add to investigation list
            vec3f hand_pos = hand.digits[2].bones[0].get_pos();
            quat hand_rot = hand.digits[2].bones[0].rotation;

            if(std::isnan(hand_pos.x()) || std::isnan(hand_pos.y()) || std::isnan(hand_pos.z()))
                continue;

            //printf("hand pos %f %f %f\n", EXPAND_3(hand_pos));

            for(int kk=0; kk < replay.containers.size() && kk < (5 * 4); kk++)
            {
                vec3f ctr_pos = xyz_to_vec(replay.containers[cid]->pos);
                quat ctr_rot = replay.containers[cid]->rot_quat;

                ctr_pos = hand_rot.get_rotation_matrix().transp() * (ctr_pos - hand_pos) + hand_pos;

                mat3f trot = hand_rot.get_rotation_matrix().transp() * ctr_rot.get_rotation_matrix();
                ctr_rot.load_from_matrix(trot);

                vec3f offset = {0, 25, -10};

                ctr_pos = ctr_pos + offset;

                ///FOR RHAND
                quat AA_0_R;
                AA_0_R.load_from_axis_angle({0, 0, 1, -M_PI/2});

                ///hand offset from top down, ie the skewiffyness of the hand vertically
                quat AA_1_R;
                AA_1_R.load_from_axis_angle({0, 1, 0, M_PI/2 - M_PI/5});

                ///hand offset in the direction of the sword, ie hand tilt
                quat AA_2_R;
                AA_2_R.load_from_axis_angle({0, 0, 1, M_PI/8});

                quat AA_0_L;
                AA_0_L.load_from_axis_angle({0, 0, 1, -M_PI/2});

                quat AA_1_L;
                AA_1_L.load_from_axis_angle({0, 1, 0, -M_PI + M_PI/8});

                quat combo_quat;

                if(hand.type == 0)
                    combo_quat = AA_0_L * AA_1_L;
                if(hand.type == 1)
                    combo_quat = AA_2_R * AA_1_R * AA_0_R;

                vec3f displace_dir = {0,0,0};

                if(hand.type == 1)
                {
                    displace_dir = -sword_current_up * 25;
                }

                if(hand.type == 0)
                {
                    displace_dir = sword_current_up * 10;
                }


                vec3f new_coordinate_system = combo_quat.get_rotation_matrix() * ((ctr_pos - hand_pos) * dyn_scale) + hand_pos;
                quat new_coordinate_quat = combo_quat * ctr_rot;

                ctr_pos = new_coordinate_system;
                ctr_rot = new_coordinate_quat;

                vec3f flat_hand_relative = ctr_pos - hand_pos;

                vec3f sword_coordinates_pos = sword_rot.get_rotation_matrix() * flat_hand_relative + sword_pos;

                sword_coordinates_pos = sword_coordinates_pos + displace_dir;

                quat sword_coordinates_rot = sword_rot * ctr_rot;

                if(any_nan(sword_coordinates_pos))
                    continue;

                if(any_nan(sword_coordinates_rot.q))
                    continue;

                replay.containers[cid]->set_pos({sword_coordinates_pos.x(), sword_coordinates_pos.y(), sword_coordinates_pos.z()});
                replay.containers[cid]->set_rot_quat(sword_coordinates_rot);
                replay.containers[cid]->set_dynamic_scale(dyn_scale * base_scale);

                /*if(cid == 8)
                {
                    printf("BONE CENTER SET %f %f %f\n", EXPAND_3(sword_coordinates_pos));
                }*/

                cid++;
            }
        }
    }
}

#if 0
///scale???
struct hand_cosmetics
{
    objects_container* obj;

    hand_cosmetics(object_context& ctx);

    ~hand_cosmetics()
    {
        obj->set_active(false);
        obj->parent->destroy(obj);
    }

    void position(std::vector<objects_container*>& ctr);
};
#endif

///not sure i need to do this yet
/*inline
JDIGIT update_bone_system(JDIGIT digit, int bone_id, vec3f offset)
{
    if(bone_id == 0)
        return digit;

    for(int i=bone_id+1; i<4; i++)
    {
        digit.bones[i].next_joint += offset;
        digit.bones[i].prev_joint += offset;
    }

    return digit;
}*/

/*inline
void fix_replays_clipping(leap_motion_capture_manager& capture_manager, objects_container* clipping_base)
{

}*/

///this method of preventing clipping is bad, it produces disformed hand shapes
///We need to like, actually fix the hand positioning. Maybe save the last valid hand position. Could potentially preprocess the animation and remove all invalid hand states,
///then we'd interpolate between valid ones (which still may not necessarily be)
///100% valid, do it on a per finger level?
///IK would be last resort
///OK BACKUP
///If we fix the quaternions this might work ok, they're the main source of it looking bad
///works fine now
inline
void fix_replays_clipping(leap_motion_capture_manager& capture_manager, objects_container* clipping_base)
{
    vec3f up = {0, 1, 0};

    vec3f reference_pos = xyz_to_vec(clipping_base->pos);
    quat reference_rot = clipping_base->rot_quat;

    vec3f reference_forward = reference_rot.get_rotation_matrix() * up;

    float clamp_distance = 15.f;

    for(auto& rmap : capture_manager.currently_replaying_map)
    {
        current_replay& replay = rmap.second;

        leap_motion_capture_frame frame = replay.last_interpolated;

        int cid = 0;

        for(auto& hand_data : frame.frame_data)
        {
            JHAND& hand = hand_data.second;

            //vec3f hand_pos = hand.digits[2].bones[0].get_pos();
            //quat hand_rot = hand.digits[2].bones[0].rotation;

            for(int digit_id = 0; digit_id < 5; digit_id++)
            {
                JDIGIT digit = hand.digits[digit_id];

                for(int bone_id = 0; bone_id < 4; bone_id++)
                {
                    ///this is a little hacky and poorly defined
                    ///should really have a get function or something
                    vec3f digit_pos = xyz_to_vec(replay.containers[cid]->pos);

                    if(any_nan(digit_pos))
                        continue;

                    //vec<N, T> point2line_shortest(const vec<N, T>& lp, const vec<N, T>& ldir, const vec<N, T>& p)

                    vec3f point_to_line = point2line_shortest(reference_pos, reference_forward.norm(), digit_pos);

                    float distance_to_line = point_to_line.length();


                    vec3f displace_dir = -point_to_line.norm();

                    float extra_dist = std::max(clamp_distance - distance_to_line, 0.f);

                    vec3f displace_vec = displace_dir * extra_dist;

                    vec3f new_point = digit_pos + displace_vec;

                    replay.containers[cid]->set_pos({new_point.x(), new_point.y(), new_point.z()});

                    if(bone_id > 0)
                    {
                        ///ok. We need to find the axis of rotation for the previous bone
                        ///and then rotate it by the amount the current bone has moved essentially

                       objects_container* behind_bone = replay.containers[cid - 1];

                       vec3f behind_pos = xyz_to_vec(behind_bone->pos);

                       vec3f behind_to_old = digit_pos - behind_pos;
                       vec3f behind_to_new = new_point - behind_pos;

                       float cos_angle = dot(behind_to_old.norm(), behind_to_new.norm());

                       float angle = acos(clamp(cos_angle, -1.f, 1.f));

                       vec3f axis_unrot = {1, 0, 0};

                       vec3f axis = behind_bone->rot_quat.get_rotation_matrix() * axis_unrot;

                       quat rot_offset;
                       rot_offset.load_from_axis_angle({axis.x(), axis.y(), axis.z(), -angle});

                       mat3f combor = rot_offset.get_rotation_matrix() * behind_bone->rot_quat.get_rotation_matrix();

                       quat result;
                       result.load_from_matrix(combor);

                       behind_bone->set_rot_quat(result);

                       quat base_quat = replay.containers[cid]->rot_quat;
                       mat3f base_r = rot_offset.get_rotation_matrix() * base_quat.get_rotation_matrix();

                       quat ret;
                       ret.load_from_matrix(base_r);

                       replay.containers[cid]->set_rot_quat(ret);
                    }

                    //digit = update_bone_system(digit, bone_id, displace_vec);

                    //for(int kk=bone_id+1; kk<4; kk++)
                    /*for(int kk = bone_id-1; kk > 0; kk--)
                    {
                        vec3f pos = xyz_to_vec(replay.containers[cid + kk]->pos);

                        pos = pos + displace_vec;

                        replay.containers[cid + kk]->set_pos({pos.x(), pos.y(), pos.z()});
                    }*/

                    cid++;
                }
            }
        }
    }
}

#endif // LEAP_MOTION_CAPTURE_MANAGEMENT_HPP_INCLUDED
