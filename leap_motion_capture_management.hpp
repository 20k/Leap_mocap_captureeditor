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
    vec3f prev_joint;
    vec3f next_joint;

    float width;

    quat rotation;

    vec3f get_pos()
    {
        return (prev_joint + next_joint)/2.f;
    }
};

struct JDIGIT
{
    int32_t finger_id;

    JBONE bones[4];
};

struct JHAND
{
    uint32_t id;

    ///0 = left, 1 = right
    int type = 0;

    JDIGIT digits[5];
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
    leap_motion_capture_data mocap;

    sf::Clock clk;
    bool going = false;
    int last_frame = 0; ///INTERNAL frame

    void set_replay_data(const leap_motion_capture_data& dat);
    void start_playblack();

    float get_time_s();

    leap_motion_capture_frame get_current_frame();
    leap_motion_capture_frame get_next_frame();

    bool should_advance_frame();

    ///0 = closest to current frame, 1 = closest to next frame
    float get_frame_frac();
    void conditionally_advance_frame();
    bool finished();

    JBONE interpolate_bones(JBONE b1, JBONE b2, float a);

    leap_motion_capture_frame get_interpolated_frame();
    leap_motion_capture_frame position_containers(std::vector<objects_container*>& containers);
};

struct current_replay
{
    std::vector<objects_container*> containers;
    leap_motion_replay replay;
    leap_motion_capture_frame last_interpolated;
};

struct leap_motion_capture_manager
{
    std::vector<leap_motion_replay> replays;

    ///in progress capture
    leap_motion_capture_data in_progress;

    int gid = 0;

    std::map<int, current_replay> currently_replaying_map;

    //std::vector<current_replay> currently_replaying;

    bool going = false;
    bool snap = false;

    void add_capture(const leap_motion_capture_data& capture);
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

    void init_manual_containers(object_context& context);
    void tick_ui();

    void save(const std::string& prefix);

    void load(const std::string& prefix);
};

///don't move, its vision is based on motion
inline
void attach_replays_to_fighter_sword(leap_motion_capture_manager& capture_manager, objects_container* sword_ctr, float dyn_scale = 0.6f, float base_scale = 10.f)
{
    //for(current_replay& replay : capture_manager.currently_replaying)
    for(auto& rmap : capture_manager.currently_replaying_map)
    {
        current_replay& replay = rmap.second;

        vec3f sword_pos = xyz_to_vec(sword_ctr->pos);
        quat sword_rot = sword_ctr->rot_quat;

        leap_motion_capture_frame frame = replay.last_interpolated;

        int cid = 0;

        for(auto& hand_data : frame.frame_data)
        {
            JHAND& hand = hand_data.second;

            vec3f hand_pos = hand.digits[2].bones[0].get_pos();
            quat hand_rot = hand.digits[2].bones[0].rotation;

            for(int kk=0; kk < replay.containers.size() && kk < (frame.frame_data.size() * 5 * 4); kk++)
            {
                vec3f ctr_pos = xyz_to_vec(replay.containers[cid]->pos);
                quat ctr_rot = replay.containers[cid]->rot_quat;

                ctr_pos = hand_rot.get_rotation_matrix().transp() * (ctr_pos - hand_pos) + hand_pos;

                mat3f trot = hand_rot.get_rotation_matrix().transp() * ctr_rot.get_rotation_matrix();
                ctr_rot.load_from_matrix(trot);

                vec3f offset = {0, 25, -10};

                ctr_pos = ctr_pos + offset;

                quat AA_0;
                AA_0.load_from_axis_angle({1, 0, 0, M_PI/2});

                quat AA_1;
                AA_1.load_from_axis_angle({0, 1, 0, -M_PI/2});

                quat AA_2;
                AA_2.load_from_axis_angle({1, 0, 0, M_PI/2});

                quat AA_3;
                AA_3.load_from_axis_angle({0, 0, 1, M_PI});

                quat AA_4;
                AA_4.load_from_axis_angle({0, 1, 0, M_PI/8});

                vec3f new_coordinate_system = AA_3.get_rotation_matrix() * AA_2.get_rotation_matrix() * AA_1.get_rotation_matrix() * AA_0.get_rotation_matrix() * AA_4.get_rotation_matrix() *  ((ctr_pos - hand_pos) * dyn_scale) + hand_pos;
                mat3f new_coordinate_matr = AA_3.get_rotation_matrix() * AA_2.get_rotation_matrix() * AA_1.get_rotation_matrix() * AA_0.get_rotation_matrix() * AA_4.get_rotation_matrix() * ctr_rot.get_rotation_matrix();

                quat new_coordinate_quat;
                new_coordinate_quat.load_from_matrix(new_coordinate_matr);

                ctr_pos = new_coordinate_system;
                ctr_rot = new_coordinate_quat;

                vec3f flat_hand_relative = ctr_pos - hand_pos;

                vec3f sword_coordinates_pos = sword_rot.get_rotation_matrix() * flat_hand_relative + sword_pos;

                mat3f sword_coordinates_matr = sword_rot.get_rotation_matrix() * ctr_rot.get_rotation_matrix();

                quat sword_coordinates_rot;
                sword_coordinates_rot.load_from_matrix(sword_coordinates_matr);


                replay.containers[cid]->set_pos({sword_coordinates_pos.x(), sword_coordinates_pos.y(), sword_coordinates_pos.z()});
                replay.containers[cid]->set_rot_quat(sword_coordinates_rot);
                replay.containers[cid]->set_dynamic_scale(dyn_scale * base_scale);

                cid++;
            }
        }
    }
}

inline
void fix_replays_clipping(leap_motion_capture_manager& capture_manager, objects_container* clipping_base)
{
    vec3f up = {0, 1, 0};

    vec3f reference_pos = xyz_to_vec(clipping_base->pos);
    quat reference_rot = clipping_base->rot_quat;

    vec3f reference_forward = reference_rot.get_rotation_matrix() * up;



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

            for(int kk=0; kk < replay.containers.size(); kk++)
            {

                cid++;
            }
        }
    }
}


#endif // LEAP_MOTION_CAPTURE_MANAGEMENT_HPP_INCLUDED
