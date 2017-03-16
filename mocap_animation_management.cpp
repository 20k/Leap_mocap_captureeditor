#include "mocap_animation_management.hpp"

#include "../imgui/imgui.h"
#include "../imgui/imgui-SFML.h"

///ie a + ret = b
leap_motion_capture_frame mocap_animation::get_map_from_a_to_b(const leap_motion_replay& start, const leap_motion_replay& next)
{
    leap_motion_capture_frame A = next.mocap.data.front();
    leap_motion_capture_frame B = start.mocap.data.back();

    A = patch_frame_hand_ids(B, A);

    return frame_op(A, B, bone_sub);
}

leap_motion_capture_frame mocap_animation::get_map_from_b_to_a(const leap_motion_replay& start, const leap_motion_replay& next)
{
    leap_motion_capture_frame A = next.mocap.data.front();
    leap_motion_capture_frame B = start.mocap.data.back();

    A = patch_frame_hand_ids(B, A);

    return frame_op(B, A, bone_sub);
}

leap_motion_capture_frame mocap_animation::get_frame_div(const leap_motion_capture_frame& start, float amount)
{
    return frame_sop(start, amount, bone_div);
}

leap_motion_replay mocap_animation::patch_hand_ids(const leap_motion_replay& start, const leap_motion_replay& next)
{
    std::vector<int32_t> hand_ids;
    std::vector<int> hand_sides; ///0 = left, 1 = right

    //for(leap_motion_capture_frame& frame : ret.mocap.data)

    const leap_motion_capture_frame& check_frame = start.mocap.data.back();

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

    leap_motion_replay next_copy = next;

    ///START PATCHING HAND IDS
    for(leap_motion_capture_frame& frame : next_copy.mocap.data)
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

    return next_copy;
}

///oh crap we need to distribute in increments... accumulate divd??
void mocap_animation::distribute_diff_across_end_of_frame(leap_motion_replay& to_modify, leap_motion_capture_frame& diff, float num_frames, float num_frame_mult)
{
    if(num_frames >= to_modify.get_frames_remaining())
    {
        num_frames = to_modify.get_frames_remaining()-1;
    }

    if(num_frames <= 0)
        return;

    leap_motion_capture_frame divd = frame_sop(diff, num_frames*num_frame_mult, bone_div);
    leap_motion_capture_frame divd_orig = divd;

    int last_frame = to_modify.mocap.data.size()-1;

    for(int fnum = last_frame - num_frames + 2; fnum <= last_frame; fnum++)
    {
        leap_motion_capture_frame& frame = to_modify.mocap.data[fnum];

        frame = frame_op(frame, divd, bone_add);
        divd = frame_op(divd, divd_orig, bone_add);
    }
}

///we could distribute the error across the entire thing... but then the end is an issue
void mocap_animation::distribute_diff_across_start_of_frame(leap_motion_replay& to_modify, leap_motion_capture_frame& diff, float num_frames, float num_frame_mult)
{
    if(num_frames >= to_modify.mocap.data.size()-1)
    {
        num_frames = to_modify.mocap.data.size()-1;
    }

    if(num_frames <= 0)
        return;

    leap_motion_capture_frame divd = frame_sop(diff, num_frames*num_frame_mult, bone_div);
    leap_motion_capture_frame divd_orig = divd;

    for(int fnum=num_frames-1; fnum >= 0; fnum--)
    {
        leap_motion_capture_frame& frame = to_modify.mocap.data[fnum];

        frame = frame_op(frame, divd, bone_add);
        divd = frame_op(divd, divd_orig, bone_add);
    }
}

///ok so the issue is, we're interpolating by unique hand ids, rather than hand types
leap_motion_replay mocap_animation::merge_replay(const leap_motion_replay& start, const leap_motion_replay& next)
{
    leap_motion_replay patched = patch_hand_ids(start, next);

    leap_motion_replay ret = start;

    ret.last_frame = 0;

    ///LAST frame of first to FIRST frame of second
    leap_motion_capture_frame start_to_next_diff = get_map_from_a_to_b(start, patched);
    leap_motion_capture_frame start_to_next_inv_diff = get_map_from_b_to_a(start, patched);

    //leap_motion_capture_frame next_to_start = get_map_from_a_to_b(patched, start);
    //leap_motion_capture_frame next_to_start_inv = get_map_from_b_to_a(patched, start);

    ///uuh. Ok, we'll need to use time later
    float frames_to_distribute_across = std::min(50.f, (float)start.get_frames_remaining());

    ///else default to slerp
    ///?
    //if(frames_to_distribute_across > 50)

    bool distribute_fairly = false;

    if(ret.exciting_end == patched.exciting_start)
        distribute_fairly = true;

    if(distribute_fairly)
    {
        //lg::log("Distributed Fairly");

        distribute_diff_across_end_of_frame(ret, start_to_next_diff, frames_to_distribute_across, 2);
        distribute_diff_across_start_of_frame(patched, start_to_next_inv_diff, frames_to_distribute_across, 2);
    }
    else
    {
        //lg::log("Unfair");

        if(ret.exciting_end)
        {
            //lg::log("Distributed at end of cur");

            distribute_diff_across_end_of_frame(ret, start_to_next_diff, frames_to_distribute_across*2, 1);
        }
        else if(patched.exciting_start)
        {
            //lg::log("Distributed at start of next");

            distribute_diff_across_start_of_frame(patched, start_to_next_inv_diff, frames_to_distribute_across*2, 1);
        }
    }

    ret.exciting_start = false;
    ret.exciting_end = false;

    if(start.exciting_start)
        ret.exciting_start = true;

    if(next.exciting_end)
        ret.exciting_end = true;

    ///these will need to be configurable
    //float animation_pad_time_s = 0.15f;
    float animation_pad_time_s = 1/115.f;

    float finish_time_s = start.mocap.data.back().time_s;

    float start_next_time = finish_time_s + animation_pad_time_s;

    ///this is the actual merge step
    for(leap_motion_capture_frame& frame : patched.mocap.data)
    {
        frame.time_s += start_next_time;

        ret.mocap.data.push_back(frame);
    }

    return ret;
}

leap_motion_replay mocap_animation::get_merged_replay(leap_motion_capture_manager* capture_manager, bool can_terminate)
{
    leap_motion_replay ret = capture_manager->replays[replay_list.front()];

    ret.can_terminate = can_terminate;

    for(int i=1; i<replay_list.size(); i++)
    {
        int id = replay_list[i];

        if(id >= capture_manager->replays.size())
        {
            lg::log("INVALID ID MERGED REPLAY", id);
            continue;
        }

        ret = merge_replay(ret, capture_manager->replays[replay_list[i]]);
    }

    return ret;
}

void mocap_animation::append_into_running(leap_motion_capture_manager* capture_manager, const leap_motion_replay& to_append)
{
    leap_motion_replay& running = capture_manager->currently_replaying_map[currently_going].replay;

    leap_motion_replay res = merge_replay(running, to_append);

    running = res;
}

///so take an animation state. Its interpolating between this frame, and next frame
///so, we clear everything AFTER next frame, then insert our new animation into there
void mocap_animation::force_merge_into_running_next_frame(leap_motion_capture_manager* capture_manager, const leap_motion_replay& to_force_merge)
{
    leap_motion_replay& cur = capture_manager->currently_replaying_map[currently_going].replay;

    int current_frame = cur.last_frame;

    int next_next_frame = current_frame + 2;

    if(next_next_frame >= cur.mocap.data.size()-1)
    {
        append_into_running(capture_manager, to_force_merge);
        return;
    }

    cur.mocap.data.resize(next_next_frame);

    cur = merge_replay(cur, to_force_merge);
}

void mocap_animation::reset()
{
    current_replay = 0;
}

void mocap_animation::start(leap_motion_capture_manager* capture_manager, bool can_terminate)
{
    if(replay_list.size() == 0)
    {
        lg::log("Invalid replay");
        return;
    }

    merged_replay = capture_manager->replays[replay_list.front()];

    merged_replay.can_terminate = can_terminate;

    current_replay = 0;
    going = true;

    for(int i=1; i<replay_list.size(); i++)
    {
        int id = replay_list[i];

        if(id >= capture_manager->replays.size())
        {
            lg::log("INVALID ID ", id);
            return;
        }

        merged_replay = merge_replay(merged_replay, capture_manager->replays[replay_list[i]]);
    }

    //currently_going = capture_manager->start_replay(replay_list.front(), capture_manager->ctrs);

    currently_going = capture_manager->start_external_replay(merged_replay, capture_manager->ctrs);
}

void mocap_animation::tick(leap_motion_capture_manager* capture_manager)
{
    if(!going)
        return;

    //leap_motion_replay& cur = capture_manager->currently_replaying_map[currently_going];

    ///we may want to insert interpolation frames
    ///we may want to merge all sub replays into a super replay for ease of interpolate
    ///remember we'd have to patch up ftimes
    ///the above is now done
    if(capture_manager->currently_replaying_map.find(currently_going) == capture_manager->currently_replaying_map.end())
    {
        /*
        current_replay++;

        if(current_replay >= replay_list.size())
        {
            going = false;
            return;
        }

        //tick(capture_manager);

        currently_going = capture_manager->start_replay(replay_list[current_replay], capture_manager->ctrs);*/

        going = false;

        return;
    }
}

///should not ever be used to check for termination etc
int mocap_animation::get_frames_remaining(leap_motion_capture_manager* capture_manager)
{
    return capture_manager->currently_replaying_map[currently_going].replay.get_frames_remaining();
}

void mocap_animation::check_and_realloc_looping(leap_motion_capture_manager* capture_manager, int max_acceptable_frames_elapsed, int min_remaining_to_realloc)
{
    leap_motion_replay& replay = capture_manager->currently_replaying_map[currently_going].replay;

    ///not a looping animation, this will free itself
    if(replay.can_terminate)
        return;

    if(replay.last_frame >= max_acceptable_frames_elapsed && replay.get_frames_remaining() >= min_remaining_to_realloc)
    {
        replay.trim_all_frames_before_current();
    }
}

void mocap_animation::force_stop(leap_motion_capture_manager* capture_manager)
{
    if(capture_manager->currently_replaying_map.find(currently_going) != capture_manager->currently_replaying_map.end())
    {
        capture_manager->currently_replaying_map.erase(currently_going);
    }
}

bool mocap_animation::finished()
{
    return !going;
}

mocap_animation_manager::mocap_animation_manager(leap_motion_capture_manager* manage)
{
    manager = manage;
}

void mocap_animation_manager::push_mocap_animation(int replay_id)
{
    in_progress.replay_list.push_back(replay_id);
}

void mocap_animation_manager::finish_mocap_building_animation()
{
    if(in_progress.replay_list.size() == 0)
        return;

    animations.push_back(in_progress);

    in_progress = mocap_animation();
}

void mocap_animation_manager::start_animation(int id)
{
    going_animations.push_back(animations[id]);

    going_animations.back().start(manager, true);
}

void mocap_animation_manager::tick()
{
    for(int i=0; i<going_animations.size(); i++)
    {
        mocap_animation& anim = going_animations[i];

        if(anim.finished())
        {
            going_animations.erase(going_animations.begin() + i);

            i--;

            continue;
        }

        anim.tick(manager);
    }
}

void mocap_animation_manager::tick_ui()
{
    ImGui::Begin("Animation UI");

    ///put length of total animation in
    for(int i=0; i<animations.size(); i++)
    {
        std::string id = std::to_string(i);
        std::string anim_length_str = std::to_string(animations[i].replay_list.size());

        std::string button_id = "Launch: " + id + " (" + anim_length_str + ")";

        if(ImGui::Button(button_id.c_str()))
        {
            start_animation(i);
        }
    }

    for(int i=0; i<manager->replays.size(); i++)
    {
        leap_motion_replay& replay = manager->replays[i];

        std::string id = std::to_string(i);

        std::string add_id = "Add to current animation: " + id;

        if(ImGui::Button(add_id.c_str()))
        {
            push_mocap_animation(i);
        }
    }

    int num_anims = in_progress.replay_list.size();

    ImGui::Button((std::to_string(num_anims) + " in animation stack").c_str());

    if(ImGui::Button("Finalise Animation"))
    {
        finish_mocap_building_animation();
    }

    ImGui::End();
}
