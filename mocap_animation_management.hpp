#ifndef MOCAP_ANIMATION_MANAGEMENT_HPP_INCLUDED
#define MOCAP_ANIMATION_MANAGEMENT_HPP_INCLUDED

#include <vector>

#include "leap_motion_capture_management.hpp"

///todays project: when we start an animation, actually mash all the underlying replays together and fire off a super replay
///for correct blending etct
///fix clipping next
struct mocap_animation
{
    bool going = false;
    int current_replay = 0;
    std::vector<int> replay_list;
    int currently_going = 0;

    leap_motion_replay merged_replay;

    ///ie a + ret = b
    leap_motion_capture_frame get_map_from_a_to_b(const leap_motion_replay& start, const leap_motion_replay& next);

    leap_motion_capture_frame get_map_from_b_to_a(const leap_motion_replay& start, const leap_motion_replay& next);

    leap_motion_capture_frame get_frame_div(const leap_motion_capture_frame& start, float amount);
    leap_motion_replay patch_hand_ids(const leap_motion_replay& start, const leap_motion_replay& next);

    ///oh crap we need to distribute in increments... accumulate divd??
    void distribute_diff_across_end_of_frame(leap_motion_replay& to_modify, leap_motion_capture_frame& diff, float num_frames, float num_frame_mult);

    ///we could distribute the error across the entire thing... but then the end is an issue
    void distribute_diff_across_start_of_frame(leap_motion_replay& to_modify, leap_motion_capture_frame& diff, float num_frames, float num_frame_mult);

    ///ok so the issue is, we're interpolating by unique hand ids, rather than hand types
    leap_motion_replay merge_replay(const leap_motion_replay& start, const leap_motion_replay& next);
    leap_motion_replay get_merged_replay(leap_motion_capture_manager* capture_manager, bool can_terminate);

    void append_into_running(leap_motion_capture_manager* capture_manager, const leap_motion_replay& to_append);

    ///so take an animation state. Its interpolating between this frame, and next frame
    ///so, we clear everything AFTER next frame, then insert our new animation into there
    void force_merge_into_running_next_frame(leap_motion_capture_manager* capture_manager, const leap_motion_replay& to_force_merge);
    void reset();

    void start(leap_motion_capture_manager* capture_manager, bool can_terminate);

    void tick(leap_motion_capture_manager* capture_manager);

    ///should not ever be used to check for termination etc
    int get_frames_remaining(leap_motion_capture_manager* capture_manager);

    void check_and_realloc_looping(leap_motion_capture_manager* capture_manager, int max_acceptable_frames_elapsed, int min_remaining_to_realloc);

    void force_stop(leap_motion_capture_manager* capture_manager);

    bool finished();
};

struct mocap_animation_manager
{
    leap_motion_capture_manager* manager;

    std::vector<mocap_animation> animations;
    std::vector<mocap_animation> going_animations;

    mocap_animation in_progress;

    mocap_animation_manager(leap_motion_capture_manager* manage);
    void push_mocap_animation(int replay_id);
    void finish_mocap_building_animation();

    void start_animation(int id);

    void tick();

    void tick_ui();
};


#endif // MOCAP_ANIMATION_MANAGEMENT_HPP_INCLUDED
