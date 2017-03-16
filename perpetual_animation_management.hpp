#ifndef PERPETUAL_ANIMATION_MANAGEMENT_HPP_INCLUDED
#define PERPETUAL_ANIMATION_MANAGEMENT_HPP_INCLUDED

#include "leap_motion_capture_management.hpp"
#include "mocap_animation_management.hpp"

///This is it. We're getting there. This is the top conceptual level of directly dealing with animations (ish)
struct perpetual_animation
{
    ///the one that plays 5ever
    ///ok. Remember that this is a series of mocap replays
    mocap_animation base_animation;
    bool going = false;

    std::deque<mocap_animation> queued_animations;

    mocap_animation looping_anim;

    void set_base_animation(mocap_animation& animation);

    void start(leap_motion_capture_manager* capture_manager);
    void add_animation(mocap_animation& animation);

    void interrupt_with_animation(leap_motion_capture_manager* capture_manager, mocap_animation& animation);

    void tick(leap_motion_capture_manager* capture_manager);
};

///this is really just a total functionality test i guess... ingame we'd define these manually not through this ui
struct perpetual_animation_manager
{
    std::vector<perpetual_animation> looping_animations;
    std::vector<perpetual_animation> currently_going;

    void tick_ui(mocap_animation_manager* mocap_manager, leap_motion_capture_manager* capture_manager);

    void tick(leap_motion_capture_manager* leap_capture_manager);
};


#endif // PERPETUAL_ANIMATION_MANAGEMENT_HPP_INCLUDED
