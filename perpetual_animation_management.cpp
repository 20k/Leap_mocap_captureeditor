#include "perpetual_animation_management.hpp"

#include "../imgui/imgui.h"
#include "../imgui/imgui-SFML.h"

void perpetual_animation::set_base_animation(mocap_animation& animation)
{
    base_animation = animation;
    looping_anim = animation;

    base_animation.reset();
    looping_anim.reset();
}

void perpetual_animation::start(leap_motion_capture_manager* capture_manager)
{
    looping_anim.start(capture_manager, false);
}

void perpetual_animation::add_animation(mocap_animation& animation)
{
    queued_animations.push_back(animation);
}

void perpetual_animation::interrupt_with_animation(leap_motion_capture_manager* capture_manager, mocap_animation& animation)
{
    leap_motion_replay merged_replay = animation.get_merged_replay(capture_manager, true);

    looping_anim.force_merge_into_running_next_frame(capture_manager, merged_replay);

    ///uuh. and then we just like, interrupt it. Ok
    ///maybe a mocap animation needs a .rewrite_from_frame?
}

void perpetual_animation::tick(leap_motion_capture_manager* capture_manager)
{
    int remaining_frames = looping_anim.get_frames_remaining(capture_manager);

    ///have some frames in flight
    ///remember that unless we manage this we'll have infinite frames!
    ///make this == to smoothing length?
    if(remaining_frames < 300)
    {
        if(queued_animations.size() > 0)
        {
            mocap_animation next_animation = queued_animations.front();
            queued_animations.pop_front();

            leap_motion_replay merged_replay = next_animation.get_merged_replay(capture_manager, true);

            looping_anim.append_into_running(capture_manager, merged_replay);
        }
        else
        {
            leap_motion_replay merged_replay = base_animation.get_merged_replay(capture_manager, false);

            looping_anim.append_into_running(capture_manager, merged_replay);
        }
    }

    looping_anim.check_and_realloc_looping(capture_manager, 100, 100);
}

void perpetual_animation_manager::tick_ui(mocap_animation_manager* mocap_manager, leap_motion_capture_manager* capture_manager)
{
    ImGui::Begin("Looping UI");

    for(int i=0; i<mocap_manager->animations.size(); i++)
    {
        mocap_animation& animation = mocap_manager->animations[i];

        std::string id = std::to_string(i);

        std::string button_id = "Create new Loop: " + id;

        if(ImGui::Button(button_id.c_str()))
        {
            perpetual_animation panim;
            panim.set_base_animation(animation);

            looping_animations.push_back(panim);
        }

        /*std::string add_id = "Push as one off extra to the currently looping animation: " + id;

        if(currently_going.size() > 0 && ImGui::Button(add_id.c_str()))
        {
            currently_going[0].add_animation(animation);
        }*/

        std::string force_id = "Force Merge: " + id;

        if(currently_going.size() > 0 && ImGui::Button(force_id.c_str()))
        {
            currently_going[0].interrupt_with_animation(capture_manager, animation);
        }
    }

    for(int i=0; i<looping_animations.size(); i++)
    {
        perpetual_animation& panim = looping_animations[i];

        std::string id = std::to_string(i);

        std::string bid = "Start looping: " + id;

        if(ImGui::Button(bid.c_str()))
        {
            currently_going.push_back(panim);

            currently_going.back().start(capture_manager);
        }
    }

    ImGui::End();
}

void perpetual_animation_manager::tick(leap_motion_capture_manager* leap_capture_manager)
{
    for(auto& i : currently_going)
    {
        i.tick(leap_capture_manager);
    }
}
