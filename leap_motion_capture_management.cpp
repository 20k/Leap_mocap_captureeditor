#include "leap_motion_capture_management.hpp"

#include "../openclrenderer/logging.hpp"
#include "../openclrenderer/objects_container.hpp"
#include "../imgui/imgui.h"
#include <tinydir/tinydir.h>


void leap_motion_replay::set_replay_data(const leap_motion_capture_data& dat)
{
    mocap = dat;
}

///we could smooth out the internals here?
void leap_motion_replay::start_playblack()
{
    clk.restart();
    going = true;
    last_frame = 0;

    /*for(int i=0; i<smooth_level; i++)
    {
        *this = smooth();
    }*/
}

float leap_motion_replay::get_time_s()
{
    return (clk.getElapsedTime().asMicroseconds() / 1000.f) / 1000.f;
}

///assuming the current frame is complete
int leap_motion_replay::get_frames_remaining() const
{
    int current_frame = last_frame;

    int number_of_frames = mocap.data.size();

    return std::max(number_of_frames - current_frame - 1, 0);
}

leap_motion_capture_frame leap_motion_replay::get_current_frame()
{
    return mocap.data[last_frame];
}

leap_motion_capture_frame leap_motion_replay::get_next_frame()
{
    if(last_frame + 1 >= mocap.data.size())
        return mocap.data[last_frame];

    return mocap.data[last_frame + 1];
}

void leap_motion_replay::trim_all_frames_before_current()
{
    int lm1 = last_frame - 1;

    if(lm1 < 0)
        return;

    //lg::log("MOCAP SIZE: ", (int)mocap.data.size());

    mocap.data.erase(mocap.data.begin(), mocap.data.begin() + lm1);

    last_frame -= lm1;
}

bool leap_motion_replay::should_advance_frame()
{
    leap_motion_capture_frame next_frame = get_next_frame();

    if(get_time_s() >= next_frame.time_s)
        return true;

    return false;
}

///0 = closest to current frame, 1 = closest to next frame
float leap_motion_replay::get_frame_frac()
{
    float first_frame_time_s = get_current_frame().time_s;
    float second_frame_time_s = get_next_frame().time_s;

    float ctime = get_time_s();

    //lg::log("CTIME ", ctime, " FTIME ", first_frame_time_s, " STIME ", second_frame_time_s);

    return (ctime - first_frame_time_s) / (second_frame_time_s - first_frame_time_s);
}

void leap_motion_replay::conditionally_advance_frame()
{
    while(should_advance_frame() && (last_frame + 1 < mocap.data.size()))
    {
        last_frame++;
    }
}

bool leap_motion_replay::finished()
{
    return (last_frame >= ((int)mocap.data.size())-1) && can_terminate;
}

JBONE leap_motion_replay::interpolate_bones(JBONE b1, JBONE b2, float a)
{
    JBONE ret = b1;

    vec3f p1 = b1.prev_joint;
    vec3f p2 = b2.prev_joint;

    vec3f n1 = b1.next_joint;
    vec3f n2 = b2.next_joint;

    float w1 = b1.width;
    float w2 = b2.width;

    quat q1 = b1.rotation;
    quat q2 = b2.rotation;

    vec3f rp = p1 * a + p2 * (1.f - a);
    vec3f rn = n1 * a + n2 * (1.f - a);

    float rw = w1 * a + w2 * (1.f - a);

    quat rq = quat::slerp(q1, q2, 1.f - a);

    ret.prev_joint = rp;
    ret.next_joint = rn;

    ret.width = rw;

    ret.rotation = rq;

    return ret;
}

///ok so the issue is, we're interpolating by unique hand ids, rather than hand types
leap_motion_capture_frame leap_motion_replay::get_interpolated_frame()
{
    leap_motion_capture_frame cur = get_current_frame();
    leap_motion_capture_frame next = get_next_frame();

    leap_motion_capture_frame ret;

    ret.time_s = get_time_s();

    float frac = get_frame_frac();
    float interpolate_frac = 1.f - frac;

    ///uint32_t, LEAP_HAND
    for(auto& hands : cur.frame_data)
    {
        uint32_t hid = hands.first;

        JHAND hand = hands.second;

        ///copy all attributes
        ret.frame_data[hid] = hand;

        for(int digit_id = 0; digit_id < 5; digit_id++)
        {
            JDIGIT dig1 = cur.frame_data[hid].digits[digit_id];
            JDIGIT dig2 = next.frame_data[hid].digits[digit_id];

            for(int bone_id = 0; bone_id < 4; bone_id++)
            {
                JBONE b1 = dig1.bones[bone_id];
                JBONE b2 = dig2.bones[bone_id];

                JBONE bi = interpolate_bones(b1, b2, interpolate_frac);

                ret.frame_data[hid].digits[digit_id].bones[bone_id] = bi;
            }
        }
    }

    return ret;
}

leap_motion_capture_frame leap_motion_replay::position_containers(std::vector<objects_container*>& containers)
{
    leap_motion_capture_frame frame = get_interpolated_frame();

    int nhand = 0;

    int cid = 0;

    //lg::log(frame.frame_data.size());

    for(auto& hands : frame.frame_data)
    {
        uint32_t hid = hands.first;

        for(int digit_id = 0; digit_id < 5; digit_id++)
        {
            JDIGIT dig = frame.frame_data[hid].digits[digit_id];

            for(int bone_id = 0; bone_id < 4; bone_id++)
            {
                JBONE bone = dig.bones[bone_id];

                //int id = hid * 5 * 4 + digit_id * 4 + bone_id;

                if(cid >= containers.size())
                {
                    lg::log("NOT ENOUGH CONTAINERS FOR HAND DIGIT/BONES CONUNDRUM");
                    return frame;
                }

                objects_container* ctr = containers[cid];

                ctr->set_pos({bone.get_pos().x(), bone.get_pos().y(), bone.get_pos().z()});
                ctr->set_rot_quat(bone.rotation);

                //lg::log(bone.get_pos().x(), bone.get_pos().y(), bone.get_pos().z());

                cid++;
            }
        }

        nhand++;
    }

    return frame;
}

leap_motion_replay leap_motion_replay::smooth()
{
    leap_motion_replay ret = *this;

    //for(leap_motion_capture_frame& frame : mocap.data)
    for(int i=0; i<mocap.data.size()-1; i++)
    {
        leap_motion_capture_frame& ref_1 = mocap.data[i];
        leap_motion_capture_frame& ref_2 = mocap.data[i+1];

        leap_motion_capture_frame& ret_frame = ret.mocap.data[i];

        for(auto& fdata : ref_1.frame_data)
        {
            const JHAND& j1 = fdata.second;
            const JHAND& j2 = ref_2.frame_data[fdata.first];

            JHAND rhand = hand_op(j1, j2, bone_avg);

            ret_frame.frame_data[fdata.first] = rhand;
        }
    }

    return ret;
}

void leap_motion_capture_manager::add_capture(const leap_motion_capture_data& capture, const std::string& name, bool active_start, bool active_end)
{
    std::string set_name = name;

    size_t strip_start = set_name.find(".mocap");

    if(strip_start != std::string::npos)
    {
        set_name.resize(strip_start);
    }

    if(set_name == "")
    {
        set_name = std::to_string(replays.size());
    }

    leap_motion_replay new_replay;

    int bytes = std::min(set_name.size(), sizeof(new_replay.name));

    memcpy(new_replay.name, set_name.c_str(), bytes);

    new_replay.set_replay_data(capture);

    new_replay.exciting_start = active_start;
    new_replay.exciting_end = active_end;

    replays.push_back(new_replay);
}

void leap_motion_capture_manager::start_capture()
{
    in_progress = leap_motion_capture_data();
    in_progress.start_capture();

    going = true;
}

void leap_motion_capture_manager::set_capture_data(std::map<uint32_t, LEAP_HAND>& hands)
{
    if(!going)
        return;

    in_progress.save_frame(hands);

    if(snap)
        finish_capture();
}

void leap_motion_capture_manager::finish_capture()
{
    if(going)
        add_capture(in_progress);

    //leap_motion_replay& re = replays.back();

    //lg::log(re.mocap.data.size());

    going = false;
    snap = false;
}

void leap_motion_capture_manager::snap_one_frame_capture()
{
    snap = true;

    start_capture();
}

int leap_motion_capture_manager::start_external_replay(const leap_motion_replay& nreplay, std::vector<objects_container*>& containers)
{
    current_replay replay;

    replay.containers = containers;
    replay.replay = nreplay;

    replay.replay.start_playblack();

    int my_id = gid++;

    currently_replaying_map[my_id] = replay;

    return my_id;
}

int leap_motion_capture_manager::start_replay(int id, std::vector<objects_container*>& containers)
{
    current_replay replay;

    replay.containers = containers;
    replay.replay = replays[id];

    replay.replay.start_playblack();

    int my_id = gid++;

    currently_replaying_map[my_id] = replay;

    return my_id;
}

void leap_motion_capture_manager::tick_replays()
{
    std::vector<int> to_erase;

    /*for(int i=0; i<currently_replaying.size(); i++)
    {
        if(currently_replaying[i].replay.finished())
        {
            lg::log("End replay");

            currently_replaying.erase(currently_replaying.begin() + i);
            i--;
            continue;
        }
    }*/

    for(auto& i : currently_replaying_map)
    {
        if(i.second.replay.finished())
        {
            to_erase.push_back(i.first);
        }
    }

    for(auto& i : to_erase)
    {
        currently_replaying_map.erase(i);
    }

    //for(current_replay& replay : currently_replaying)
    for(auto& replay : currently_replaying_map)
    {
        replay.second.replay.conditionally_advance_frame();
        replay.second.last_interpolated = replay.second.replay.position_containers(replay.second.containers);
    }
}


void leap_motion_capture_manager::init_manual_containers(object_context& context)
{
    ctx = &context;

    for(int i=0; i<6 * 5 * 4; i++)
    {
        objects_container* ctr = ctx->make_new();

        ctr->set_file("../openclrenderer/objects/high_cylinder_forward.obj");
        ctr->set_active(true);

        ctx->load_active();
        ctr->hide();
        ctr->set_dynamic_scale(scale);
        ctx->build_request();

        ctrs.push_back(ctr);
    }
}

void leap_motion_capture_manager::hide_manual_containers()
{
    for(auto& i : ctrs)
    {
        i->hide();
    }
}

void leap_motion_capture_manager::tick_ui()
{
    ImGui::Begin("Capture Manager");

    for(int i=0; i<replays.size(); i++)
    {
        leap_motion_replay& replay = replays[i];

        int frames = replay.mocap.data.size();

        int id = i;

        std::string id_str = std::to_string(id);

        std::string button_name = "ID: " + id_str + " " + "Frames: " + std::to_string(frames);

        ImGui::Button(button_name.c_str());

        ImGui::SameLine();

        std::string smooth_name = "Smooth##asasdf" + id_str;

        if(ImGui::Button(smooth_name.c_str()))
        {
            replay = replay.smooth();
        }

        //ImGui::DragInt(smooth_name.c_str(), &replay.smooth_level, 1, 0, 1000);

        ImGui::SameLine();

        std::string button_id_str = std::string("Begin Playback") + std::string("##REKDAGAIN") + std::to_string(i);

        if(ImGui::Button(button_id_str.c_str()))
        {
            start_replay(i, ctrs);
        }

        std::string is_exciting_start_str = std::string("Has Exciting Start###isexcite") + id_str;

        ImGui::Checkbox(is_exciting_start_str.c_str(), &replay.exciting_start);

        std::string is_exciting_end_str = std::string("Has Exciting End###isexcite3434") + id_str;

        ImGui::Checkbox(is_exciting_end_str.c_str(), &replay.exciting_end);

        ImGui::InputText((std::string("Name##qwerqer") + id_str).c_str(), replay.name, sizeof(replay.name) - sizeof(char));

        std::string delete_id_str = std::string("Delete") + std::string("##fdfdfdf") + std::to_string(i);

        if(ImGui::Button(delete_id_str.c_str()))
        {
            replays.erase(replays.begin() + i);
            i--;
            continue;
        }
    }

    if(replays.size() > 0)
        ImGui::NewLine();

    if(ImGui::Button("Start Capture"))
    {
        start_capture();
    }

    if(ImGui::Button("Stop Capture"))
    {
        finish_capture();
    }

    if(going)
    {
        std::string frame = "F: " + std::to_string(in_progress.data.size());

        ImGui::Button(frame.c_str());
    }

    if(ImGui::Button("Snap Capture"))
    {
        snap_one_frame_capture();
    }

    if(ImGui::Button("Save"))
    {
        save("mocap/");
    }

    if(ImGui::Button("Load"))
    {
        load("mocap/");
    }

    ImGui::Checkbox("Fix Clipping (global)", &fix_clipping);

    ImGui::End();
}

void leap_motion_capture_manager::save(const std::string& prefix)
{
    const std::string& extension = ".mocap";

    for(int i=0; i<replays.size(); i++)
    {
        leap_motion_replay& replay = replays[i];


        std::string main_fname = std::to_string(i);

        ///ie we have a string in there
        if(replay.name[0] != 0)
        {
            main_fname = std::string(replay.name);
        }

        std::ofstream out(prefix + main_fname + extension, std::ofstream::binary);

        int num = replay.mocap.data.size();

        out.write((const char*)&num, sizeof(num));

        bool active_start = replay.exciting_start;
        bool active_end = replay.exciting_end;

        out.write((const char*)&active_start, sizeof(active_start));
        out.write((const char*)&active_end, sizeof(active_end));

        for(int m = 0; m < replay.mocap.data.size(); m++)
        {
            leap_motion_capture_frame& frame = replay.mocap.data[m];

            int num_hands = frame.frame_data.size();
            float time_s = frame.time_s;

            ///outfile.write (buffer,size);

            out.write((const char*)&num_hands, sizeof(num_hands));
            out.write((const char*)&time_s, sizeof(time_s));

            for(auto& hands : frame.frame_data)
            {
                out.write((const char*)&hands.first, sizeof(hands.first));
                out.write((const char*)&hands.second, sizeof(hands.second));
            }
        }
    }
}

void leap_motion_capture_manager::load(const std::string& prefix)
{
    int postfix = 0;

    const std::string& extension = ".mocap";

    tinydir_dir dir;
    int i;
    tinydir_open_sorted(&dir, prefix.c_str());

    ///ok... dirty hack ;_; we should just use tinydir
    ///remember, if we do this itll only work if files are actually named consecutively, and extras won't be gone
    for (i = 0; i < dir.n_files; i++)
    {
        tinydir_file tdfile;
        tinydir_readfile_n(&dir, &tdfile, i);

        if (tdfile.is_dir)
        {
            continue;
        }

        ///we have a double // here but it doesn't seem to matter (due to input having a trailing forward slash)
        std::string file = std::string(tdfile.path);
        std::string name = std::string(tdfile.name);

        lg::log(file);

        std::ifstream in(file, std::ifstream::binary);

        if(!in.good())
            continue;

        int num;

        in.read((char*)&num, sizeof(num));

        bool active_start;
        bool active_end;

        in.read((char*)&active_start, sizeof(active_start));
        in.read((char*)&active_end, sizeof(active_end));

        leap_motion_capture_data data;

        for(int i=0; i<num; i++)
        {
            leap_motion_capture_frame frame;

            int num_hands;
            float time_s;

            in.read((char*)&num_hands, sizeof(num_hands));
            in.read((char*)&time_s, sizeof(time_s));

            std::map<uint32_t, JHAND> frame_data;

            for(int i=0; i<num_hands; i++)
            {
                uint32_t hand_id;
                JHAND hand;

                in.read((char*)&hand_id, sizeof(hand_id));
                in.read((char*)&hand, sizeof(hand));

                frame_data[hand_id] = hand;
            }

            frame.frame_data = frame_data;
            frame.time_s = time_s;

            data.data.push_back(frame);
        }

        add_capture(data, name, active_start, active_end);

        postfix++;
    }



    tinydir_close(&dir);

}
