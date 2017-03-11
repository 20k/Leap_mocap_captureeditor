#include "../openclrenderer/proj.hpp"

#include <Leap/LeapC.h>
#include <thread>
#include <atomic>
#include <mutex>


///todo eventually
///split into dynamic and static objects

///todo
///fix memory management to not be atrocious

///we're completely hampered by memory latency

///rift head movement is wrong

#include "leap_motion.hpp"
#include "bbox.hpp"
#include "leap_object_manager.hpp"


#include "../imgui/imgui.h"
#include "../imgui/imgui-SFML.h"
#include <vec/vec.hpp>
#include <fstream>
#include "../Sword/fighter.hpp"
#include "../Sword/physics.hpp"
#include "../Sword/map_tools.hpp"
#include "../Sword/util.hpp"

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

    bool valid_hand_id(int id, const std::vector<int32_t>& hand_ids)
    {
        for(auto& i : hand_ids)
        {
            if(i == id)
                return true;
        }

        return false;
    }

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
        lg::log("SEMI ERROR ANIM STATE HAND CONFLICT ARE YOU BEING AN IDIOT");

        //assert(false);

        return old_id;
    }

    ///ok so the issue is, we're interpolating by unique hand ids, rather than hand types
    leap_motion_replay merge_replay(const leap_motion_replay& start, const leap_motion_replay& next)
    {
        leap_motion_replay ret = start;

        int32_t first_left_id = -1;
        int32_t first_right_id = -1;

        std::vector<int32_t> hand_ids;
        std::vector<int> hand_sides; ///0 = left, 1 = right

        for(leap_motion_capture_frame& frame : ret.mocap.data)
        {
           for(auto& i : frame.frame_data)
            {
                hand_ids.push_back(i.first);
                hand_sides.push_back(i.second.type);
            }
        }

        if(hand_ids.size() > 2)
            hand_ids.resize(2);

        if(hand_sides.size() > 2)
            hand_sides.resize(2);

        ///these will need to be configurable
        //int frames_of_padding = 1;
        float animation_pad_time_s = 0.2f;

        float finish_time_s = start.mocap.data.back().time_s;

        float start_next_time = finish_time_s + animation_pad_time_s;

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

        for(leap_motion_capture_frame& frame : next_copy.mocap.data)
        {
            frame.time_s += start_next_time;

            ret.mocap.data.push_back(frame);
        }

        return ret;
    }

    void start(leap_motion_capture_manager* capture_manager)
    {
        if(replay_list.size() == 0)
        {
            lg::log("Invalid replay");
            return;
        }

        merged_replay = capture_manager->replays[replay_list.front()];

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

    void tick(leap_motion_capture_manager* capture_manager)
    {
        if(!going)
            return;

        //leap_motion_replay& cur = capture_manager->currently_replaying_map[currently_going];

        ///we may want to insert interpolation frames
        ///we may want to merge all sub replays into a super replay for ease of interpolate
        ///remember we'd have to patch up ftimes
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

    bool finished()
    {
        return !going;
    }
};

struct mocap_animation_manager
{
    leap_motion_capture_manager* manager;

    std::vector<mocap_animation> animations;
    std::vector<mocap_animation> going_animations;

    mocap_animation in_progress;

    mocap_animation_manager(leap_motion_capture_manager* manage)
    {
        manager = manage;
    }

    void push_mocap_animation(int replay_id)
    {
        in_progress.replay_list.push_back(replay_id);
    }

    void finish_mocap_building_animation()
    {
        if(in_progress.replay_list.size() == 0)
            return;

        animations.push_back(in_progress);

        in_progress = mocap_animation();
    }

    void start_animation(int id)
    {
        going_animations.push_back(animations[id]);

        going_animations.back().start(manager);
    }

    void tick()
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

    void tick_ui()
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
};


///move light down and to the side for specular
///2 hands -> shotgun <-- second
///turn stretched finger into pole <-- first
///step one: fix the disconnect between hand and grabbable
///step two: implement image passthrough
int main(int argc, char *argv[])
{
    lg::set_logfile("./logging.txt");

    std::streambuf* b1 = std::cout.rdbuf();

    std::streambuf* b2 = lg::output->rdbuf();

    std::ios* r1 = &std::cout;
    std::ios* r2 = lg::output;

    r2->rdbuf(b1);


    sf::Clock load_time;

    object_context context;
    context.set_clear_colour({0.25, 0.25, 0.25});

    //sponza->hide();

    engine window;

    window.append_opencl_extra_command_line("-D SHADOWBIAS=20");
    window.append_opencl_extra_command_line("-D SHADOWEXP=2");
    window.append_opencl_extra_command_line("-D SSAO_RAD=5");
    window.append_opencl_extra_command_line("-D depth_icutoff=100");
    window.append_opencl_extra_command_line("-D LEAP");
    window.load(1680,1050,1000, "turtles", "../openclrenderer/cl2.cl", true);
    window.window.setVerticalSyncEnabled(false);

    ImGui::SFML::Init(window.window);

    physics phys;
    phys.load();

    polygonal_world_map polygonal_map;
    polygonal_map.init(context, "../Sword/Res/poly_maps/midmap_1.txt");

    context.load_active();
    context.build(true);

    polygonal_map.level_floor->set_active(false);

    for(auto& i : polygonal_map.other_objects)
    {
        i->set_active(false);
    }

    context.build(true);

    world_collision_handler collision_handler;
    collision_handler.set_map(polygonal_map);

    object_context transparency_context;

    fighter* fight = new fighter(context);

    init_fighter(fight, &phys, 1, &collision_handler, context, transparency_context, "poo", true);


    objects_container* sword = context.make_new();
    sword->set_file("../Sword/Res/sword_red.obj");
    sword->set_active(true);
    sword->hide();

    //window.set_camera_pos({0, 108, -169});

    //rerr: 37.9796 290.584 -255.681
    //rotation: 0.359276 0 0

    window.set_camera_pos({37.9796, 290.584, -255.681});
    window.set_camera_rot({0.359276, 0, 0});

    //window.set_camera_pos((cl_float4){-800,150,-570});

    ///we need a context.unload_inactive
    context.load_active();

    sword->set_dynamic_scale(50.f);

    //sponza->scale(10.f);
    //sponza->set_specular(0.f);

    context.build(true);

    ///if that is uncommented, we use a metric tonne less memory (300mb)
    ///I do not know why
    ///it might be opencl taking a bad/any time to reclaim. Investigate

    auto object_dat = context.fetch();
    window.set_object_data(*object_dat);

    //window.set_tex_data(context.fetch()->tex_gpu);

    sf::Event Event;

    light l;
    l.set_col((cl_float4){1.0f, 1.0f, 1.0f, 0.0f});
    l.set_shadow_casting(1);
    l.set_brightness(0.5f);
    l.radius = 100000;
    l.set_pos((cl_float4){-400, 800, -100, 0});

    light::add_light(&l);

    l.set_pos({0, 1000, 0});
    l.set_shadow_casting(0);

    light::add_light(&l);

    auto light_data = light::build();
    ///

    window.set_light_data(light_data);
    //window.construct_shadowmaps();

    printf("load_time %f\n", load_time.getElapsedTime().asMicroseconds() / 1000.f);

    sf::Keyboard key;

    float avg_ftime = 6000;

    leap_motion leap;
    //leap.enable_images(transparency_context, transparency_context2);

    leap_object_manager leap_object_spawner(&context, &leap);

    leap_motion_capture_manager capture_manager;
    capture_manager.init_manual_containers(context);

    mocap_animation_manager mocap_manager(&capture_manager);

    ImGui::NewFrame();

    ///use event callbacks for rendering to make blitting to the screen and refresh
    ///asynchronous to actual bits n bobs
    ///clSetEventCallback
    while(window.window.isOpen())
    {
        sf::Clock c;

        while(window.window.pollEvent(Event))
        {
            ImGui::SFML::ProcessEvent(Event);

            if(Event.type == sf::Event::Closed)
                window.window.close();

        }

        capture_manager.hide_manual_containers();

        compute::event event;

        context.build_tick();
        context.flip();

        leap.tick(10);
        leap_object_spawner.tick(0.6f);

        capture_manager.tick_replays();


        std::map<uint32_t, LEAP_HAND> this_hand;

        if(leap.hand_history.size() > 0)
            this_hand = leap.hand_history.back();

        if(this_hand.size() > 0)
            capture_manager.set_capture_data(this_hand);

        mocap_manager.tick();

        #if 0
        /*std::vector<leap_object> objects = leap_object_spawner.get_objects();

        std::vector<uint32_t> hand_ids = leap.get_hand_ids();

        for(uint32_t id : hand_ids)
        {
            std::vector<leap_object> objs = leap_object_spawner.get_hand_objects(id);

            leap_object* middle_finger = leap_object_spawner.get_leap_object(id, 2, 0);

            if(!middle_finger)
            {
                lg::log("NO MIDDLE");
                continue;
            }

            vec3f base_pos = middle_finger->current_positional.pos;
            quat  base_rot = middle_finger->current_positional.rot;

            for(leap_object& o : objs)
            {
                vec3f cpos = xyz_to_vec(o.ctr->pos);
                quat  crot = o.ctr->rot_quat;

                vec3f from_base = cpos - base_pos;

                from_base = base_rot.get_rotation_matrix().transp() * from_base;

                vec3f fin_pos = from_base + base_pos;

                mat3f fin_mat = base_rot.get_rotation_matrix().transp() * crot.get_rotation_matrix();

                quat fin_quat;
                fin_quat.load_from_matrix(fin_mat);

                o.ctr->set_pos(conv_implicit<cl_float4>(fin_pos));
                o.ctr->set_rot_quat(fin_quat);
            }
        }*/

        leap_object* left = leap_object_spawner.get_hand(eLeapHandType_Left, 2, 0);
        leap_object* right = leap_object_spawner.get_hand(eLeapHandType_Right, 2, 0);

        /*if(left)
        {
            objects_container* ctr = left->ctr;

            vec4f AA = {1, 0, 0, M_PI/2};

            quat q;
            q.load_from_axis_angle(AA);

            vec4f AA_2 = {0, 1, 0, M_PI/2 - M_PI/16};

            quat q2;
            q2.load_from_axis_angle(AA_2);

            mat3f rmat = ctr->rot_quat.get_rotation_matrix() * q2.get_rotation_matrix() * q.get_rotation_matrix();

            quat rquat;
            rquat.load_from_matrix(rmat);

            vec3f offset = {0, -1, 0};
            vec3f zoffset = {1, 0, 0};

            offset = ctr->rot_quat.get_rotation_matrix() * offset;
            zoffset = ctr->rot_quat.get_rotation_matrix() * zoffset;

            vec3f roffset = xyz_to_vec(ctr->pos) + offset * 20.f + zoffset * 5;

            sword->set_pos(conv_implicit<cl_float4>(roffset));
            sword->set_rot_quat(rquat);
        }*/
        #endif

        if(once<sf::Keyboard::F>())
        {
            fight->queue_attack(attacks::STAB);
        }

        fight->tick(true);
        fight->pos.v[1] = 200.f;
        fight->pos.v[2] = -100;
        fight->rot.v[1] = M_PI;

        fight->look_displacement = {0,0,0};

        #if 0
        if(left)
        {
            fight->override_rhand_pos((xyz_to_vec(left->ctr->pos) - fight->pos).back_rot(0.f, fight->rot));
        }

        if(right)
        {
            fight->focus_pos = (xyz_to_vec(right->ctr->pos) - fight->pos).back_rot(0.f, fight->rot);
        }
        #endif

        fight->update_render_positions();
        fight->update_lights();

        attach_replays_to_fighter_sword(capture_manager, fight->weapon.obj());

        if(capture_manager.fix_clipping)
            fix_replays_clipping(capture_manager, fight->weapon.obj());

        #if 0
        if(left)
        {
            //fight->weapon.obj()->set_pos(left->ctr->pos);

            objects_container* ctr = left->ctr;

            objects_container* csword = fight->weapon.obj();

            vec4f AA = {1, 0, 0, M_PI/2};

            quat q;
            q.load_from_axis_angle(AA);

            vec4f AA_2 = {0, 1, 0, M_PI/2 - M_PI/16};

            quat q2;
            q2.load_from_axis_angle(AA_2);

            mat3f rmat = ctr->rot_quat.get_rotation_matrix() * q2.get_rotation_matrix() * q.get_rotation_matrix();

            quat rquat;
            rquat.load_from_matrix(rmat);

            vec3f offset = {0, -1, 0};
            vec3f zoffset = {1, 0, 0};

            offset = ctr->rot_quat.get_rotation_matrix() * offset;
            zoffset = ctr->rot_quat.get_rotation_matrix() * zoffset;

            vec3f roffset = xyz_to_vec(ctr->pos) + offset * 20.f + zoffset * 5;

            csword->set_pos(conv_implicit<cl_float4>(roffset));
            csword->set_rot_quat(rquat);
        }
        #endif

        for(light* l : fight->my_lights)
        {
            l->set_active(false);
        }

        fight->parts[bodypart::LHAND].obj()->hide();
        fight->parts[bodypart::RHAND].obj()->hide();

        fight->joint_links[1].obj->hide();
        fight->joint_links[3].obj->hide();

        light_data = light::build(&light_data);
        window.set_light_data(light_data);

        context.flush_locations();

        {
            ///do manual async on thread
            ///make a enforce_screensize method, rather than make these hackily do it
            window.generate_realtime_shadowing(*context.fetch());
            event = window.draw_bulk_objs_n(*context.fetch());
            //event = window.do_pseudo_aa();

            context.fetch()->swap_buffers();
        }


        window.set_render_event(event);

        window.blit_to_screen(*context.fetch());

        capture_manager.tick_ui();
        mocap_manager.tick_ui();

        ImGui::Render();
        sf::Time t = sf::microseconds(window.get_frametime_ms() * 1000.f);
        ImGui::SFML::Update(t);

        window.window.resetGLStates();

        window.flip();

        window.render_block();

        avg_ftime += c.getElapsedTime().asMicroseconds();

        avg_ftime /= 2;

        if(key.isKeyPressed(sf::Keyboard::M))
            std::cout << c.getElapsedTime().asMicroseconds() << std::endl;

        if(key.isKeyPressed(sf::Keyboard::Comma))
            std::cout << avg_ftime << std::endl;

        if(key.isKeyPressed(sf::Keyboard::F10))
            break;
    }

    ///if we're doing async rendering on the main thread, then this is necessary
    window.render_block();
    cl::cqueue.finish();
    cl::cqueue2.finish();
    glFinish();

    ImGui::SFML::Shutdown();
}
