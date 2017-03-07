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


/*void spawn_cubes(object_context& context, grabbable_manager& grab)
{
    std::vector<objects_container*> ctr;

    for(int i=0; i<20; i++)
    {
        for(int j=0; j<20; j++)
        {
            vec3f pos = {i, 200, j};

            pos = pos * (vec3f){10, 1, 10};

            objects_container* sponza = context.make_new();
            sponza->set_file("../openclrenderer/objects/cube.obj");

            sponza->set_active(true);

            sponza->request_scale(4.f);

            sponza->set_pos(conv_implicit<cl_float4>(pos));

            sponza->cache = false;

            ctr.push_back(sponza);
        }
    }

    context.load_active();
    context.build_request();

    for(auto& i : ctr)
    {
        grab.add(i);
    }
}*/

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

JHAND leap_hand_to_engine(LEAP_HAND& hand)
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

            jhand.digits[digit_id].bones[bone_id].next_joint = xyz_to_vec(b1.next_joint);
            jhand.digits[digit_id].bones[bone_id].prev_joint = xyz_to_vec(b1.prev_joint);

            jhand.digits[digit_id].bones[bone_id].width = b1.width;
            jhand.digits[digit_id].bones[bone_id].rotation = convert_from_leap_quaternion(b1.rotation);
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

    void save_frame(const std::map<uint32_t, LEAP_HAND>& dat)
    {
        leap_motion_capture_frame frame;

        //frame.frame_data = dat;
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

    void set_replay_data(const leap_motion_capture_data& dat)
    {
        mocap = dat;
    }

    void start_playblack()
    {
        clk.restart();
        going = true;
        last_frame = 0;
    }

    float get_time_s()
    {
        return (clk.getElapsedTime().asMicroseconds() / 1000.f) / 1000.f;
    }

    leap_motion_capture_frame get_current_frame()
    {
        return mocap.data[last_frame];
    }

    leap_motion_capture_frame get_next_frame()
    {
        if(last_frame + 1 >= mocap.data.size())
            return mocap.data[last_frame];

        return mocap.data[last_frame + 1];
    }

    bool should_advance_frame()
    {
        leap_motion_capture_frame next_frame = get_next_frame();

        if(get_time_s() >= next_frame.time_s)
            return true;

        return false;
    }

    ///0 = closest to current frame, 1 = closest to next frame
    float get_frame_frac()
    {
        float first_frame_time_s = get_current_frame().time_s;
        float second_frame_time_s = get_next_frame().time_s;

        float ctime = get_time_s();

        return (ctime - first_frame_time_s) / (second_frame_time_s - first_frame_time_s);
    }

    void conditionally_advance_frame()
    {
        if(should_advance_frame() && (last_frame + 1 < mocap.data.size()))
        {
            last_frame++;
        }
    }

    bool finished()
    {
        return last_frame == mocap.data.size()-1;
    }

    JBONE interpolate_bones(JBONE b1, JBONE b2, float a)
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

        vec3f rn = n1 * a + n2 * (1.f - a);
        vec3f rp = p1 * a + p2 * (1.f - a);

        float rw = w1 * a + w2 * (1.f - a);

        quat rq = quat::slerp(q1, q2, a);

        ret.prev_joint = rn;
        ret.next_joint = rp;

        ret.width = rw;

        ret.rotation = rq;

        return ret;
    }

    leap_motion_capture_frame get_interpolated_frame()
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

    void position_containers(std::vector<objects_container*>& containers)
    {
        leap_motion_capture_frame frame = get_interpolated_frame();

        int nhand = 0;

        int cid = 0;

        for(auto& hands : frame.frame_data)
        {
            uint32_t hid = hands.first;

            for(int digit_id = 0; digit_id < 5; digit_id)
            {
                JDIGIT dig = frame.frame_data[hid].digits[digit_id];

                for(int bone_id = 0; bone_id < 4; bone_id++)
                {
                    JBONE bone = dig.bones[bone_id];

                    //int id = hid * 5 * 4 + digit_id * 4 + bone_id;

                    objects_container* ctr = containers[cid];

                    ctr->set_pos({bone.get_pos().x(), bone.get_pos().y(), bone.get_pos().z()});
                    ctr->set_rot_quat(bone.rotation);

                    cid++;
                }
            }

            nhand++;
        }
    }
};

struct current_replay
{
    std::vector<objects_container*> containers;
    leap_motion_replay replay;
};

struct leap_motion_capture_manager
{
    std::vector<leap_motion_replay> replays;

    ///in progress capture
    leap_motion_capture_data in_progress;

    std::vector<current_replay> currently_replaying;

    bool going = true;

    void add_capture(const leap_motion_capture_data& capture)
    {
        leap_motion_replay new_replay;

        new_replay.set_replay_data(capture);

        replays.push_back(new_replay);
    }

    void start_capture()
    {
        in_progress = leap_motion_capture_data();
        in_progress.start_capture();

        going = true;
    }

    void set_capture_data(const std::map<uint32_t, LEAP_HAND>& hands)
    {
        if(!going)
            return;

        in_progress.save_frame(hands);
    }

    void finish_capture()
    {
        if(going)
            add_capture(in_progress);

        going = false;
    }

    void replay_capture(int id, std::vector<objects_container*>& containers)
    {
        current_replay replay;

        replay.containers = containers;
        replay.replay = replays[id];

        replay.replay.start_playblack();

        currently_replaying.push_back(replay);
    }

    void tick_replays()
    {
        for(int i=0; i<currently_replaying.size(); i++)
        {
            if(currently_replaying[i].replay.finished())
            {
                currently_replaying.erase(currently_replaying.begin() + i);
                i--;
                continue;
            }
        }

        for(current_replay& replay : currently_replaying)
        {
            replay.replay.position_containers(replay.containers);
        }
    }

    void tick_ui()
    {
        ImGui::Begin("Capture Manager");

        for(int i=0; i<replays.size(); i++)
        {

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

    ImGui::SFML::Init(window.window);


    objects_container* sword = context.make_new();
    sword->set_file("../Sword/Res/sword_red.obj");
    sword->set_active(true);

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


        compute::event event;

        context.build_tick();
        context.flip();

        leap.tick(0);
        leap_object_spawner.tick(0.6f);


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

        if(left)
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
        }

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

        ImGui::Begin("Hello");

        ImGui::End();

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
