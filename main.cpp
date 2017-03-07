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

    //window.set_camera_pos({0, 108, -169});

    //rerr: 37.9796 290.584 -255.681
    //rotation: 0.359276 0 0

    window.set_camera_pos({37.9796, 290.584, -255.681});
    window.set_camera_rot({0.359276, 0, 0});

    //window.set_camera_pos((cl_float4){-800,150,-570});

    ///we need a context.unload_inactive
    context.load_active();

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

    ///use event callbacks for rendering to make blitting to the screen and refresh
    ///asynchronous to actual bits n bobs
    ///clSetEventCallback
    while(window.window.isOpen())
    {
        sf::Clock c;

        while(window.window.pollEvent(Event))
        {
            if(Event.type == sf::Event::Closed)
                window.window.close();
        }


        compute::event event;

        context.build_tick();
        context.flip();

        //for()

        leap.tick(0);
        leap_object_spawner.tick();


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
}
