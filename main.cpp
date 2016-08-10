#include "../openclrenderer/proj.hpp"

#include <Leap/LeapC.h>
#include <thread>
#include <atomic>
#include <mutex>

#include "BasicExample.h"

#include "CommonExampleInterface.h"
#include "CommonGUIHelperInterface.h"
#include <LinearMath/btTransform.h>
#include "CommonRigidBodyBase.h"


///todo eventually
///split into dynamic and static objects

///todo
///fix memory management to not be atrocious

///we're completely hampered by memory latency

///rift head movement is wrong

#include "leap_motion.hpp"
#include "bbox.hpp"
#include "leap_object_manager.hpp"
#include "grabbables.hpp"
#include "physics_object_manager.hpp"
#include "hand_firer.hpp"


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

    objects_container* sponza = context.make_new();
    sponza->set_file("../openclrenderer/objects/cube.obj");
    //sponza->set_load_cube_blank({10, 10, 10});

    sponza->set_active(true);


    //sponza->hide();

    engine window;

    window.load(1680,1050,1000, "turtles", "../openclrenderer/cl2.cl", true);

    window.set_camera_pos({0, 108, -169});

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
    l.set_shadow_casting(0);
    l.set_brightness(1);
    l.radius = 100000;
    l.set_pos((cl_float4){-200, 1000, -100, 0});

    light::add_light(&l);

    auto light_data = light::build();
    ///

    window.set_light_data(light_data);
    window.construct_shadowmaps();

    printf("load_time %f\n", load_time.getElapsedTime().asMicroseconds() / 1000.f);

    sf::Keyboard key;

    float avg_ftime = 6000;

    leap_motion leap;

    //spawn_cubes(context, grab_manager);

	DummyGUIHelper noGfx;

	CommonExampleOptions options(&noGfx);
	CommonRigidBodyBase*    example = BasicExampleCreateFunc(options);

	example->initPhysics();

    leap_object_manager leap_object_spawner(&context, &leap, example);

    grabbable_manager grab_manager;
    grab_manager.init(&leap, example, &leap_object_spawner);

	physics_object_manager phys(&context, example->getBodies(), example, &grab_manager);

	hand_firer hand_fire;
	hand_fire.init(&leap);

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

        example->stepSimulation(window.get_frametime_ms() / 1000.f);

        example->tick();


        compute::event event;

        {
            ///do manual async on thread
            ///make a enforce_screensize method, rather than make these hackily do it
            event = window.draw_bulk_objs_n(*context.fetch());

            //event = window.do_pseudo_aa();

            //event = window.draw_godrays(*context.fetch());

            window.increase_render_events();

            context.fetch()->swap_depth_buffers();
        }

        leap.tick();
        leap_object_spawner.tick();
        grab_manager.tick(window.get_frametime_ms());
        phys.tick();
        hand_fire.tick();

        std::vector<pvec> positions = hand_fire.get_fire_positions();

        for(int i=0; i<positions.size(); i++)
        {
            pvec pv = positions[i];

            vec3f p = pv.pos;
            vec3f d = pv.dir;

            float vel = 5000.f;

            //printf("%f %f %f d\n", d.v[0], d.v[1], d.v[2]);

            btTransform trans;

            btQuaternion quat;
            quat = quat.getIdentity();

            trans.setRotation(quat);
            trans.setOrigin({p.v[0], p.v[1], p.v[2]});

            float scale = 20.f;

            btBoxShape* colShape = example->createBoxShape(btVector3(scale,scale,scale));

            btRigidBody* rigid = example->createRigidBody(0.1f, trans, colShape);
            rigid->setLinearVelocity(btVector3(d.v[0], d.v[1], d.v[2]) * vel);

            example->insertIntoVector(rigid);
        }

        //sponza->set_pos(conv_implicit<cl_float4>(leap.get_index_tip()));

        window.set_render_event(event);

        window.blit_to_screen(*context.fetch());

        window.flip();

        window.render_block();

        context.build_tick();
        context.flip();
        context.flush_locations();

        avg_ftime += c.getElapsedTime().asMicroseconds();

        avg_ftime /= 2;

        if(key.isKeyPressed(sf::Keyboard::M))
            std::cout << c.getElapsedTime().asMicroseconds() << std::endl;

        if(key.isKeyPressed(sf::Keyboard::Comma))
            std::cout << avg_ftime << std::endl;
    }

    example->exitPhysics();
    delete example;

    ///if we're doing async rendering on the main thread, then this is necessary
    window.render_block();
    cl::cqueue.finish();
    cl::cqueue2.finish();
    glFinish();
}
