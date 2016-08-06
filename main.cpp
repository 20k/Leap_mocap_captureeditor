#include "../openclrenderer/proj.hpp"

#include <Leap/LeapC.h>

///todo eventually
///split into dynamic and static objects

///todo
///fix memory management to not be atrocious

///we're completely hampered by memory latency

///rift head movement is wrong

struct finger
{
    vec3f tip;
    vec3f rot;
    int hand_id;
};

struct bone
{
    vec3f pos;
    vec3f rot;
    int hand_id;
};

struct leap_motion
{
    LEAP_CONNECTION connection;
    LEAP_DEVICE* device;
    LEAP_CALIBRATION calibration;

    LEAP_DEVICE_REF my_device_ref;

    LEAP_CONNECTION_CONFIG cfg;

    bool device_init = false;
    bool connection_init = false;

    std::map<uint32_t, LEAP_HAND> hand_map;

    leap_motion()
    {
        eLeapRS state = LeapCreateConnection(nullptr, &connection);

        connection_init = true;

        lg::log("LEAP\n\n\n\n\n\n\n");

        if(state != eLeapRS_Success)
        {
            lg::log("Leap not connected probly", state);
            return;
        }

        state = LeapOpenConnection(connection);

        if(state != eLeapRS_Success)
        {
            lg::log("Leap service not started probly", state);
            return;
        }

        LEAP_CONNECTION_MESSAGE* evt = new LEAP_CONNECTION_MESSAGE;

        bool connected = false;

        sf::Clock timeout_clock;
        float timeout_time_ms = 1000.f;

        while(!connected && timeout_clock.getElapsedTime().asMilliseconds() < timeout_time_ms)
        {
            state = LeapPollConnection(connection, 0, evt);

            /*if(state != eLeapRS_Success)
            {
                //lg::log("Poll error", state);
                //printf("poll %x\n", state);
                //return;
            }*/

            LEAP_CONNECTION_INFO* connection_info = new LEAP_CONNECTION_INFO;

            state = LeapGetConnectionInfo(connection, connection_info);

            int len = connection_info->size;

            //printf("Connection len %i\n", len);

            if(connection_info->status != eLeapConnectionStatus_Connected)
            {
                //printf("Not connected %x\n", connection_info->status);
                //lg::log("Leap not connected ", connection_info->status);
            }
            else
            {
                printf("Connected\n");
                connected = true;
            }
        }

        state = LeapPollConnection(connection, 1, evt);

        uint32_t num = 0;

        state = LeapGetDeviceList(connection, nullptr, &num);

        if(num == 0)
        {
            lg::log("0 leaps connected 1 ", state);
            return;
        }

        LEAP_DEVICE_REF* device_ref = new LEAP_DEVICE_REF[num];

        state = LeapGetDeviceList(connection, device_ref, &num);

        if(num == 0)
        {
            lg::log("0 leaps connected 2");
            return;
        }

        my_device_ref = device_ref[0];

        device = new LEAP_DEVICE;

        state = LeapOpenDevice(my_device_ref, device);

        device_init = true;

        if(state != eLeapRS_Success)
        {
            lg::log("device error ", state);
            return;
        }
    }

    void tick()
    {
        if(!device_init)
            return;

        LEAP_CONNECTION_MESSAGE evt;

        LeapPollConnection(connection, 0, &evt);

        if(evt.type == eLeapEventType_Tracking)
        {
            hand_map.clear();

            const LEAP_TRACKING_EVENT* track = evt.tracking_event;

            for(int i=0; i<track->nHands; i++)
            {
                hand_map[track->pHands[i].id] = track->pHands[i];
            }

            //printf("%f frame\n", track->framerate);
        }

        //printf("%i type\n", evt.type);
    }

    vec3f get_index_tip()
    {
        if(hand_map.size() == 0)
            return {0,0,0};

        LEAP_HAND first = hand_map.begin()->second;

        LEAP_VECTOR lvec = first.digits[1].stabilized_tip_position;

        printf("tv %f %f %f\n", lvec.x, lvec.y, lvec.z);

        return {lvec.x, lvec.y, -lvec.z};
    }

    std::vector<bone> get_bones(LEAP_DIGIT d)
    {
        std::vector<bone> ret;

        for(int i=0; i<4; i++)
        {
            bone b;

            LEAP_BONE lb = d.bones[i];

            vec3f prev, next;

            prev = xyz_to_vec(lb.prev_joint);
            next = xyz_to_vec(lb.next_joint);

            b.pos = (next + prev) / 2.f;

            quat q;
            q.from_vec({lb.rotation.x, lb.rotation.y, lb.rotation.z, lb.rotation.w});

            mat3f mat = q.get_rotation_matrix();

            //mat.v[2][0] = -mat.v[2][0];
            //mat.v[2][1] = -mat.v[2][1];
            //mat.v[2][2] = -mat.v[2][2];

            b.rot = mat.get_rotation();
            //b.rot.v[0] = -b.rot.v[0];

            ret.push_back(b);
        }

        return ret;
    }

    std::vector<finger> get_fingers()
    {
        std::vector<finger> ret;

        for(auto& i : hand_map)
        {
            LEAP_HAND first = i.second;

            vec3f palm_pos = xyz_to_vec(first.palm.stabilized_position);
            palm_pos.v[2] = -palm_pos.v[2];

            for(int j=0; j<5; j++)
            {
                LEAP_DIGIT d = first.digits[j];

                finger f;
                f.tip = xyz_to_vec(d.stabilized_tip_position);
                f.tip.v[2] = -f.tip.v[2];

                //vec3f dir = xyz_to_vec(palm);

                vec3f rot = (f.tip - palm_pos).get_euler();

                f.hand_id = first.id;

                std::vector<bone> bones = get_bones(d);

                f.rot = bones.back().rot;

                ret.push_back(f);
            }
        }

        return ret;
    }

    ~leap_motion()
    {
        if(device_init)
        {
            LeapCloseDevice(*device);

        }
        if(connection_init)
        {
            LeapDestroyConnection(connection);
        }
    }
};

struct leap_object_manager
{
    object_context* context;
    leap_motion* motion;

    leap_object_manager(object_context* _context, leap_motion* _motion)
    {
        context = _context;
        motion = _motion;
    }

    std::vector<objects_container*> objects;

    void tick()
    {
        std::vector<finger> fingers = motion->get_fingers();

        for(int i=objects.size(); i<fingers.size(); i++)
        {
            objects_container* ctr = context->make_new();
            ctr->set_file("../openclrenderer/objects/cylinder.obj");
            ctr->set_active(true);
            ///requesting scale will break caching, we cant have a bunch of differently scaled, yet cached objects
            ///yet. Its possible, i just need to figure it out cleanly
            ctr->request_scale(10.f);

            context->load_active();
            context->build_request();

            objects.push_back(ctr);
        }

        for(int i=0; i<fingers.size(); i++)
        {
            objects[i]->set_pos(conv_implicit<cl_float4>(fingers[i].tip));
            objects[i]->set_rot(conv_implicit<cl_float4>(fingers[i].rot));
        }

        for(int i=fingers.size(); i<objects.size(); i++)
        {
            objects[i]->hide();
        }
    }
};

///gamma correct mipmap filtering
///7ish pre tile deferred
///try first bounce in SS, then go to global if fail
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
    //sponza->set_file("../openclrenderer/objects/cylinder.obj");
    sponza->set_load_cube_blank({1, 1, 1});

    sponza->set_active(true);

    sponza->hide();

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

        //sponza->set_pos(conv_implicit<cl_float4>(leap.get_index_tip()));

        window.set_render_event(event);

        window.blit_to_screen(*context.fetch());

        window.flip();

        window.render_block();

        context.build_tick();
        context.flush_locations();

        avg_ftime += c.getElapsedTime().asMicroseconds();

        avg_ftime /= 2;

        if(key.isKeyPressed(sf::Keyboard::M))
            std::cout << c.getElapsedTime().asMicroseconds() << std::endl;

        if(key.isKeyPressed(sf::Keyboard::Comma))
            std::cout << avg_ftime << std::endl;
    }

    ///if we're doing async rendering on the main thread, then this is necessary
    window.render_block();
    cl::cqueue.finish();
    cl::cqueue2.finish();
    glFinish();
}
