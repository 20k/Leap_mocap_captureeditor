#ifndef HAND_TO_HAND_INTERACT_HPP_INCLUDED
#define HAND_TO_HAND_INTERACT_HPP_INCLUDED

struct hand_to_hand_interactor
{
    leap_object_manager* leap_object_manage;
    grabbable_manager* grab_manage;

    void init(leap_object_manager* _leap_object_manage, grabbable_manager* _grab_manage)
    {
        leap_object_manage = _leap_object_manage;
        grab_manage = _grab_manage;
    }

    void tick()
    {
        std::vector<leap_object> objects = leap_object_manage->get_objects();

        std::vector<btRigidBody*> bodies;

        for(auto& i : objects)
        {
            bodies.push_back(i.kinematic);
        }

        std::map<pinch, std::vector<btRigidBody*>, by_id> hand_pinches = grab_manage->get_all_pinches(bodies, 0.2f);
    }
};

#endif // HAND_TO_HAND_INTERACT_HPP_INCLUDED
