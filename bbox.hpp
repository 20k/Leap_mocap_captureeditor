#ifndef BBOX_CRAP_HPP_INCLUDED
#define BBOX_CRAP_HPP_INCLUDED

struct bbox_leap
{
    vec3f min;
    vec3f max;
};

bool within(bbox_leap& b, vec3f test_relative, float fudge = 0.f)
{
    for(int i=0; i<3; i++)
        if(test_relative.v[i] < b.min.v[i] - fudge) ///lower than minimum point, not inside cube
            return false;

    for(int i=0; i<3; i++)
        if(test_relative.v[i] >= b.max.v[i] + fudge) ///greater than maximum point, not inside cube
            return false;

    ///must be within cube
    return true;
}

bbox_leap get_bbox_im_not_even_using_this_anymore(objects_container* obj)
{
    vec3f tl = {FLT_MAX, FLT_MAX, FLT_MAX}, br = {FLT_MIN, FLT_MIN, FLT_MIN};

    for(object& o : obj->objs)
    {
        for(triangle& t : o.tri_list)
        {
            for(vertex& v : t.vertices)
            {
                vec3f pos = {v.get_pos().x, v.get_pos().y, v.get_pos().z};

                tl = min(pos, tl);
                br = max(pos, br);
            }
        }
    }

    return {tl, br};
}


#endif // BBOX_CRAP_HPP_INCLUDED
