#include "searching_parent_methods.h"

searching_parent_methods::searching_parent_methods()
{
   // p_func_methods_searching_parent=new function_methods();
   // p_sampling_methods_searching_parent=new sampling_methods();
}

bool searching_parent_methods::searching_parent_node()
{
    tree<type_node_point>::iterator it;
    double min_cost = 1e9;
    for (it = (*p_func_methods_searching_parent)->m_tree.begin(); it != (*p_func_methods_searching_parent)->m_tree.end(); it++)
    {
        double x = it->x;
        double y = it->y;
        double angle = it->theta;
        double sx = (*p_sampling_methods_searching_parent)->sample_node.x;
        double sy = (*p_sampling_methods_searching_parent)->sample_node.y;
        double sangle = (*p_sampling_methods_searching_parent)->sample_node.angle;
        double k, dk, L;
        if((x-sx)*(x-sx) + (y-sy)*(y-sy) < 1) return false;
        buildClothoid(x, y, angle, sx, sy, sangle, k, dk, L);

        if(it->cost + L < min_cost)
        {
            min_cost = it->cost + L;
            parent_node = it;
        }
    }
    if (parent_node == NULL)
    {
        std::cout << "can not find a parent node, error happened in searching_parent_node" << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}