#include "propegating_methods.h"

propegating_methods::propegating_methods()
{
    //p_func_method_propegating=new function_methods();
}
propegating_methods::~propegating_methods()
{
    //delete p_func_method_propegating;
}
bool propegating_methods::curve_propegation(tree<type_node_point>::iterator parent_node, type_road_point sample_node,type_node_point& new_node)
{
    double x = parent_node->x;
    double y = parent_node->y;
    double angle = parent_node->theta;
    double sx = sample_node.x;
    double sy = sample_node.y;
    double sangle = sample_node.angle;
    double k, dk, L;
    buildClothoid(x, y, angle, sx, sy, sangle, k, dk, L);
    std::vector<double> X, Y, Theta;
    pointsOnClothoid(x, y, angle, k, dk, L, 100, X, Y, Theta);
    new_node.x = sx;
    new_node.y = sy;
    new_node.theta = sangle;
    new_node.L = L;
    new_node.k = k;
    new_node.dk = dk;
    new_node.cost = L + parent_node->cost;
    new_node.flag_effective = true;
    type_road_point ps;
    for(int i = 0;i < 100; i++)
    {
       ps.x = X[i];
       ps.y = Y[i];
       ps.angle = Theta[i];
       if((*p_func_method_propegating)->collision_check(ps))
       {
           return false;
       }
    }
    return true;
}
