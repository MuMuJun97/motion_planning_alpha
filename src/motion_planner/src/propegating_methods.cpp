#include "propegating_methods.h"

propegating_methods::propegating_methods()
{
    //p_func_method_propegating=new function_methods();
}
propegating_methods::~propegating_methods()
{
    //delete p_func_method_propegating;
}
bool propegating_methods::curve_propegation(
    tree<type_node_point>::iterator parent_node, 
    type_road_point sample_node, std::vector<type_node_point>& new_nodes )
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
    int npts = ceil(L * NODES_DENSITY);
    pointsOnClothoid( x, y, angle, k, dk, L, npts, X, Y, Theta );
    
    type_road_point ps;
    for(int i = 1;i < X.size(); i++)
    {
       ps.x = X[i];
       ps.y = Y[i];
       ps.angle = Theta[i];
       if( (*p_func_method_propegating)->collision_check(ps) )
       {
           if ( !new_nodes.empty() ){
               new_nodes[ new_nodes.size() - 1 ].state = 2;
               return true;
           }
           return false;
       }
       else if ( ! (*p_func_method_propegating)->passability_check(ps) )
       {
           if ( !new_nodes.empty() )
           {
               new_nodes[ new_nodes.size() - 1 ].state = 2;
               return true;
           }
           return false;
       }
       type_node_point tnp;
       tnp.x = X[i]; tnp.y = Y[i]; tnp.theta = Theta[i];
       tnp.k = k + L / npts * (i-1) * dk; tnp.dk = dk, tnp.L = L / npts;
       tnp.cost = L / npts * i + parent_node -> cost;
       tnp.flag_effective = true; 
       tnp.size = true;
       new_nodes.push_back(tnp);
    }

    type_node_point tnp;
    tnp.x = sx; tnp.y = sy; tnp.theta = sangle;
    tnp.k = k + L * dk; tnp.dk = dk, tnp.L = L / npts;
    tnp.cost = L + parent_node -> cost;
    tnp.flag_effective = true;
    tnp.size = true;

    new_nodes.push_back(tnp);

    return true;
}
