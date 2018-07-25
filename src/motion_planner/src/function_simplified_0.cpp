#include "function_simplified.hpp"

namespace func_simplified {
fun_simple::fun_simple()
{

}
fun_simple::~fun_simple()
{

}

bool fun_simple::collision_check(type_road_point point)
{

//    MatrixXd P(2,4);
//    car_model.imu_location=point;
//    generate_four_corner_of_car(P,car_model);
//    double minx=P.block(0,0,1,4).minCoeff();
//    double maxx=P.block(0,0,1,4).maxCoeff();
//    if (minx<-3.5)
//        return true;
//    if (maxx>3.5)
//        return true;
//    P*=100.0;
//    //P=ceil(P);
//    Path subj, clip;
//    Paths solution;
//    for(int i=0;i<4;i++)
//    {
//        clip.push_back(IntPoint(P(0,i),P(1,i)));
//        subj.push_back(IntPoint(100*(*bp)->obstacles[i][0],100*(*bp)->obstacles[i][1]));
//    }
//    Clipper c;
//    c.AddPath(subj, ptSubject, true);
//    c.AddPath(clip, ptClip, true);
//    c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
//    if (!solution.empty())
//    {
//        return true;
//    }
    return false;

}
void fun_simple::update_info(type_road_point global_coordinate, std::vector<type_road_point> reference_path,double current_speed)
{
    global_coord=global_coordinate;
    local_reference_path=reference_path;
    speed=current_speed;
}
bool fun_simple::is_goal(type_node_point node)
{
    type_road_point ps;
    transform_from_node_to_point(node,ps);
    if(norm_sqrt(ps,goal_point)<goal_size)
    {
        return true;
    }
    else
        return false;
}
bool fun_simple::add_node_into_tree(type_node_point new_node)
{
    tree<type_node_point>::iterator it;
        bool flag_add=false,flag_new_region=true;
        type_road_point tp,tp_new;
        bool flag_delete_leaf=false;
        for (it=m_tree.begin();it!=m_tree.end();)
        {

            transform_from_node_to_point(*it,tp);
            transform_from_node_to_point(new_node,tp_new);
            if(norm_sqrt(tp,tp_new)<delta_drain)
            {
                if (new_node.cost<it->cost)
                {
                    if(m_tree.number_of_children(it)!=0)
                    {
                        it->flag_effective=false;
                        flag_add=true;
                    }
                    else
                    {
                        it=m_tree.erase(it);
                        flag_delete_leaf=true;
                    }
                }
                flag_new_region=false;
            }
            if (!flag_delete_leaf)
            {
                it++;
            }
            else
            {
                flag_delete_leaf=false;
            }
        }
        if(flag_new_region||flag_add||is_goal(new_node))
        {
            return true;
        }
        return false;
}
void fun_simple::setup()
{
    delta_drain=0.1;
    goal_size=1;
}
void fun_simple::initialize_tree()
{
    tree<type_node_point>::iterator top,first;
        type_node_point pts;
        pts.x=global_coord.x;
        pts.y=global_coord.y;
        pts.theta=global_coord.angle;
        pts.cost=0;
        pts.flag_effective=true;
        top=m_tree.begin();
        first=m_tree.insert(top,pts);
}
bool fun_simple::search_best_path()
{
     tree<type_node_point>::iterator it;
     std::vector<tree<type_node_point>::iterator> save_goal_nodes;
    for(it=m_tree.begin();it!=m_tree.end();it++)
    {
        if(is_goal(*it))
        {
            save_goal_nodes.push_back(it);
        }
    }
    if(save_goal_nodes.empty())
        return false;
    else
        for (int i=0;i<save_goal_nodes.size();i++)
        {

        }
}
}
