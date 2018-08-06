#include "function_simplified.hpp"
bool smalltogreat(std::pair<double,int>a,std::pair<double,int>b)
{
    return a.first<b.first;
}
namespace func_simplified {
fun_simple::fun_simple()
{

}
fun_simple::~fun_simple()
{

}

bool fun_simple::collision_check(type_road_point point)
{
    // TODO Coordinate tranformation
    double transformed_x = local_grid_map.height / 2 - point.x;
    double transformed_y = point.y + local_grid_map.width / 2;
    
    unsigned long point_height = transformed_x / local_grid_map.resolution;
    unsigned long point_width = transformed_y / local_grid_map.resolution;
    if (local_grid_map.width > point_width
        && local_grid_map.height > point_height)
    {
        if (local_grid_map.data.at(
            point_height*local_grid_map.width + point_width) > 10)
        {
            cout<<"Collision!!!: ("<<point.x<<","<<point.y<<") crashs in ("<<\
                point_height<<","<<point_width<<")";
            return true;
        }
    }
    return false;
}

void fun_simple:: update_info(
    type_road_point global_coordinate, 
    std::vector<type_road_point> reference_path,
    double current_speed, GridMap grid_map)
{
    global_coord=global_coordinate;
    local_reference_path=reference_path;
    local_grid_map = grid_map;
    speed=current_speed;
}
bool fun_simple::is_goal(type_node_point node)
{
    type_road_point ps;
    ps = transform_from_node_to_point(node);
    goal_point.x = 11296.2177734;
    goal_point.y = 8746.04199219;
    if(norm_sqrt(ps, goal_point) < goal_size)
    {
        //std::cout << "goal point : " << goal_point.x << " " << goal_point.y << std::endl;
        return true;
    }
    else
        return false;
}
bool fun_simple::add_node_into_tree(type_node_point new_node)
{
    tree<type_node_point>::iterator it;
    bool flag_add = false, flag_new_region = true;
    type_road_point tp, tp_new;
    bool flag_delete_leaf = false;
    for (it = m_tree.begin(); it != m_tree.end();)
    {

        tp = transform_from_node_to_point(*it);
        //std::cout << "(*it).x: " << (*it).x << std::endl;
        //std::cout << tp.x << std::endl;
        tp_new = transform_from_node_to_point(new_node);
        //std::cout << "new_node.cost: " << new_node.cost << std::endl;
        //std::cout << "tree_node.cost: " << it->cost << std::endl << std::endl;
        if(norm_sqrt(tp,tp_new) < delta_drain)
        {
            //std::cout << "norm_sqrt(tp,tp_new): " << norm_sqrt(tp,tp_new) << std::endl;
            //std::cout << "in" << std::endl;
            if (new_node.cost < it->cost)
            {
                flag_add = true;
                if(m_tree.number_of_children(it) != 0)
                {
                    it->flag_effective = false;
                }
                else
                {
                    it = m_tree.erase(it);
                    flag_delete_leaf = true;
                }
            }
            flag_new_region = false;
        }
        if (!flag_delete_leaf)
        {
            it++;
        }
        else
        {
            flag_delete_leaf = false;
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
    delta_drain = 0.1;
    goal_size = 1;
    weight_dk = 0.3;
    weight_diff_curvature = 0.3;
    weight_length = 0.4;
}
void fun_simple::initialize_tree()
{
    m_tree.clear();
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
    std::cout << "tree size: " << m_tree.size() << std::endl;
    for(it=m_tree.begin();it!=m_tree.end();it++)
    {
        if(is_goal(*it))
        {
            //std::cout << "found a goal!!!" << std::endl;
            save_goal_nodes.push_back(it);
        }
    }
    if(save_goal_nodes.empty())
    {
        std::cout << "can't find the goal_point" << std::endl;
        return false;
    }
    else
    {
        std::vector<type_node_point> path;

        std::vector<type_path_cost> save_path_cost;
        for (int i = 0; i < save_goal_nodes.size(); i++)
        {
            std::cout << "save_goal_nodes.size(): " << save_goal_nodes.size() << std::endl;
            it = save_goal_nodes[i];
            path.clear();
            while (it != m_tree.begin() && it != NULL)
            {
                it = m_tree.parent(it);
                //std::cout << "save_goal_nodes[i]: " << save_goal_nodes[i]->x << "  parent: " << it->x << std::endl;
                path.push_back(*it);
            }
            std::cout << "path.size: " << path.size() << std::endl; 
            double cost_dk = 0;
            double cost_diff_curvature = 0;
            double cost_L = (*save_goal_nodes[i]).L;
            //std::cout << "i = " << i << ", (*save_goal_nodes[i]).L: " << (*save_goal_nodes[i]).L << std::endl; 
            //int n = 0;
            for(int j = 0; j < path.size(); j++)
            {
                cost_dk += (path[j]).dk;
                //n++;
            }
            //std::cout << path.size() << std::endl; 
            cost_dk /= path.size();
            //n = 0;

            for(int j = 0; j < path.size()-1; j++)
            {
                cost_diff_curvature += (path[j+1]).k + (path[j+1]).dk * (*save_goal_nodes[i]).L;
                //n++;
            }
            cost_diff_curvature /= path.size();
            type_path_cost path_cost;
            path_cost.diff_curvature = cost_diff_curvature;
            path_cost.dk = cost_dk;
            path_cost.length = cost_L;
            path_cost.index = i;
            save_path_cost.push_back(path_cost);
        }
        double sum_diff_curvature = 0.0000001,sum_dk = 0.0000001, sum_L = 0.0000001;
        for(int m = 0; m < save_path_cost.size(); m++)
        {
            sum_diff_curvature += save_path_cost[m].diff_curvature;
            sum_dk += save_path_cost[m].dk;
            sum_L += save_path_cost[m].length;
        }
        std::vector<std::pair<double,int> > save_cost;
        for(int m = 0; m < save_path_cost.size(); m++)
        {
            save_path_cost[m].diff_curvature /= sum_diff_curvature;
            save_path_cost[m].dk /= sum_dk;
            save_path_cost[m].length /= sum_L;
            save_cost.push_back(std::make_pair(save_path_cost[m].length*weight_length+save_path_cost[m].diff_curvature*weight_diff_curvature+
                                save_path_cost[m].dk*weight_dk,save_path_cost[m].index));

        }
        // SORT THE COST OF ALL THE PATH
        std::sort(save_cost.begin(),save_cost.end(),smalltogreat);
        it = save_goal_nodes[save_cost.begin()->second];
        std::vector<double> X,Y,Theta;
        while(m_tree.is_valid(it) && m_tree.parent(it) != NULL)
        {
            std::cout << "tree node :  x: "<< it->x << ",  y: " << it->y << ",  heading: " << it->theta << std::endl;
            pointsOnClothoid((m_tree.parent(it))->x,(m_tree.parent(it))->y,(m_tree.parent(it))->theta, it->k,it->dk,it->L,30,X,Y,Theta);
            for(int k = X.size()-1; k >= 0; k--)
            {
                type_road_point tps;
                tps.x = X[k];
                tps.y = Y[k];
                tps.angle = Theta[k];
                selected_path.push_back(tps);
            }
            it = m_tree.parent(it);
            //std::cout << "hhhhhhhhhhhhh" << std::endl;
        }
        std::cout << "tree node :  x: "<< it->x << ",  y: " << it->y << ",  heading: " << it->theta << std::endl;
        //std::cout << "hhhhhhhhhhhhhhhhhhhhhhhhhhhh" << std::endl;
        std::reverse(selected_path.begin(), selected_path.end());
        return true;
    }
}
}
