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
    unsigned long point_height = 
        (local_grid_map.height / 2 - point.x - global_coord.x) /
        local_grid_map.resolution;
    unsigned long point_width =
        (point.y + local_grid_map.width / 2 - global_coord.y) /
        local_grid_map.resolution;

    if (local_grid_map.width > point_width && 
        local_grid_map.height > point_height)
    {
        if (local_grid_map.data.at(
            point_height*local_grid_map.width + point_width) > 10)
        {
            return true;
        }
    }
    return false;
}


void fun_simple:: update_info(type_road_point global_coordinate)
{
    global_coord=global_coordinate;
}


bool fun_simple::is_goal(type_node_point node)
{
    type_road_point ps;
    ps = transform_from_node_to_point(node);
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
    goal_size = 5;
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
    std::cout<< "[IN(FUN) search_best_path]" << "[>>>>INFO<<<<] " <<
        "tree size: " << m_tree.size() 
    << std::endl;

    if ( m_tree.size() <= 1 )
    {
        std::cout<< "[IN(FUN) search_best_path]" << "[>>>WARNING<<] " <<
            "Tree is empty, no need to search, SKIP"
        <<std::endl;

        return false;
    }

    tree<type_node_point>::iterator it;
    std::vector<tree<type_node_point>::iterator> save_goal_nodes;
    std::cout << "tree size: " << m_tree.size() << std::endl;
    for(it=m_tree.begin();it!=m_tree.end();it++)
    {
        if(is_goal(*it))
        {
            //std::cout << "Found a goal: " << std::endl;
            save_goal_nodes.push_back(it);
        }
    }
    if(save_goal_nodes.empty())
    {
        std::cout << "Can't find the goal_point, continuing with local goal:" \
            << std::endl;
        tree<type_node_point>::iterator the_nearest_to_goal;
        double min_cost = 100000000;
        for(it=m_tree.begin();it!=m_tree.end();it++)
        {
            double distance_to_goal = sqrt(
                pow((it->x - goal_point.x),2) + pow((it->y - goal_point.y),2));
            if (distance_to_goal < min_cost)
            {
                min_cost = distance_to_goal;
                the_nearest_to_goal = it;
            }
        }
        save_goal_nodes.push_back(the_nearest_to_goal);
    }

    std::vector<type_node_point> path;

    std::vector<type_path_cost> save_path_cost;
    for (int i = 0; i < save_goal_nodes.size(); i++)
    {
        std::cout << "save_goal_nodes.size(): " << save_goal_nodes.size() << std::endl;
        std::cout << save_goal_nodes.at(i)->x << save_goal_nodes.at(i)->y << std::endl;
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

bool fun_simple::repropagating()
{
    if ( ! vehicle_loc.size )
    {
        std::cout<< "[IN(FUN) repropagating]" << "[>>>WARNING<<] " <<
            "vehicle location is not updated, using the old one"
        <<endl;

        return true;

    }else{
        vehicle_loc.size = false;
    }

    //TODO store the updated vehicle location.
    type_road_point vehicle_loc_updated;
    vehicle_loc_updated.x = vehicle_loc.x;
    vehicle_loc_updated.y = vehicle_loc.y;

    if( norm_sqrt( vehicle_loc_updated, global_coord ) <= 1 )
    {
        return true;
    }

    std::cout<< "[IN(FUN) repropagating]" << "[>>>>INFO<<<<] " <<
        "Updated vehicle_loc (x: "<<vehicle_loc_updated.x<<", y: " << 
        vehicle_loc_updated.y<<")"
    <<std::endl;

    double k ,dk ,L;
    double cost = 0;
    int index = -1, selected_index = -1;
    std::vector<std::pair<double,int> > save_cost;
    std::vector<double> X,Y,Theta;

    //TODO calculate the cost from updated vehicle location to each points in 
    //     selected path.
    for(int i = 0; i < selected_path.size(); i++)
    {
        type_road_point tps;
        tps.x = selected_path.at(i).x;
        tps.y = selected_path.at(i).y;
        tps.angle = selected_path.at(i).angle;
        buildClothoid(
            vehicle_loc_updated.x,
            vehicle_loc_updated.y,
            vehicle_loc_updated.angle,
            tps.x, tps.y, tps.angle, k, dk, L);
        cost = L * fun_simple::weight_length + dk * fun_simple::weight_dk +
                dk * L * fun_simple::weight_diff_curvature;
        index = i;
        save_cost.push_back(std::make_pair(cost, index));
    }

    //TODO search and pick up the point that is nearest to vehicle.
    std::sort(save_cost.begin(), save_cost.end(),smalltogreat);
    selected_index = save_cost.begin()->second;

    //TODO generate additional points on the trajectory from updated vehicle location 
    //     to the nearest point.
    pointsOnClothoid(
        vehicle_loc_updated.x, 
        vehicle_loc_updated.y,
        vehicle_loc_updated.angle,
        selected_path.at(selected_index).x, 
        selected_path.at(selected_index).y, 
        selected_path.at(selected_index).angle,
        2,X,Y,Theta);

    //TODO delete the useless points in the selected path.
    selected_path.erase(
        selected_path.begin(), 
        selected_path.begin()+selected_index);

    //TODO add the additional points to the selected path
    for(int k = X.size()-1; k >= 0; k--)
    {
        type_road_point tps;
        tps.x = X[k];
        tps.y = Y[k];
        tps.angle = Theta[k];
        selected_path.insert(selected_path.begin(),tps);
    }
    return true;
}

}
