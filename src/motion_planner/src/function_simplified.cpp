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
    //TODO coordination transform
    double delta_x = point.x - global_coord.x;
    double delta_y = point.y - global_coord.y;
    std::vector<double> RT = yield_ratation_matrix( global_coord.angle, 0 );
    double tfd_x = RT[0] * delta_x + RT[1] * delta_y;
    double tfd_y = RT[2] * delta_x + RT[3] * delta_y;
    //TODO find the position of the point in the grid map.
    unsigned long point_height = (unsigned long)
        ( local_grid_map.height * local_grid_map.resolution / 2 - tfd_x) 
        / local_grid_map.resolution;
    unsigned long point_width = (unsigned long)
        ( local_grid_map.width * local_grid_map.resolution / 2 + tfd_y ) 
        / local_grid_map.resolution;
    //TODO check if there is collision. 
    if (local_grid_map.width > point_width && 
        local_grid_map.height > point_height)
    {
        if (local_grid_map.data.at(
            point_height*local_grid_map.width + point_width) > 0)
        {
            std::cout<< "[IN(FUN) collision_check]" << "[>>>>INFO<<<<] ";
            printf( 
                "Collision happened at point(x: %f& %f, y: %f & %f)\n", 
                point.x, tfd_x, point.y, tfd_y);
            return true;
        }
    }

    return false;
}

bool fun_simple::passability_check( type_road_point point )
{
    int box_length = ceil( VEHICLE_BOX_LENGTH / VEHICLE_BOX_RESOLUTION );
    int box_width = ceil( VEHICLE_BOX_WIDTH  / VEHICLE_BOX_RESOLUTION );
    std::vector<double> RT = yield_ratation_matrix( 0, point.angle );
    for (int l = 0; l < box_length; l++)
    {
        double unit_x = VEHICLE_BOX_LENGTH / 2 - l * VEHICLE_BOX_RESOLUTION;
        for ( int w = 0; w < box_width; w++)
        {
            double unit_y = VEHICLE_BOX_WIDTH / 2 - w * VEHICLE_BOX_RESOLUTION;
            double tfd_unit_x = RT[0] * unit_x + RT[1] * unit_y + point.x;
            double tfd_unit_y = RT[2] * unit_x + RT[3] * unit_y + point.y;
            type_road_point trp;
            trp.x = tfd_unit_x; 
            trp.y = tfd_unit_y; 
            trp.angle = point.angle;
            if ( collision_check( trp ) )
            {
                std::cout<< "[IN(FUN) passability_check]" << "[>>>>INFO<<<<]\n";
                return false;
            }
        }
    }
    return true;
}

std::vector<double> fun_simple::yield_ratation_matrix( double source, double target )
{
    double theta = target - source;
    std::vector<double> matrix;
    matrix.push_back(  cos( theta ) );
    matrix.push_back( -sin( theta ) );
    matrix.push_back(  sin( theta ) );
    matrix.push_back(  cos( theta ) );
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
    local_goal_it = it;
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

void fun_simple::trim_tree()
{
    tree<type_node_point>::iterator it, top, first, first_child;
    type_node_point pts;
    tree<type_node_point> td_tree;

    pts.x=vehicle_loc.x;
    pts.y=vehicle_loc.y;
    pts.theta=vehicle_loc.angle;
    pts.cost=0;
    pts.flag_effective=true;
    top=td_tree.begin();
    first=td_tree.insert(top,pts);

    it = local_goal_it;
    while( m_tree.is_valid(it) && m_tree.parent(it) != m_tree.begin() )
    {
        it = m_tree.parent(it);
    }
    first_child = it;
    td_tree.append_child( first, first_child );
    m_tree.clear();
    m_tree = td_tree;
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
    buildClothoid(
        vehicle_loc_updated.x, 
        vehicle_loc_updated.y,
        vehicle_loc_updated.angle,
        selected_path.at(selected_index).x, 
        selected_path.at(selected_index).y, 
        selected_path.at(selected_index).angle,
        k, dk, L
    );
    pointsOnClothoid(
        vehicle_loc_updated.x, 
        vehicle_loc_updated.y,
        vehicle_loc_updated.angle,
        k, dk, L,
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

void fun_simple::set_local_reference_path()
{
    local_reference_path.clear();
    //TODO delete nodes in global path that vehicle has passed.
    int index = -1;
    for (int i = 0; i < global_path.size(); i++)
    {
        double temp = norm_sqrt(vehicle_loc, global_path[i]);
        if ( temp <= PERCISION*2 )
        {
            index = i;
            break;
        }
    }
    if (index >= 0 && index < global_path.size()-1 )
    {
        global_path.erase(global_path.begin(), global_path.begin() + index);
    }
    //TODO select three nodes that are 10m, 20m 30m far from the vehicle 
    //     to form local reference path
    std::vector<type_road_point> candidate_path;
    for ( int i=0; i < global_path.size(); i++ )
    {
        if ( norm_sqrt(vehicle_loc, global_path[i]) >= THRESHOLD)
        {
            local_goal = global_path[i];
            break;
        }else if ( norm_sqrt(goal_point, global_path[i]) <= goal_size ){
            local_goal = goal_point;
            local_reference_path.push_back( local_goal );
            break;
        }else{
            candidate_path.push_back( global_path[i] );
        }
    }

    for (int i=1; i <= JOINTS_NUMBER; i++ ){
        double gap = i * THRESHOLD / (JOINTS_NUMBER + 1);
        double head, tail = -1;
        for (int j=1; j < candidate_path.size(); j++ )
        {
            double distance = norm_sqrt(vehicle_loc, candidate_path[j]);
            if ( abs(gap - distance) <= PERCISION )
            {
                local_reference_path.push_back( candidate_path[j] );
                break;
            }else if ( distance - gap < 0 ){
                head = j;
            }else{
                tail = j;
                break;
            }
        }
        type_road_point trp;
        if ( head != -1 && tail != -1 ){
            trp = yield_joint_by_distance( 
                candidate_path[head], candidate_path[tail], gap );
            local_reference_path.push_back( trp );
        }else if ( head == -1 && tail != -1 ){
            trp = yield_joint_by_distance( 
                vehicle_loc, candidate_path[tail], gap );
            local_reference_path.push_back( trp );
        }else if ( head != -1 && tail == -1 ){
            trp = yield_joint_by_distance( 
                candidate_path[head], local_goal, gap );
            local_reference_path.push_back( trp );
        }else{
            trp = yield_joint_by_distance( 
                vehicle_loc, local_goal, gap );
            local_reference_path.push_back( trp );
        }
    }

    //TODO yield joints by collision
    for ( int i = 0; i < local_reference_path.size(); i++ )
    {
        type_road_point begin, end;
        double k, dk, L;
        std::vector<double> X,Y,Theta;
        if( i == 0 )
        {
            begin.x = vehicle_loc.x;
            begin.y = vehicle_loc.y;
            begin.angle = vehicle_loc.angle;
        }else{
            begin = local_reference_path[i-1];
        }
        end = local_reference_path[i];
        buildClothoid(
        begin.x, begin.y, begin.angle,
        end.x, end.y, end.angle,
        k, dk, L);
        pointsOnClothoid(
            begin.x, begin.y, begin.angle,
            k, dk, L, ceil(L), X, Y, Theta
        );

        for ( int j = 0; j < X.size(); j++ )
        {
            type_road_point trp;
            trp.x = X[j]; trp.y = Y[j]; trp.angle = Theta[j];
            if ( collision_check(trp) )
            {
                if ( norm_sqrt( trp, end ) <= PERCISION*3 )
                {
                    local_reference_path[i].x = trp.x;
                    local_reference_path[i].y = trp.y;
                    local_reference_path[i].angle = trp.angle;
                }else if ( norm_sqrt( trp, begin ) <= PERCISION*3 )
                {
                    break;
                }
                else{
                    local_reference_path.insert(
                        local_reference_path.begin()+i, trp);
                }
                break;
            }
        }
    }

}

type_road_point fun_simple::yield_joint_by_distance( 
    type_road_point begin, type_road_point end, double distance )
{
    if ( norm_sqrt( begin, end ) < PERCISION*2 )
    {
        return end;
    }
    type_road_point trp;
    double k, dk, L;
    std::vector<double> X,Y,Theta;
    buildClothoid(
        begin.x, begin.y, begin.angle,
        end.x, end.y, end.angle,
        k, dk, L);
    pointsOnClothoid(
        begin.x, begin.y, begin.angle,
        k, dk, L, ceil(L), X, Y, Theta
    );
    double min = 1e9;
    double index = -1;
    for ( int i = 0; i < X.size(); i++ )
    {
        double temp = sqrt(
            pow( (X[i] - vehicle_loc.x), 2) + pow( (Y[i] - vehicle_loc.y), 2) );
        if ( abs( temp - distance ) < min )
        {
            min = abs( temp - distance );
            index = i;
        }
    }
    trp.x = X[index]; trp.y = Y[index]; trp.angle = Theta[index];
    return trp;
}

void fun_simple::yield_expected_speeds()
{
    int mode = 1;
    type_road_point the_end_point = selected_path[ selected_path.size() - 1 ];
    if ( norm_sqrt( goal_point, the_end_point ) <= goal_size ||
         the_end_point.state == 2)
    {
        mode = 1;
    }else{
        mode = 0;
    }

    for ( int i = 0; i < selected_path.size(); i++ )
    {   
        type_road_point from, here;
        here = selected_path[i];
        if ( i == 0 )
        {   
            from.x = vehicle_loc.x;
            from.y = vehicle_loc.y;
            from.angle = vehicle_loc.angle;
            from.speed = sqrt( pow( vehicle_vel.vx, 2 ) + pow( vehicle_vel.vy, 2 ) );
        }else{
            from = selected_path[i-1];
        }
        
        selected_path[i].speed = yield_expected_speed(from, here, mode);
    }
}

double fun_simple::yield_expected_speed( type_road_point from, type_road_point here, int mode )
{
    if ( norm_sqrt( goal_point, here ) <= goal_size )
    {
        return 0;
    }
    if ( mode == 0 )
    {
        double k, dk, L, k_i;
        double cur_vel, uniacc_vel;
        std::vector<double> velocity_candidates;
        velocity_candidates.push_back( speed.speed );

        //TODO calculate velocity by curvature
        buildClothoid( 
            from.x, from.y, from.angle,
            here.x, here.y, here.angle,
            k, dk, L );
        k_i = k + L * dk;
        cur_vel = sqrt( LATERAL_ACC / k_i );
        velocity_candidates.push_back( cur_vel ) ;

        //TODO calculate velocity by uniform acceleration
        uniacc_vel = sqrt( pow( from.speed, 2 ) + 2 * LONGITUDINAL_ACC * L );
        velocity_candidates.push_back( uniacc_vel );

        //TODO take the minmum
        vector<double>::iterator min_velocity = min_element(
            velocity_candidates.begin(), velocity_candidates.end());
        
        return *min_velocity;
    }

    if ( mode == 1 )
    {
        double k, dk, L, k_i;
        double cur_vel, nunidcc_vel;
        std::vector<double> velocity_candidates;

        //TODO calculate velocity by curvature
        buildClothoid( 
            from.x, from.y, from.angle,
            here.x, here.y, here.angle,
            k, dk, L );
        k_i = k + L * dk;
        cur_vel = sqrt( LATERAL_ACC / k_i );
        velocity_candidates.push_back( cur_vel ) ;

        //TODO calculate velocity by non-uniform deceleration
        nunidcc_vel = pow( pow( from.speed, 3/2 ) - 3 * L / DEC_COEFF, 2/3 );
        velocity_candidates.push_back( nunidcc_vel );

        //TODO take the minmum
        vector<double>::iterator min_velocity = min_element(
            velocity_candidates.begin(), velocity_candidates.end());
        
        return *min_velocity;
    }
}

}
