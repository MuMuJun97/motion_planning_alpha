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
    if(norm_sqrt(ps, local_goal) < goal_size)
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
    if ( m_tree.size() == 0 )
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

        std::cout<< "[IN(FUN) initialize_tree]" << "[>>>>INFO<<<<] " <<
            "initialized the tree"
            <<endl;
    }else if ( m_tree.size > 0 ){

        trim_tree();
    }else{
        
        std::cout<< "[IN(FUN) initialize_tree]" << "[>>>>WARN<<<<] " <<
            "value that should not appear is present, you should stop the whole system"
            <<endl;
    }
}

std::vector<tree<type_node_point>::iterator> fun_simple::select_path_end_nodes()
{
    //TODO select possible end nodes of potential paths.
    std::vector<tree<type_node_point>::iterator> candidate_ends;
    type_road_point rep = local_reference_path[ local_reference_path.size()-1 ];
    int required_states[] = {0, 1 ,2};
    if ( norm_sqrt( rep , goal_point ) < goal_size )
    {
        required_states[1] = 2; required_states[2] = 1;
    }
    int state_index = 0;
    while( candidate_ends.empty() || state_index < 3 )
    {
        //TODO select node with free state and nearby the local goal as possible end nodes
        tree<type_node_point>::iterator it;
        for( it=m_tree.begin();it!=m_tree.end();it++ )
        {   
            if ( is_goal(*it) && it -> state == required_states[state_index] )
            {
                candidate_ends.push_back(it);
            }
        }
        //TODO select leaf node with free state as possible end nodes
        if( candidate_ends.empty() )
        {
            tree<type_node_point>::leaf_iterator lit = m_tree.begin_leaf();
            tree<type_node_point>::leaf_iterator end = m_tree.end_leaf();
            while ( lit != end )
            {
                if ( lit -> state == required_states[state_index] )
                {
                    candidate_ends.push_back( lit );
                }
                ++lit;
            }
        }

        ++state_index;
    }

    return candidate_ends;
}

tree<type_node_point>::iterator fun_simple::select_path_end_node(
    std::vector<tree<type_node_point>::iterator> candidate_ends )
{
    //TODO calculate cost of each path.
    std::vector< type_node_point > candidate_path;
    std::vector< type_path_cost > candidate_paths_costs;
    std::vector< std::pair<double,int> > save_cost;

    for (int i = 0; i < candidate_ends.size(); i++)
    {
        type_path_cost path_cost;
        tree<type_node_point>::iterator pit;
        pit = candidate_ends[i];
        candidate_path.clear();

        while ( pit != m_tree.begin() && pit != NULL )
        {
            candidate_path.push_back( *pit );
            pit = m_tree.parent( pit );
        }

        double end_k = candidate_path[0].k + 
            candidate_path[0].dk * ( candidate_path[0].L - candidate_path[1].L );
        double cost_dk = 0;
        double cost_k = end_k;
        double cost_L = candidate_ends[i] -> L;

        for( int j = 0; j < candidate_path.size(); j++ )
        {
            cost_dk += candidate_path[j].dk;
            cost_k += candidate_path[j].k;
        }
        cost_dk /= ( candidate_path.size() + 1 );
        cost_k /= ( candidate_path.size() + 1 );

        path_cost.diff_curvature = cost_k;
        path_cost.dk = cost_dk;
        path_cost.length = cost_L;
        path_cost.index = i;

        candidate_paths_costs.push_back(path_cost);
    }

    double sum_k = 1e-9, sum_dk = 1e-9, sum_L = 1e-9;
    for ( int m = 0; m < candidate_paths_costs.size(); m++ )
    {
        sum_k += candidate_paths_costs[m].diff_curvature;
        sum_dk += candidate_paths_costs[m].dk;
        sum_L += candidate_paths_costs[m].length;
    }

    for(int m = 0; m < candidate_paths_costs.size(); m++)
    {
        candidate_paths_costs[m].diff_curvature /= sum_k;
        candidate_paths_costs[m].dk /= sum_dk;
        candidate_paths_costs[m].length /= sum_L;
        
        type_path_cost tpc = candidate_paths_costs[m];
        save_cost.push_back( 
            std::make_pair(
                tpc.length * weight_length + 
                tpc.diff_curvature * weight_diff_curvature +
                tpc.dk * weight_dk
                , tpc.index
            )
        );

    }

    //TODO sort the cost of all paths and selected the minimum
    std::sort( save_cost.begin(), save_cost.end(), smalltogreat );
    tree<type_node_point>::iterator it = candidate_ends[ save_cost.begin()->second ];

    return it;
}

bool fun_simple::yield_selected_path( 
    tree<type_node_point>::iterator path_end )
{   
    selected_path.clear();

    std::vector<tree<type_node_point>::iterator> path_its;
    tree<type_node_point>::iterator it = path_end;

    while ( m_tree.is_valid(it) )
    {
        path_its.push_back( it );
        it = m_tree.parent( it );
    }

    std::reverse( path_its.begin(), path_its.end() );

    for (int i = 0; i < path_its.size()-1; i++ )
    {
        std::vector<double> X,Y,Theta;
        tree<type_node_point>::iterator begin = path_its[i];
        tree<type_node_point>::iterator end = path_its[i+1];

        Clothoid::pointsOnClothoid(
            begin -> x, begin -> y, begin -> theta,
            end -> k, end -> dk, end -> L,
            round( it->L * WAYPOINTS_DENSITY ), X, Y, Theta
        );

        X.push_back( end -> x);
        Y.push_back( end -> y);
        Theta.push_back( end -> theta);
        
        for ( int i = 1; i < X.size(); i++ )
        {
            type_road_point trp;
            trp.x = X[i];
            trp.y = Y[i];
            trp.angle = Theta[i];
            if( !passability_check( trp ) )
            {
                if ( i > 1 )
                {
                    double k, dk, L;
                    Clothoid::buildClothoid(
                        begin -> x, begin -> y, begin -> theta,
                        X[i-1], Y[i-1], Theta[i-1], k, dk, L);
                    type_node_point tnp;
                    tnp.x = X[i-1]; tnp.y = Y[i-1]; tnp.theta = Theta[i-1];
                    tnp.k = k; tnp.dk = dk; tnp.L = L;
                    tnp.cost = begin->cost + L;
                    tnp.state = 2;
                    tnp.flag_effective = true;
                    tnp.size = true;
                    m_tree.append_child( begin, tnp );
                }
                m_tree.erase(end);
                selected_path.clear();
                return false;
            }else{

                selected_path.push_back(trp);
            }
        }
    }
    selected_path.push_back( transform_from_node_to_point( *path_end ) );
    return true;
}

bool fun_simple::search_best_path()
{
    std::cout<< "[IN(FUN) search_best_path]" << "[>>>>INFO<<<<] " <<
        "tree size: " << m_tree.size() 
    << std::endl;

    if ( m_tree.size() <= 1 )
    {
        std::cout<< "[IN(FUN) search_best_path]" << "[>>>>WARN<<<<] " <<
            "Tree is empty, no need to search, SKIP"
        <<std::endl;

        return false;
    }
    
    do
    {
        std::vector<tree<type_node_point>::iterator> candidate_ends;
        candidate_ends = select_path_end_nodes();
    
        selected_path_end = select_path_end_node( candidate_ends );

    }while( !yield_selected_path( selected_path_end ) );
    
    return true;
}

void fun_simple::trim_tree()
{
    std::cout<< "[IN(FUN) trim_tree]" << "[>>>>info<<<<] " <<
        "starting trim tree"
        <<endl;
    tree<type_node_point>::iterator it, first_child;
    
    it = selected_path_end;
    while( m_tree.is_valid(it) && m_tree.parent(it) != m_tree.begin() )
    {
        it = m_tree.parent(it);
    }
    first_child = it;

    tree<type_node_point>::sibling_iterator sib = m_tree.begin(m_tree.begin());
    tree<type_node_point>::sibling_iterator end = m_tree.end(m_tree.begin());
    while( sib != end )
    {
        if ( sib != first_child )
            m_tree.erase(sib);
        ++sib;
    }

    m_tree.begin() -> x = vehicle_loc.x;
    m_tree.begin() -> y = vehicle_loc.y;
    m_tree.begin() -> theta = vehicle_loc.angle;

    double k, dk, L;
    Clothoid::buildClothoid(
        m_tree.begin() -> x,
        m_tree.begin() -> y,
        m_tree.begin() -> theta,
        first_child -> x,
        first_child -> y,
        first_child -> theta,
        k, dk, L
    );

    double cost_delta = L - first_child->cost;
    it = m_tree.begin();
    ++it;
    while( it != m_tree.end() )
    {
        it -> cost += cost_delta;
        ++it;
    }

    std::cout<< "[IN(FUN) trim_tree]" << "[>>>>info<<<<] " <<
        "finished trim tree"
        <<endl;

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

    if( norm_sqrt( vehicle_loc_updated, global_coord ) <= PERCISION )
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
        Clothoid::buildClothoid(
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
    Clothoid::buildClothoid(
        vehicle_loc_updated.x, 
        vehicle_loc_updated.y,
        vehicle_loc_updated.angle,
        selected_path.at(selected_index).x, 
        selected_path.at(selected_index).y, 
        selected_path.at(selected_index).angle,
        k, dk, L
    );
    Clothoid::pointsOnClothoid(
        vehicle_loc_updated.x, 
        vehicle_loc_updated.y,
        vehicle_loc_updated.angle,
        k, dk, L,
        round(L * WAYPOINTS_DENSITY), X,Y,Theta);

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
        double temp = norm_sqrt(global_coord, global_path[i]);
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
    //TODO select three nodes that are 10m, 20m 30m 40m(as local goal) far from the vehicle 
    //     to form local reference path
    type_road_point reference_point;
    std::vector<type_road_point> candidate_path;
    for ( int i=0; i < global_path.size(); i++ )
    {
        if ( norm_sqrt(global_coord, global_path[i]) >= THRESHOLD)
        {
            reference_point = global_path[i];
            break;
        }else if ( norm_sqrt(goal_point, global_path[i]) <= goal_size ){
            reference_point = global_path[i];
            break;
        }else{
            candidate_path.push_back( global_path[i] );
        }
    }

    for (int i=1; i <= JOINTS_NUMBER; i++ ){
        double gap = i * THRESHOLD / JOINTS_NUMBER;
        if ( norm_sqrt( global_coord, reference_point ) < gap + PERCISION*2 )
        {
            local_reference_path.push_back( reference_point );
            break;
        }
        double head, tail = -1;
        for (int j=1; j < candidate_path.size(); j++ )
        {
            double distance = norm_sqrt(global_coord, candidate_path[j]);
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
                global_coord, candidate_path[tail], gap );
            local_reference_path.push_back( trp );
        }else if ( head != -1 && tail == -1 ){
            trp = yield_joint_by_distance( 
                candidate_path[head], reference_point, gap );
            local_reference_path.push_back( trp );
        }else{
            trp = yield_joint_by_distance( 
                global_coord, reference_point, gap );
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
            begin.x = global_coord.x;
            begin.y = global_coord.y;
            begin.angle = global_coord.angle;
        }else{
            begin = local_reference_path[i-1];
        }
        end = local_reference_path[i];
        Clothoid::buildClothoid(
            begin.x, begin.y, begin.angle,
            end.x, end.y, end.angle,
            k, dk, L
        );
        Clothoid::pointsOnClothoid(
            begin.x, begin.y, begin.angle,
            k, dk, L, ceil(L), X, Y, Theta
        );

        for ( int j = 0; j < X.size(); j++ )
        {
            type_road_point trp;
            trp.x = X[j]; trp.y = Y[j]; trp.angle = Theta[j];
            if ( collision_check(trp) )
            {
                if ( norm_sqrt( trp, end ) <= PERCISION*2 )
                {
                    local_reference_path[i].x = trp.x;
                    local_reference_path[i].y = trp.y;
                    local_reference_path[i].angle = trp.angle;
                }else if ( norm_sqrt( trp, begin ) <= PERCISION*2 )
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

    local_goal = local_reference_path[ local_reference_path.size() - 1 ];

}

type_road_point fun_simple::yield_joint_by_distance( 
    type_road_point begin, type_road_point end, double distance )
{
    if ( norm_sqrt( begin, end ) < PERCISION*2 )
    {
        return end;
    }
    if ( norm_sqrt( global_coord, end ) <= distance )
    {
        return end;
    }
    type_road_point trp;
    double k, dk, L;
    std::vector<double> X,Y,Theta;
    Clothoid::buildClothoid(
        begin.x, begin.y, begin.angle,
        end.x, end.y, end.angle,
        k, dk, L);
    Clothoid::pointsOnClothoid(
        begin.x, begin.y, begin.angle,
        k, dk, L, ceil(L), X, Y, Theta
    );
    double min = 1e9;
    double index = -1;
    for ( int i = 0; i < X.size(); i++ )
    {
        double temp = sqrt(
            pow( (X[i] - global_coord.x), 2) + pow( (Y[i] - global_coord.y), 2) );
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
        Clothoid::buildClothoid( 
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
        Clothoid::buildClothoid( 
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
