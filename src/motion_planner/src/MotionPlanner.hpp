#ifndef MOTION_PLANNER_MOTIONPLANNER_HPP
#define MOTION_PLANNER_MOTIONPLANNER_HPP

#include "sampling_methods.h"
#include "searching_parent_methods.h"
#include "propegating_methods.h"
#include "function_simplified.hpp"

#include <vector>
#include <utility>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sys/time.h>
#include <stdio.h>

#include "ros/ros.h"

using namespace func_simplified;


class MotionPlanner
{
public:

    double PROPAGATION_TIME = 0.08; // unit is ms

    std::vector<type_road_point> sample_nodes;
    
    fun_simple* p_fun_main;
    //TODO Creat tree-expanding-related utility object.
    sampling_methods* p_sample_main;
    //TODO Creat parent-searching-related utility object.
    searching_parent_methods* p_search_main;
    //TODO Creat curve-propegating-related utility object
    propegating_methods* p_propegate_main;

    int times_recorder = 0;

    MotionPlanner()
    {
        initialization();
    }

private:

    void initialization()
    {
        p_fun_main = new fun_simple;

        p_sample_main = new sampling_methods;
        p_sample_main->p_func_from_sampling = &p_fun_main;

        p_search_main = new searching_parent_methods;
        p_search_main->p_func_methods_searching_parent = &p_fun_main;
        p_search_main->p_sampling_methods_searching_parent = &p_sample_main;

        p_propegate_main = new propegating_methods();
        p_propegate_main->p_func_method_propegating = &p_fun_main;
    }

public:

    bool run_once()
    {
        if ( !is_ok() ){    return false;    }

        if ( is_finished() ) {  return true;    }
        
        printf("Starting run once\n");

        update_state();

        propegate_tree();

        search_path();

        return false;

    }

    void record_process(string filepath)
    {
        times_recorder++;

        ofstream tree_out;
        ofstream waypoints_out;
        ofstream reference_path_out;
        ofstream positions_out;

        string tree_file = filepath + "tree.txt";
        string waypoints_file = filepath + "waypoints.txt";
        string reference_path_file = filepath + "reference_path.txt";
        string positions_file = filepath + "positions.txt";

        if ( times_recorder == 1 )
        {
            tree_out.open( tree_file, ios::out );

            waypoints_out.open( waypoints_file, ios::out );

            reference_path_out.open( reference_path_file, ios::out );

            positions_out.open( positions_file, ios::out );
        }else{
            tree_out.open( tree_file, ios::app );

            waypoints_out.open( waypoints_file, ios::app );

            reference_path_out.open( reference_path_file, ios::app );

            positions_out.open( positions_file, ios::app );
        }

        //TODO record tree of once motion planning.
        tree<type_node_point>::iterator it;
        tree_out << p_fun_main -> m_tree.size() << std::endl;
        for (it = p_fun_main -> m_tree.begin(); 
             it != p_fun_main -> m_tree.end(); it++)
        {
            if( p_fun_main -> m_tree.parent(it) != NULL)
            {
                tree_out <<
                    p_fun_main -> m_tree.parent(it) -> x
                    << "," <<
                    p_fun_main -> m_tree.parent(it) -> y
                << " ";
                tree_out <<
                    it -> x << "," << it -> y
                << std::endl;
            }
        }

        //TODO record selected path of once motion planning.
        waypoints_out << p_fun_main -> selected_path.size() << std::endl;
        for ( int i = 0; i < p_fun_main->selected_path.size(); i++ )
        {
            waypoints_out << 
                p_fun_main->selected_path.at(i).x
                << "," <<
                p_fun_main->selected_path.at(i).y
            << std::endl;
        }

        //TODO record positions of vehicle.
        positions_out << 
            p_fun_main -> vehicle_loc.x
            << "," <<
            p_fun_main -> vehicle_loc.y
        << std::endl;

        // if ( times_recorder > 1 )
        // {
        //     return;
        // }

        //TODO record reference path of once motion planning.
        reference_path_out << p_fun_main -> local_reference_path.size() << std::endl;
        for ( int j = 0; j < p_fun_main->local_reference_path.size(); j++ )
        {
            reference_path_out <<
                p_fun_main->local_reference_path.at(j).x
                << "," <<
                p_fun_main->local_reference_path.at(j).y
            << std::endl;
        }

        return;

    }

    void clear_storages()
    {
        p_fun_main -> selected_path.clear();
    }

    bool is_finished()
    {
        if(
            norm_sqrt( p_fun_main -> vehicle_loc, p_fun_main -> goal_point ) 
            <= p_fun_main -> goal_size
        ){
            return true;
        }
        
        return false;
    }


    void update_state()
    {
        p_fun_main -> vehicle_loc.size = false;
        p_fun_main -> vehicle_vel.size = false;

        // TODO Initialize tree-expanding-related utility object
        p_fun_main->update_info( p_fun_main -> vehicle_loc );
        p_fun_main->setup();
        p_fun_main->initialize_tree();
        p_fun_main->set_local_reference_path();

        printf(
            "Motion Planning From source: (x: %f @%f, y: %f @%f) to Goal: (x: %f, y: %f), goal size is: %f\n",
            p_fun_main -> vehicle_loc.x, p_fun_main -> vehicle_vel.vx,
            p_fun_main -> vehicle_loc.y, p_fun_main -> vehicle_vel.vy,
            p_fun_main -> goal_point.x, p_fun_main -> goal_point.y,
            p_fun_main -> goal_size
        );
        printf("Local Reference Path is: \n");
        for ( int i = 0; i < p_fun_main->local_reference_path.size(); i++ ){
            printf(
            ">>>>>>: (x: %f, y: %f, angle: %f)\n",
            p_fun_main -> local_reference_path[i].x,
            p_fun_main -> local_reference_path[i].y,
            p_fun_main -> local_reference_path[i].angle
            );
        }
    }

    void propegate_tree()
    {
        printf("Starting Propagating tree\n");
        ros::Duration timeout(PROPAGATION_TIME);
        ros::Time start_time = ros::Time::now();

        while ( ros::Time::now() - start_time < timeout )
        {
            bool 
            flag_sample = false,
            flag_search_parent = false,
            flag_prop = false;
            std::vector<type_node_point> new_nodes;

            // TODO Node-sampling and collision-checking.
            flag_sample = p_sample_main -> sampling_nearby_reference_path(
                p_fun_main->local_reference_path
            );

            if ( flag_sample )
            {
                // TODO Parent-searching.
                flag_search_parent = p_search_main->searching_parent_node();
            }

            if ( flag_search_parent )
            {

                // TODO Curve-propegation and collision-checking.
                flag_prop = p_propegate_main -> curve_propegation(
                    p_search_main -> parent_node, 
                    p_sample_main -> sample_node, 
                    new_nodes);
            }

            if( flag_prop )
            {
                // TODO Node-adding.
                tree<type_node_point>::iterator it;
                it = p_search_main -> parent_node;
                for (int i = 0; i < new_nodes.size(); i++)
                {
                    if( p_fun_main->add_node_into_tree( new_nodes[i] ) )
                    {
                        it = p_fun_main -> m_tree.append_child( it, new_nodes[i]);
                    }
                    else
                    {
                        break;
                    }
                }
            }
            
            sample_nodes.push_back( p_sample_main -> sample_node );
            
        }
        printf("Finished  Propagating tree\n");
    }

    void search_path()
    {
        printf("Starting  Searching best path\n");
        printf("Starting  Searching path\n");
        if( p_fun_main->search_best_path() )
        {
            printf("Finished  Searching path\n");
            printf("Starting  Repropagating\n");

            p_fun_main->repropagating();

            p_fun_main->yield_expected_speeds();

            printf("Finished  Repropagating\n");
            printf("Found the path\n");
            printf("The best path is :\n");
            vector<type_road_point>::iterator iter;
            for(
                iter = p_fun_main->selected_path.begin(); 
                iter != p_fun_main->selected_path.end(); 
                iter++)
            {
                printf("(x: %f, y: %f)\n", (*iter).x, (*iter).y);
            }
        }
        printf("Finished Searching best path\n");
    }

    bool is_ok()
    {
        if ( ! p_fun_main -> local_grid_map.size )
        {
            // printf("Missing grid map\n");
            return false;
        }
        if ( ! p_fun_main -> vehicle_loc.size )
        {
            // printf("Missing vehicle location\n");
            return false;
        }
        if ( ! p_fun_main -> vehicle_vel.size )
        {
            // printf("Missing vehicle velocity\n");
            return false;
        }
        if ( p_fun_main -> global_path.size() <= 0 )
        {
            printf("Missing global path\n");
            return false;
        }
        if ( ! p_fun_main -> goal_point.size )
        {
            // printf("Missing goal point\n");
            return false;
        }
        if ( p_fun_main -> speed.size = false )
        {}

        return true;
    }



};

#endif