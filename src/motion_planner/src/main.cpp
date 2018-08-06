#include <iostream>
#include <cmath>
#include "sampling_methods.h"
#include "searching_parent_methods.h"
#include "propegating_methods.h"
#include "function_simplified.hpp"
#include "plot_utility.h"

#include "ros/ros.h"
#include "autopilot_msgs/Location.h"
#include "autopilot_msgs/RoutePath.h"
#include "autopilot_msgs/RouteNode.h"
#include "autopilot_msgs/MotionState.h"
#include "nav_msgs/OccupancyGrid.h"
#include "delphi_esr_msgs/EsrTrack.h"
#include "autopilot_msgs/WayPoints.h"
#include "autopilot_msgs/RouteMap.h"
#include "autopilot_msgs/RouteEdge.h"

using namespace std;
using namespace func_simplified;

vector<type_road_point> reference_path; //global path
vector<type_road_point> sample_nodes;
type_road_point vehicle_loc; // coordinate 
vehicle_velocity vehicle_vel; //speed
GridMap grid_map;

//TODO get the global path and srtore it in a vector.
void reference_path_callback(const autopilot_msgs::RoutePath::ConstPtr& msg)
{
    for(int i = 0; i < msg->goals.size(); i++)
    {
        type_road_point rp;
        rp.x = msg->goals[i].x;
        rp.y = msg->goals[i].y;
        rp.latitude = msg->goals[i].latitude;
        rp.longitude = msg->goals[i].longitude;
        if(i != msg->goals.size()-1)
        {
            rp.angle = atan(
                (msg->goals[i+1].y - msg->goals[i].y)/
                (msg->goals[i+1].x - msg->goals[i].x));
        }
        else
        {
            rp.angle = atan(
                (msg->goals[i].y - msg->goals[i-1].y)/
                (msg->goals[i].x - msg->goals[i-1].x));
        }
        reference_path.push_back(rp);
        cout << rp.x << " " << rp.y << endl;
    }
    ROS_INFO("Listener:Get the global path");
}

// TODO get the vehicle current state
void vehicle_state_callback(const autopilot_msgs::MotionState::ConstPtr& msg)
{
    //set the location
    vehicle_loc.latitude = msg->gps.latitude;
    vehicle_loc.longitude = msg->gps.longitude;
    vehicle_loc.x = msg->odom.pose.pose.position.x;
    vehicle_loc.y = msg->odom.pose.pose.position.y;
    vehicle_loc.angle = atan(
        msg->odom.pose.pose.orientation.
        y/msg->odom.pose.pose.orientation.x);
    //set the velocity
    vehicle_vel.vx = msg->odom.twist.twist.linear.x;  //vx
    vehicle_vel.vy = msg->odom.twist.twist.linear.y;  //vy
    vehicle_vel.vz = msg->odom.twist.twist.angular.z; //vth
    
    ROS_INFO("Listener:Get the vehicle location");
}

//TODO get the grid map and store into a gridMap object.
void grid_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //TODO Store grid map's width, height, resolution and data. 
    grid_map.width = msg->info.width;
    grid_map.height = msg->info.height;
    grid_map.resolution = msg->info.resolution;
    //TODO Print given width, height and resolution for checking.
    cout<<"GRID_MAP: "<<grid_map.width<<"  "<<grid_map.height<<"  "<<endl;
    //TODO Store grid map's data.
    cout << "Printing grid_map.data which doesn't equal -1:" << endl;
    for(int i=0;i<grid_map.width*grid_map.height;i++)
    {
        grid_map.data.push_back(msg->data[i]);
        if (grid_map.data.at(i) != -1)
        {
            cout << grid_map.data.at(i) << "_";
        }
    }
    cout << endl << "Printing grid_map.data Done" << endl;
}
/*
void static_obstacle_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //to do
}

void dynamic_obstacle_callback(const delphi_esr_msgs::EsrTrack::ConstPtr& msg)
{

}
*/

//TODO show built tree and sampled nodes
void show_tree(tree<type_node_point> m_tree)
{
    plot_utility gp;
    //reference_path
    vector<pair<double,double> > refer_path, refer_path2;
    vector<type_road_point>::iterator it1;
    for(it1 = reference_path.begin(); it1 != reference_path.end(); it1++)
    {
        //cout << "refer_path: " << it1->x << " " << it1->y << endl;
        refer_path.push_back(make_pair(it1->x, it1->y));
    }
    refer_path2 = refer_path;
    //tree
    vector<vector<pair<double,double> > > treedata;
    tree<type_node_point>::iterator it2;
    for (it2 = m_tree.begin(); it2 != m_tree.end(); it2++)
    {
        if(m_tree.parent(it2) != NULL)
        {
            //cout << "tree" << endl;
            vector<pair<double,double>> limb;
            limb.push_back(make_pair(m_tree.parent(it2)->x, 
                           m_tree.parent(it2)->y));
            limb.push_back(make_pair(it2->x, it2->y));
            treedata.push_back(limb);
        }
    }
    //sample_nodes
    vector<pair<double,double> > samp_nodes;
    vector<type_road_point>::iterator it3;
    for(it3 = sample_nodes.begin(); it3 != sample_nodes.end(); it3++)
    {
        //cout << "samp_nodes: " << it3->x << " " << it3->y << endl;
        samp_nodes.push_back(make_pair(it3->x, it3->y));
    }
    gp.plot_tree_for_thesis(refer_path, refer_path2, treedata, 
                            samp_nodes, "tree.pdf");
    cout << "plot the tree" << endl;
}

//TODO print built real path and given reference path into PDF files.
void show_path(vector<type_road_point> path)
{
    plot_utility gp;
    vector<pair<double,double> > real_path;
    vector<type_road_point>::iterator it1;
    for(it1 = path.begin(); it1 != path.end(); it1++)
    {
        //cout << "real_path: " << it1->x << " " << it1->y << endl;
        real_path.push_back(make_pair(it1->x, it1->y));
    }
    vector<pair<double,double> > refer_path;
    for(it1 = reference_path.begin(); it1 != reference_path.end(); it1++)
    {
        //cout << "refer_path: " << it1->x << " " << it1->y << endl;
        refer_path.push_back(make_pair(it1->x, it1->y));
    }
    gp.plot_1d_data_withoriginal(real_path, refer_path, "rpath.pdf");
    gp.plot_random_point(real_path, "path_point.pdf");
}

int main (int argc, char** argv)
{
	ros::init(argc,argv,"motion_planner");
	ros::NodeHandle nh;
    ros::Subscriber grid_map_sub = nh.subscribe(
        "/map", 1000, grid_map_callback);
    // ros::Subscriber route_map_sub = nh.subscribe(
    //     "/map/route_map", 1000, route_map_callback);
	ros::Subscriber reference_path_sub = nh.subscribe(
        "/route/path", 1000, reference_path_callback);
	ros::Subscriber vehicle_state_sub = nh.subscribe(
        "/localization/motion_state", 1000, vehicle_state_callback);
    // ros::Subscriber static_obstacle_sub = nh.subscribe(
    //     "/detection/static_obstacle_grid", 1000, static_obstacle_callback);
    // ros::Subscriber dynamic_obstacle_sub = nh.subscribe(
    //     "/detection/dynamic_obstacle", 1000, dynamic_obstacle_callback);
    ros::Publisher way_points_pub = nh.advertise<autopilot_msgs::WayPoints>(
        "/planner/way_points",1000);
	
    
    // TODO load the global path, and search 30m ahead to get a reference path 
    //      and save it into reference_path.

    // TODO Creat tree-expanding-related utility object.
    fun_simple* p_fun_main;
    p_fun_main = new fun_simple;
    // TODO Creat node-sampling-related utility object.
    sampling_methods* p_sample_main;
    p_sample_main = new sampling_methods;
    p_sample_main->p_func_from_sampling = &p_fun_main;
    // TODO Creat parent-searching-related utility object.
    searching_parent_methods* p_search_main;
    p_search_main = new searching_parent_methods;
    p_search_main->p_func_methods_searching_parent = &p_fun_main;
    p_search_main->p_sampling_methods_searching_parent = &p_sample_main;
    // TODO Creat curve-propegating-related utility object
    propegating_methods* p_propegate_main;
    p_propegate_main = new propegating_methods();
    p_propegate_main->p_func_method_propegating = &p_fun_main;

    bool flag = true;
    // TODO Set working frequency of motion planning node.
    ros::Rate loop_rate(5);
    // TODO Toost that motion planning node is running.
    cout << "Motion Planning Node is Running..." << endl;

    // TODO node-sampling, parent-searching, curve-propegation, node-adding,
    //      collision-checking, path-selecting and other related works. 
    while (nh.ok()) {
        ros::spinOnce();
        if(reference_path.size() && grid_map.data.size())
        {
            // TODO Initialize related parameters
            vehicle_loc.x = 11290.4667969;
            vehicle_loc.y = 8706.98730469;
            vehicle_loc.angle = reference_path[0].angle;
            double speed = 5;
            // TODO Initialize tree-expanding-related utility object
            p_fun_main->update_info(
                vehicle_loc, reference_path, speed, grid_map);
            p_fun_main->setup();
            p_fun_main->initialize_tree();
            // TODO Initialize vector for storing sampled nodes.
            sample_nodes.clear();
            // TODO Initialize work-iteration-time counter,
            //      For each time n hits up 1000, output a path. 
            int n=0;
            // TODO node-sampling, parent-searching, curve-propegation,
            //      node-adding, collision-checking and other related works.
            while(true)
            {
                if (n == 1000) break;
                n++;
                // TODO Node-sampling and collision-checking.
                bool flag_sample=p_sample_main->sampling_nearby_reference_path(
                    p_fun_main->local_reference_path);

                if (flag_sample)
                {
                    // TODO Parent-searching.
                    bool flag_search_parent =\
                        p_search_main->searching_parent_node();

                    if (flag_search_parent)
                    {
                        type_node_point new_node;
                        // TODO Curve-propegation and collision-checking.
                        bool flag_prop = p_propegate_main->curve_propegation(
                            p_search_main->parent_node, 
                            p_sample_main->sample_node, new_node);

                        sample_nodes.push_back(p_sample_main->sample_node);

                        if(flag_prop)
                        {
                            // TODO Node-adding.
                            if(p_fun_main->add_node_into_tree(new_node))
                            {
                                p_fun_main->m_tree.append_child(
                                    p_search_main->parent_node, new_node);
                            }
                        }
                    }

                }
                // TODO Path-selecting and other related works
                if (n%1000==0)
                {
                    cout << n << endl;
                    if(flag && p_fun_main->search_best_path())
                    {
                        
                        flag = false;
                        cout << "found the path" << endl;
                        show_path(p_fun_main->selected_path);
                        show_tree(p_fun_main->m_tree);
                        cout << "the best path is :" << endl;
                        vector<type_road_point>::iterator iter;
                        for(
                            iter = p_fun_main->selected_path.begin(); 
                            iter != p_fun_main->selected_path.end(); 
                            iter++)
                        {
                            cout << "x: " << (*iter).x << \
                            ", y:" << (*iter).y << endl;
                        }
                        break;
                    }
                }
            }

        }
        loop_rate.sleep();  
    }
    
    return 0;
}
