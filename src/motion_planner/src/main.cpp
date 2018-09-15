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
#include "autopilot_msgs/Map2WGS.h"
#include <tf/transform_datatypes.h>
#include "Geography/GaussLocalGeographicCS.hpp"

using namespace std;
using namespace func_simplified;

vector<type_road_point> reference_path; //global path
vector<type_road_point> sample_nodes;
type_road_point vehicle_loc; // coordinate 
vehicle_velocity vehicle_vel; //speed
GridMap grid_map;
bool got_refer_path_flag = false, \
     got_grid_map_flag = false, \
     got_vehicle_state_flag = false, \
     got_motion_goal = false;

double PROPAGATION_TIME = 0.07;
type_road_point motion_goal;

//TODO record got motion goal
void motion_goal_callback(const autopilot_msgs::RouteNode::ConstPtr& msg)
{
    motion_goal.x = msg->x;
    motion_goal.y = msg->y;

    got_motion_goal = true;
}

//TODO get the global path and srtore it in a vector.
void reference_path_callback(const autopilot_msgs::RoutePath::ConstPtr& msg)
{
    ROS_INFO("Starting get the global path");
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
    ROS_INFO("Finished get the global path");
    got_refer_path_flag = true;
}

// TODO get the vehicle current state
void vehicle_state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //set the location
    vehicle_loc.x = msg->pose.pose.position.x;
    vehicle_loc.y = msg->pose.pose.position.y;

    tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 matrix3x3(quat);
    double roll, pitch, yaw;
    matrix3x3.getRPY(roll, pitch, yaw);
    vehicle_loc.angle = yaw;
    
    //set the velocity
    vehicle_vel.vx = msg->twist.twist.linear.x;  //vx
    vehicle_vel.vy = msg->twist.twist.linear.y;  //vy
    vehicle_vel.vz = msg->twist.twist.linear.z;
    
    got_vehicle_state_flag = true;
}

//TODO get the grid map and store into a gridMap object.
void grid_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //TODO store grid map's width, height, resolution and data. 
    grid_map.width = msg->info.width;
    grid_map.height = msg->info.height;
    grid_map.resolution = msg->info.resolution;
    for(int i=0;i<grid_map.width*grid_map.height;i++)
    {
        grid_map.data.push_back(msg->data[i]);
    }
    //TODO print given width, height, resolution and data for checking.
    // cout<<"grid_MAP: "<<grid_map.width<<"  "<<grid_map.height<<"  "<<endl;
    for(int i=0;i<grid_map.height;i++)
    {
        // cout<<"<"<<i<<"th row>: ";
        for(int j=0;j<grid_map.width;j++)
        {
            // if (grid_map.data[i*grid_map.width+j] > 0) 
            // {
            //     cout<<grid_map.data[i*grid_map.width+j]<<" ";
            // }
        }
        // cout<<"</"<<i<<"th row>"<<endl;
    }
    got_grid_map_flag = true;
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
        "/map", 1, grid_map_callback
    );
    // ros::Subscriber route_map_sub = nh.subscribe(
    //     "/map/route_map", 1000, route_map_callback);
	ros::Subscriber reference_path_sub = nh.subscribe(
        "/route/path", 1, reference_path_callback
    );
	ros::Subscriber vehicle_state_sub = nh.subscribe(
        "player_odometry", 1, vehicle_state_callback
    );
    ros::Subscriber motion_goal_sub = nh.subscribe(
        "motion/goal", 1, motion_goal_callback
    );
    // ros::Subscriber static_obstacle_sub = nh.subscribe(
    //     "/detection/static_obstacle_grid", 1000, static_obstacle_callback);
    // ros::Subscriber dynamic_obstacle_sub = nh.subscribe(
    //     "/detection/dynamic_obstacle", 1000, dynamic_obstacle_callback);
    ros::Publisher way_points_pub = nh.advertise<autopilot_msgs::WayPoints>(
        "/planner/way_points",10);

    ros::ServiceClient map_to_wgs_client = \
        nh.serviceClient<autopilot_msgs::Map2WGS>("/map/map_to_wgs");
	
    
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
    ros::Rate loop_rate(10);
    // TODO Toost that motion planning node is running.
    ROS_INFO("Motion Planning Node is Running...");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // TODO node-sampling, parent-searching, curve-propegation, node-adding,
    //      collision-checking, path-selecting and other related works. 
    while (nh.ok()) 
    {
        if( !got_vehicle_state_flag ||
            !got_grid_map_flag ||
            !got_refer_path_flag ||
            !got_motion_goal )
        {
            continue;
        }else{
            got_vehicle_state_flag = false;
            got_grid_map_flag = false;
            got_refer_path_flag = false;
            got_motion_goal = false;
        }
        ROS_INFO("GOT ALL INFORMATION FOR MOTION PLANNING");
        // TODO Initialize related parameters
        // vehicle_loc.x = 11290.4667969;
        // vehicle_loc.y = 8706.98730469;
        // vehicle_loc.angle = reference_path[0].angle;
        double speed = 5;
        type_road_point predicted_vehicle_loc;
        predicted_vehicle_loc.x = \
            vehicle_loc.x + vehicle_vel.vx * PROPAGATION_TIME;
        predicted_vehicle_loc.y = \
            vehicle_loc.y + vehicle_vel.vy * PROPAGATION_TIME;
        predicted_vehicle_loc.angle = vehicle_loc.angle;

        // TODO Initialize tree-expanding-related utility object
        p_fun_main->update_info(
            predicted_vehicle_loc, reference_path, speed, grid_map, motion_goal);
        p_fun_main->setup();
        p_fun_main->initialize_tree();
        ROS_INFO("Motion Goal: (x: %f, y: %f)", motion_goal.x, motion_goal.y);
        // TODO Initialize vector for storing sampled nodes.
        sample_nodes.clear();
        // TODO Initialize work-iteration-time counter,
        //      For each time n hits up 1000, output a path. 
        int n=0;
        // TODO node-sampling, parent-searching, curve-propegation,
        //      node-adding, collision-checking and other related works.
        while(true)
        {
            if (n > 0)
            {
                continue;
            }
            ROS_INFO("Starting  Propagating tree");
            ros::Duration timeout(PROPAGATION_TIME); // Timeout of 0.07 seconds
            ros::Time start_time = ros::Time::now();
            while(ros::Time::now() - start_time < timeout) 
            {
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
                        bool flag_prop = p_propegate_main->curve_propegation
                        (
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
            }

            ROS_INFO("Finished  Propagating tree");
            ROS_INFO("Starting  Searching best path");

            // TODO Path-selecting and other related works
            if(flag && p_fun_main->search_best_path())
            {
                ROS_INFO("Finished  Searching best path");
                ROS_INFO("Starting  Repropagating");
                p_fun_main->repropagating(vehicle_loc);
                flag = false;
                ROS_INFO("Found the path");
                show_path(p_fun_main->selected_path);
                show_tree(p_fun_main->m_tree);
                ROS_INFO("The best path is :");
                vector<type_road_point>::iterator iter;
                for(
                    iter = p_fun_main->selected_path.begin(); 
                    iter != p_fun_main->selected_path.end(); 
                    iter++)
                {
                    ROS_INFO("(x: %f, y: %f)", (*iter).x, (*iter).y);
                }
            }
            ROS_INFO("Finished  Repropagating");
            ROS_INFO("Starting  Publishing the path to Controller");
            
            // Useless currently. Maybe used in the future.
            autopilot_msgs::WayPoints waypoints_msg;
            autopilot_msgs::Map2WGS map2wgs;

            for (int i = 0; i < p_fun_main->selected_path.size(); i++)
            {
                //TODO transform map coordination to WGS coordination.
                GaussLocalGeographicCS gausslocalgeographiccs = 
                    GaussLocalGeographicCS(22.9886565512, 113.2691559583);
                double _ ;
                gausslocalgeographiccs.xyz2llh(
                    p_fun_main->selected_path.at(i).x,
                    p_fun_main->selected_path.at(i).y,
                    0,
                    p_fun_main->selected_path.at(i).latitude,
                    p_fun_main->selected_path.at(i).longitude,
                    _
                );

                ROS_INFO(
                        "Got WG2: (%.10f, %.10f) from Map: (%f, %f)", 
                        p_fun_main->selected_path.at(i).latitude, 
                        p_fun_main->selected_path.at(i).longitude,
                        p_fun_main->selected_path.at(i).x, 
                        p_fun_main->selected_path.at(i).y);

                autopilot_msgs::RouteNode routenode;
                routenode.latitude =
                        p_fun_main->selected_path.at(i).latitude;
                routenode.longitude =
                        p_fun_main->selected_path.at(i).longitude;
                routenode.x =
                        p_fun_main->selected_path.at(i).x;
                routenode.y =
                        p_fun_main->selected_path.at(i).y;
                waypoints_msg.points.push_back(routenode);
                waypoints_msg.speeds.push_back(speed);
            }
            way_points_pub.publish(waypoints_msg);

            ROS_INFO("Finished  Publishing the path to Controller");

            n++;
        }

    }
    loop_rate.sleep();

    return 0;
}
