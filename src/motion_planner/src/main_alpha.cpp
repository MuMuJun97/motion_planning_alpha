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

#include <vector>
#include <utility>
#include <string>
#include <typeinfo>
#include <std_msgs/Float64.h>

using namespace func_simplified;


int main(int argc, char** argv)
{
    //TODO initialize ROS Node
    ros::init(argc,argv,"motion_planner");
    ros::NodeHandle nh;

    MotionPlanner motion_planner = MotionPlanner();

    //TODO subscribe demanded topic
    Listener listener = Listener(&nh, motion_planner.p_fun_main);

    //TODO initialize publisher
    Talker talker = Talker(&nh);

    ROS_INFO("Motion Planning Node is Running...");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Rate loop_rate(10);
    while ( nh.ok() )
    {
        motion_planner.run();
        
    }
    loop_rate.sleep();

    return 0;
}

class MotionPlanner
{
public:

    double PROPAGATION_TIME = 0.07;

    std::vector<type_road_point> sample_nodes;
    
    fun_simple* p_fun_main;
    //TODO Creat tree-expanding-related utility object.
    sampling_methods* p_sample_main;
    //TODO Creat parent-searching-related utility object.
    searching_parent_methods* p_search_main;
    //TODO Creat curve-propegating-related utility object
    propegating_methods* p_propegate_main;

    MotionPlanner()
    {
        initialization();
    }

    void run()
    {
        if ( !is_ok() ){    return;    }
        
        while ( true )
        {
            update_state();

            propegate_tree();

            search_best_path();

        }
    }

    void update_state()
    {
        type_road_point predicted_vehicle_loc;
        predicted_vehicle_loc.x =
            p_fun_main -> vehicle_loc.x + 
            p_fun_main -> vehicle_vel.vx * PROPAGATION_TIME;
        
        predicted_vehicle_loc.y =
            p_fun_main -> vehicle_loc.y + 
            p_fun_main -> vehicle_vel.vy * PROPAGATION_TIME;
        
        predicted_vehicle_loc.angle = p_fun_main -> vehicle_loc.angle;

        p_fun_main -> vehicle_loc.size = false;
        p_fun_main -> vehicle_vel.size = false;

        // TODO Initialize tree-expanding-related utility object
        p_fun_main->update_info( predicted_vehicle_loc );
        p_fun_main->setup();
        p_fun_main->initialize_tree();

        ROS_INFO(
            "Motion Planning From source: (x: %f, y: %f) to Goal: (x: %f, y: %f)",
            predicted_vehicle_loc.x, predicted_vehicle_loc.y,
            p_fun_main -> goal_point.x, p_fun_main -> goal_point.y
        );
    }

    void propegate_tree()
    {
        ROS_INFO("Starting  Propagating tree");
        ros::Duration timeout(PROPAGATION_TIME);
        ros::Time start_time = ros::Time::now();
        while ( ros::Time::now() - start_time < timeout )
        {
            // TODO Node-sampling and collision-checking.
            bool flag_sample = p_sample_main -> sampling_nearby_reference_path(
                p_fun_main->local_reference_path
            );

            if ( flag_sample )
            {
                // TODO Parent-searching.
                bool flag_search_parent =
                    p_search_main->searching_parent_node();

                if ( flag_search_parent )
                {
                    type_node_point new_node;

                    // TODO Curve-propegation and collision-checking.
                    bool flag_prop = p_propegate_main -> curve_propegation(
                        p_search_main -> parent_node, 
                        p_sample_main -> sample_node, new_node
                    );

                    sample_nodes.push_back( p_sample_main -> sample_node );

                    if( flag_prop )
                    {
                        // TODO Node-adding.
                        if( p_fun_main->add_node_into_tree( new_node ) )
                        {
                            p_fun_main -> m_tree.append_child(
                                p_search_main -> parent_node, new_node
                            );
                        }
                    }
                }

            }
            
        }
        ROS_INFO("Finished  Propagating tree");
    }

    std::vector<type_road_point> search_best_path()
    {
        ROS_INFO("Starting  Searching best path");
        ROS_INFO("Starting  Searching path");
        if(p_fun_main -> m_tree.size() && p_fun_main->search_best_path())
        {
            ROS_INFO("Finished  Searching path");
            ROS_INFO("Starting  Repropagating");

            p_fun_main->repropagating(p_fun_main -> vehicle_loc);
            
            ROS_INFO("Finished  Repropagating");
            ROS_INFO("Found the path");
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
        ROS_INFO("Finished Searching best path");

    }

    bool is_ok()
    {
        if ( ! p_fun_main -> goal_point.size )
        {
            ROS_WARN("Missing the Goal Point");
            return false;
        }
        if ( p_fun_main -> local_grid_map.size )
        {
             ROS_WARN("Missing the Grid Map");
            return false;
        }
        if ( p_fun_main -> vehicle_loc.size )
        {
            ROS_WARN("Missing the Vehicle Loction");
            return false;
        }
        if ( p_fun_main -> vehicle_vel.size )
        {
            ROS_WARN("Missing the Vehicle Velocity");
            return false;
        }
        if( p_fun_main -> speed.size = false )
        {
            ROS_INFO("Using default speed setting");
        }

        return true;
    }

private:

    void initialization()
    {
        p_fun_main = new fun_simple;

        p_sample_main = new sampling_methods;
        p_sample_main->p_func_from_sampling = &p_fun_main;

        p_search_main->p_func_methods_searching_parent = &p_fun_main;
        p_search_main->p_sampling_methods_searching_parent = &p_sample_main;

        p_propegate_main = new propegating_methods();
        p_propegate_main->p_func_method_propegating = &p_fun_main;
    }

};


class Talker
{
public: 
    Talker(ros::NodeHandle* nodehandle):_nh(*nodehandle)
    {
        initializePublishers();
    }

    ros::Publisher get_publisher(string topic)
    {
        ros::Publisher returned_pub;
        for ( int i = 0; i < _pubs.size(); i++ )
        {
            if ( _pubs.at(i).first == topic )
            {
                returned_pub = _pubs.at(i).second;
            }
        }
        return returned_pub;
    }

private:
    ros::NodeHandle _nh;
    std::vector<std::pair<string,ros::Publisher>> _pubs;

    void initializePublishers()
    {
        ros::Publisher pub_0 = _nh.advertise
            <autopilot_msgs::WayPoints>( "/planner/way_points", 10);
        
        _pubs.push_back( make_pair("/planner/way_points", pub_0) );
    }        
};


class Listener
{
public:
    Listener(ros::NodeHandle* nodehandle,fun_simple* p_fun_main)
    :_nh(*nodehandle), _fun_simple(*p_fun_main)
    {
        initializeSubscribers();
    }
private:
    ros::NodeHandle _nh;
    fun_simple _fun_simple;

    void initializeSubscribers()
    {
        ros::Subscriber sub_0 = _nh.subscribe(
            "/map", 10, &Listener::grid_map_callback, this
        );

        ros::Subscriber sub_1 = _nh.subscribe(
            "/route/path", 10, &Listener::reference_path_callback, this
        );

        ros::Subscriber sub_2 = _nh.subscribe(
            "player_odometry", 10, &Listener::vehicle_odometry_callback, this
        );

        ros::Subscriber sub_3 = _nh.subscribe(
            "motion/goal", 10, &Listener::motion_goal_callback, this
        );

        ros::Subscriber sub_4 = _nh.subscribe(
            "motion/speed", 10, &Listener::motion_speed_callback, this
        );

    }

    void grid_map_callback(
        const nav_msgs::OccupancyGrid::ConstPtr& msg
    ){
        _fun_simple.local_grid_map.width = msg->info.width;
        _fun_simple.local_grid_map.height = msg->info.height;
        _fun_simple.local_grid_map.resolution = msg->info.resolution;
        for(int i=0; i< msg->info.width * msg->info.height; i++)
        {
            _fun_simple.local_grid_map.data.push_back(msg->data[i]);
        }

        _fun_simple.local_grid_map.size = true;
    }

    void reference_path_callback(
        const autopilot_msgs::RoutePath::ConstPtr& msg
    ){
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
            _fun_simple.local_reference_path.push_back(rp);
            ROS_INFO("(x : %f, y : %f, angle : %f)", rp.x, rp.y, rp.angle);
        }

        _fun_simple.local_reference_path.size = true;
        ROS_INFO("Finished get the global path");
    }

    void motion_goal_callback(
        const autopilot_msgs::RouteNode::ConstPtr& msg
    ){
        _fun_simple.goal_point.x = msg -> x;
        _fun_simple.goal_point.y = msg -> y;

        _fun_simple.goal_point.size = true;

    }

    void vehicle_odometry_callback(
        const nav_msgs::Odometry::ConstPtr& msg
    ){
        _fun_simple.vehicle_loc.x = msg -> pose.pose.position.x;
        _fun_simple.vehicle_loc.y = msg -> pose.pose.position.y;

        tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 matrix3x3(quat);
        double roll, pitch, yaw;
        matrix3x3.getRPY(roll, pitch, yaw);

        _fun_simple.vehicle_loc.angle = yaw;

        _fun_simple.vehicle_vel.vx = msg -> twist.twist.linear.x;
        _fun_simple.vehicle_vel.vy = msg -> twist.twist.linear.y;
        _fun_simple.vehicle_vel.vz = msg -> twist.twist.linear.z;

        _fun_simple.vehicle_loc.size = true;
        _fun_simple.vehicle_vel.size = true;
    }

    void motion_speed_callback(
        const std_msgs::Float64::ConstPtr& msg
    ){
        _fun_simple.speed.speed = msg -> data;

        _fun_simple.speed.size = true;
    }
};