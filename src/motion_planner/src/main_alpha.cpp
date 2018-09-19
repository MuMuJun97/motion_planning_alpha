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

using namespace func_simplified;


int main(int argc, char** argv)
{
    //TODO initialize ROS Node
    ros::init(argc,argv,"motion_planner");
    ros::NodeHandle nh;

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

    //TODO subscribe demanded topic
    Listener listener = Listener(&nh, p_fun_main);

    //TODO initialize publisher
    Talker talker = Talker(&nh);

    MotionPlanner motion_planner = MotionPlanner();
    motion_planner.run();

    return 0;
}

class MotionPlanner
{
public:
    MotionPlanner()
    {
        
    }
    void run();
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
    Listener(ros::NodeHandle* nodehandle,fun_simple* p_fun_simple)
    :_nh(*nodehandle), _fun_simple(*p_fun_simple)
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
        ROS_INFO("Finished get the global path");
    }

    void motion_goal_callback(
        const autopilot_msgs::RouteNode::ConstPtr& msg
    ){
        _fun_simple.goal_point.x = msg -> x;
        _fun_simple.goal_point.y = msg -> y;

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
    }
};