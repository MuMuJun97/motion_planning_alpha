
#include "MotionPlanner.hpp"

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
#include <std_msgs/Float64.h>
#include <fstream>
#include <iostream>
#include <cmath>

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

    autopilot_msgs::WayPoints generate_msg_to_controller(
        std::vector<type_road_point> selected_path, double speed
    ){
        autopilot_msgs::WayPoints waypoints_msg;

        for ( int i = 0; i < selected_path.size(); i++ )
        {
            //TODO transform map coordination to WGS coordination.
            GaussLocalGeographicCS gausslocalgeographiccs = 
                GaussLocalGeographicCS(22.9886565512, 113.2691559583);
            
            double _ ;
            gausslocalgeographiccs.xyz2llh(
                selected_path.at(i).x,
                selected_path.at(i).y,
                0,
                selected_path.at(i).latitude,
                selected_path.at(i).longitude,
                _
            );

            // ROS_INFO(
            //         "Got WG2: (%.10f, %.10f) from Map: (%f, %f)", 
            //         p_fun_main->selected_path.at(i).latitude, 
            //         p_fun_main->selected_path.at(i).longitude,
            //         p_fun_main->selected_path.at(i).x, 
            //         p_fun_main->selected_path.at(i).y);

            autopilot_msgs::RouteNode routenode;

            routenode.latitude =
                selected_path.at(i).latitude;
            routenode.longitude =
                selected_path.at(i).longitude;
            routenode.x =
                selected_path.at(i).x;
            routenode.y =
                selected_path.at(i).y;
            
            waypoints_msg.points.push_back( routenode );
            waypoints_msg.speeds.push_back( speed );
        }

        return waypoints_msg;
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
    :_nh(*nodehandle), _fun_simple(p_fun_main)
    {
        initializeSubscribers();
    }
private:
    ros::NodeHandle _nh;
    fun_simple* _fun_simple;
    std::vector<std::pair<string,ros::Subscriber>> _subs;

    void initializeSubscribers()
    {
        ros::Subscriber sub_0 = _nh.subscribe(
            "/grid_test/obstacle_grid_map", 10, &Listener::grid_map_callback, this
        );
        _subs.push_back( make_pair( "/grid_test/obstacle_grid_map", sub_0 ) );

        ros::Subscriber sub_1 = _nh.subscribe(
            "/route/path", 10, &Listener::reference_path_callback, this
        );
        _subs.push_back( make_pair( "/route/path", sub_1 ) );

        ros::Subscriber sub_2 = _nh.subscribe(
            "player_odometry", 10, &Listener::vehicle_odometry_callback, this
        );
        _subs.push_back( make_pair( "player_odometry", sub_2 ) );

        ros::Subscriber sub_3 = _nh.subscribe(
            "motion/goal", 10, &Listener::motion_goal_callback, this
        );
        _subs.push_back( make_pair( "motion/goal", sub_3 ) );

        ros::Subscriber sub_4 = _nh.subscribe(
            "motion/speed", 10, &Listener::motion_speed_callback, this
        );
        _subs.push_back( make_pair( "motion/speed", sub_4 ) );

    }

    void grid_map_callback(
        const nav_msgs::OccupancyGrid::ConstPtr& msg
    ){
        _fun_simple -> local_grid_map.width = msg->info.width;
        _fun_simple -> local_grid_map.height = msg->info.height;
        _fun_simple -> local_grid_map.resolution = msg->info.resolution;
        for(int i=0; i< msg->info.width * msg->info.height; i++)
        {
            _fun_simple -> local_grid_map.data.push_back(msg->data[i]);
        }

        _fun_simple -> local_grid_map.size = true;
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
            _fun_simple -> local_reference_path.push_back(rp);
            ROS_INFO("(x : %f, y : %f, angle : %f)", rp.x, rp.y, rp.angle);
        }

        ROS_INFO("Finished get the global path");
    }

    void motion_goal_callback(
        const autopilot_msgs::RouteNode::ConstPtr& msg
    ){
        ROS_INFO("Starting get the motion goal");
        _fun_simple -> goal_point.x = msg -> x;
        _fun_simple -> goal_point.y = msg -> y;

        _fun_simple -> goal_point.size = true;
        ROS_INFO("Finished get the motion goal");

    }

    void vehicle_odometry_callback(
        const nav_msgs::Odometry::ConstPtr& msg
    ){
        _fun_simple -> vehicle_loc.x = msg -> pose.pose.position.x;
        _fun_simple -> vehicle_loc.y = fabs( msg -> pose.pose.position.y );

        tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 matrix3x3(quat);
        double roll, pitch, yaw;
        matrix3x3.getRPY(roll, pitch, yaw);

        _fun_simple -> vehicle_loc.angle = yaw;

        _fun_simple -> vehicle_vel.vx = msg -> twist.twist.linear.x;
        _fun_simple -> vehicle_vel.vy = msg -> twist.twist.linear.y;
        _fun_simple -> vehicle_vel.vz = msg -> twist.twist.linear.z;

        _fun_simple -> vehicle_loc.size = true;
        _fun_simple -> vehicle_vel.size = true;
    }

    void motion_speed_callback(
        const std_msgs::Float64::ConstPtr& msg
    ){
        ROS_INFO("Starting get the motion speed");
        _fun_simple -> speed.speed = msg -> data;

        _fun_simple -> speed.size = true;

        ROS_INFO("Finished get the global path");
    }
};


int main(int argc, char** argv)
{
    //TODO initialize ROS Node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    //TODO initialize the motion planner
    MotionPlanner planner;

    //TODO subscribe demanded topic
    Listener listener(&nh, planner.p_fun_main);

    //TODO initialize publisher
    Talker talker(&nh);

    ROS_INFO("Motion Planning Node is Running...");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    while ( ros::ok() )
    {
        if ( planner.run_once() )
        {
            continue;
        }

        if ( planner.p_fun_main -> selected_path.size() <= 0 )
        {
            continue;
        }

        talker.get_publisher( "/planner/way_points" ).publish( 
            talker.generate_msg_to_controller( 
                planner.p_fun_main->selected_path, 
                planner.p_fun_main->speed.speed ) 
        );
        
        ROS_INFO("Finished publish way points");

        planner.record_process( "../ros_ws/motion_planning_alpha/logs/" );

        planner.clear_storages();
    }

    return 0;
}
