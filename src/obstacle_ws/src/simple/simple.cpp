#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

using namespace grid_map;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class OccupancyGridNode{

  ros::Publisher obstacle_grid_map;

public:
  OccupancyGridNode(ros::NodeHandle& nh,
                    const std::string& grid_map_topic,
                    const bool& latch = false){
    obstacle_grid_map = nh.advertise< nav_msgs::OccupancyGrid>(grid_map_topic, 1, latch);
  }

  void grid_call_back(const pcl::PointCloud<pcl::PointXYZ>& cloud){
    int points_size = cloud.size();
    std::cout<<"cloud_size: "<<points_size<<std::endl;
    // Create grid map.
    GridMap map({"obstacle"});
    map.setFrameId("lidar_0");
    // length(x_direction,y_direction), resolution
    map.setGeometry(Length(80, 20), 0.20);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
    ros::Time time = ros::Time::now();

    // pick up all nodes in the specific scope.
    for(int i=0; i<points_size; i++){
        float x_x = cloud[i].x;
        float y_y = cloud[i].y;
        float z_z = cloud[i].z;
        if(fabs(x_x) <= 40 && fabs(y_y) <= 10 && z_z < 0.5 && z_z > -1.9){
            if( fabs(x_x) > 3 || fabs(y_y) > 1.1  )
            {
              Position position(x_x, y_y);
              map.atPosition("obstacle", position) = 1;
            }
        }  
    }
    nav_msgs::OccupancyGrid result;
    GridMapRosConverter::toOccupancyGrid(map,"obstacle",0,100,result);
    obstacle_grid_map.publish(result);

  }


};


int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  std::string input_topic = "/lidar_0";
  std::string output_topic = "obstacle_grid_map";
  ros::Subscriber sub;
  OccupancyGridNode girdNode(nh, output_topic, false);
  sub = nh.subscribe(input_topic, 1, &OccupancyGridNode::grid_call_back, &girdNode);

  // ros::Publisher publisher = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
  // PointCloudT temp;
  // nav_msgs::OccupancyGrid result;
  // Position position;


  

  // Work with grid mainp in a loop.
  // ros::Rate rate(30.0);
  // while (nh.ok()) {

  //   // Add data to grid map.
    
  //   for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
  //     Position position;
  //     map.getPosition(*it, position);
  //     map.at("obstacle", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
  //   }

  //   // Publish grid map.
  //   map.setTimestamp(time.toNSec());
  //   grid_map_msgs::GridMap message;
  //   //GridMapRosConverter::toMessage(map, message);
  //   GridMapRosConverter::toOccupancyGrid(map,"obstacle",0,100,result);
  //   publisher.publish(result);
  //   ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

  //   // Wait for next cycle.
  //   rate.sleep();
  // }
  ros::spin();

  return 0;
}