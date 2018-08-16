#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>

#include <autopilot_msgs/RouteNode.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <mutex>

#include <unordered_map>

#include "Geography/MecatorLocalGeographicCS.hpp"
#include <Geography/GaussLocalGeographicCS.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace autopilot_rtk_mapping {

    std::shared_ptr<ros::Publisher> publisherPtr;
    std::shared_ptr<ros::Publisher> intensityMapPublisherPtr;

    autopilot_msgs::RouteNodePtr originPtr;
    std::string projectorName;
    std::shared_ptr<LocalGeographicCSBase> projectorPtr;

    namespace occ_map {
        typedef std::vector<float> OccupancyGridMap;
    }

    occ_map::OccupancyGridMap occupancyMap;
    occ_map::OccupancyGridMap intensityMap;

    float resolution = 1;
    float regionOfInterest = 50.0;
    float freeOccupancyLogit = -0.7f;
    float occupiedOccupancyLogit = 2.9f;
    unsigned int mapSize = 4096;

    std::mutex mapMutex;
    std::mutex intensityMapMutex;

    namespace filter {
        float gridDownSamplingResolution = 0.1f;

        float gradientThreshold = 0.1;
        float heightThreshold = -1.5f;
        float maxHeight = 0.0;
        float searchRadius = 0.1;

        float minXAbs = 0.20;
        float minYAbs = 0.20;
    }

    void filterPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
                      double lidarX, double lidarY, double lidarZ);

    void messageCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloudPtr,
                         const sensor_msgs::NavSatFixConstPtr &navSatFixPtr,
                         const sensor_msgs::ImuConstPtr &imuPtr) {

        ros::WallTime startTime = ros::WallTime::now();

        if (navSatFixPtr->status.status != sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
            ROS_ERROR("GPS NOT FIXED");
            return;
        }

        // 通过 GPS 获取位置信息
        double latitude = navSatFixPtr->latitude;
        double longitude = navSatFixPtr->longitude;

        // 设置原点
        if (!originPtr) {
            originPtr = autopilot_msgs::RouteNodePtr(new autopilot_msgs::RouteNode());
            originPtr->latitude = latitude;
            originPtr->longitude = longitude;
            if (projectorName == "mecator") {
                projectorPtr = std::make_shared<MecatorLocalGeographicCS>(latitude, longitude);
            } else {
                projectorPtr = std::make_shared<GaussLocalGeographicCS>(latitude, longitude);
            }
            ROS_INFO("Origin %f, %f", autopilot_rtk_mapping::originPtr->latitude,
                     autopilot_rtk_mapping::originPtr->longitude);

        }

        // 高斯投影
        // OSM 需要简单的墨卡托投影
        double lidarX, lidarY, lidarZ;
        projectorPtr->llh2xyz(latitude, longitude, 0, lidarX, lidarY, lidarZ);

        // 通过 IMU 获取朝向姿态
        Eigen::Quaternion<double> quaternion(imuPtr->orientation.w,
                                             imuPtr->orientation.x,
                                             imuPtr->orientation.y,
                                             imuPtr->orientation.z);
        Eigen::Matrix3f rotationMatrix = quaternion.toRotationMatrix().cast<float>();
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        transformMatrix.block(0, 0, 3, 3) = rotationMatrix;
        transformMatrix(0, 3) = static_cast<float>(lidarX);
        transformMatrix(1, 3) = static_cast<float>(lidarY);

        pcl::PointCloud<pcl::PointXYZI>::Ptr currentPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointCloudPtr, *currentPointCloudPtr);


        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*currentPointCloudPtr, *transformedPointCloudPtr, transformMatrix);

        filterPoints(transformedPointCloudPtr, lidarX, lidarY, lidarZ);
        ros::WallDuration filterTimeCost = ros::WallTime::now() - startTime;
        //ROS_INFO("filtering time cost: %f ms", filterTimeCost.toSec() * 1000.0f);

        std::shared_ptr<occ_map::OccupancyGridMap> currentOccupancyMap = std::make_shared<occ_map::OccupancyGridMap>(
                mapSize * mapSize, 0);
        // 初始化雷达周围区域的栅格
        for (auto x = static_cast<int>(std::round((lidarX - regionOfInterest) / resolution));
             x <= static_cast<int>(std::round((lidarX + regionOfInterest) / resolution)); x++) {
            for (auto y = static_cast<int>(std::round((lidarY - regionOfInterest) / resolution));
                 y <= static_cast<int>(std::round((lidarY + regionOfInterest) / resolution)); y++) {

                if (std::abs(x) > mapSize / 2 || std::abs(y) > mapSize / 2) {
                    continue;
                }
                // 初始为自由网格
                (*currentOccupancyMap)[(x + mapSize / 2) * mapSize + y + mapSize / 2] = freeOccupancyLogit;
            }
        }
        filterTimeCost = ros::WallTime::now() - startTime - filterTimeCost;
        //ROS_INFO("occ grid init time cost: %f ms", filterTimeCost.toSec() * 1000.0f);
        intensityMapMutex.lock();
        {
            for (auto const &point: transformedPointCloudPtr->points) {
                if (std::abs(point.x - lidarX) >= regionOfInterest || std::abs(point.y - lidarY) >= regionOfInterest) {
                    continue;
                }

                int x = static_cast<int>(std::round(point.x / resolution));
                int y = static_cast<int>(std::round(point.y / resolution));
                if (std::abs(x) > mapSize / 2 || std::abs(y) > mapSize / 2) {
                    continue;
                }

                // 点落到的网格设为占用
                (*currentOccupancyMap)[(x + mapSize / 2) * mapSize + y + mapSize / 2] =
                        occupiedOccupancyLogit - freeOccupancyLogit;
                intensityMap[(x + mapSize / 2) * mapSize + y + mapSize / 2] = std::max(
                        intensityMap[(x + mapSize / 2) * mapSize + y + mapSize / 2], point.intensity);

            }
        }
        intensityMapMutex.unlock();
        filterTimeCost = ros::WallTime::now() - startTime - filterTimeCost;
        //ROS_INFO("occ grid update time cost: %f ms", filterTimeCost.toSec() * 1000.0f);
        // 叠加到总地图上
        mapMutex.lock();
        for (auto x = static_cast<int>(std::round((lidarX - regionOfInterest) / resolution));
             x <= static_cast<int>(std::round((lidarX + regionOfInterest) / resolution)); x++) {
            for (auto y = static_cast<int>(std::round((lidarY - regionOfInterest) / resolution));
                 y <= static_cast<int>(std::round((lidarY + regionOfInterest) / resolution)); y++) {
                if (std::abs(x) > mapSize / 2 || std::abs(y) > mapSize / 2) {
                    continue;
                }
                occupancyMap[(x + mapSize / 2) * mapSize + y + mapSize / 2] += (*currentOccupancyMap)[
                        (x + mapSize / 2) * mapSize + y + mapSize / 2];
            }
        }
        mapMutex.unlock();
        filterTimeCost = ros::WallTime::now() - startTime - filterTimeCost;
        //ROS_INFO("occ grid concat time cost: %f ms", filterTimeCost.toSec() * 1000.0f);

        // Time cost
        ros::WallDuration timeCost = ros::WallTime::now() - startTime;
        //ROS_INFO("Mapping time cost: %f ms", timeCost.toSec() * 1000.0f);
    }

    void filterPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
                      double lidarX, double lidarY, double lidarZ) {
        //ROS_INFO("pointcloud size %zu", pointCloud->size());
        pcl::VoxelGrid<pcl::PointXYZI> voxelGridFilter;
        voxelGridFilter.setInputCloud(pointCloud);
        voxelGridFilter.setLeafSize(filter::gridDownSamplingResolution,
                                    filter::gridDownSamplingResolution,
                                    filter::gridDownSamplingResolution);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampledPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
        voxelGridFilter.filter(*downSampledPointCloudPtr);
        *pointCloud = *downSampledPointCloudPtr;
        //ROS_INFO("pointcloud size %zu", pointCloud->size());

        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
        kdTree->setInputCloud(pointCloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());

        ros::WallTime startProcess = ros::WallTime::now();
        for (auto &searchPoint : *pointCloud) {
            if (std::isnan(searchPoint.x) || std::isnan(searchPoint.y) || std::isnan(searchPoint.z)) {
                continue;
            }
            if (std::abs(searchPoint.x) < filter::minXAbs && std::abs(searchPoint.y) < filter::minYAbs) {
                continue;
            }
            if (std::abs(searchPoint.x - lidarX) > regionOfInterest &&
                std::abs(searchPoint.y - lidarY) > regionOfInterest) {
                continue;
            }
            if (searchPoint.z >= filter::maxHeight) {
                continue;
            }
            filteredPointCloudPtr->insert(filteredPointCloudPtr->end(), searchPoint);

            /*
            std::vector<int> pointIndexInSearch;
            std::vector<float> pointSquaredDistanceInSearch;

            if (kdTree->radiusSearch(searchPoint, filter::searchRadius, pointIndexInSearch,
                                     pointSquaredDistanceInSearch) <= 0) {
                continue;
            }

            float gradientInX = 0;
            float gradientInY = 0;
            float gradientInZ = 0;
            for (int neighbourIndex : pointIndexInSearch) {
                pcl::PointXYZI neighbour = pointCloud->points[neighbourIndex];

                gradientInX += std::min((float) 50.0, std::abs(neighbour.x - searchPoint.x));
                gradientInY += std::min((float) 50.0, std::abs(neighbour.y - searchPoint.y));
                gradientInZ += std::min((float) 50.0, std::abs(neighbour.z - searchPoint.z));
            }
            gradientInX /= pointIndexInSearch.size();
            gradientInY /= pointIndexInSearch.size();
            gradientInZ /= pointIndexInSearch.size();

            if (gradientInZ > filter::gradientThreshold) {
                searchPoint.intensity = gradientInZ;
                filteredPointCloudPtr->insert(filteredPointCloudPtr->end(), searchPoint);
                continue;
            }
             */
        }
        ros::WallDuration filterTimeCost = ros::WallTime::now() - startProcess;
        //ROS_INFO("filter cost: %f", filterTimeCost.toSec() * 1000.0f);

        *pointCloud = *filteredPointCloudPtr;
    }

    void mapPublishTimerCallback(const ros::TimerEvent &timerEvent) {
        // 转成 ROS Msg
        nav_msgs::OccupancyGrid msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.info.width = msg.info.height = mapSize;
        msg.info.resolution = resolution;
        msg.info.map_load_time = ros::Time::now();
        msg.info.origin.orientation.w = 1.0;

        // 初始化为未知区域
        msg.data = std::vector<int8_t>(mapSize * mapSize, 50);

        mapMutex.lock();
        {
            for (int i = 0; i < occupancyMap.size(); i++) {
                double logitPower = std::pow(10.0, occupancyMap[i]);
                msg.data[i] = static_cast<int8_t>(100.0f * logitPower / (1 + logitPower));
            }
        }
        mapMutex.unlock();

        publisherPtr->publish(msg);

        nav_msgs::OccupancyGrid intensityMapMsg;
        intensityMapMsg.header = msg.header;
        intensityMapMsg.info = msg.info;
        intensityMapMsg.data = std::vector<int8_t>(mapSize * mapSize, 0);

        intensityMapMutex.lock();
        {
            // scaling
            float maxIntensity = 0, minIntensity = 0;
            for (const float &intensity : intensityMap) {
                maxIntensity = std::max(maxIntensity, intensity);
                minIntensity = std::min(minIntensity, intensity);
            }
            ROS_INFO("max intensity %f", maxIntensity);
            ROS_INFO("min intensity %f", minIntensity);
            float scale = 100 / (maxIntensity - minIntensity);
            for (int i = 0; i < intensityMap.size(); i++) {
                intensityMapMsg.data[i] = static_cast<int>((intensityMap[i] - minIntensity) * scale);
            }
        }
        intensityMapMutex.unlock();

        intensityMapPublisherPtr->publish(intensityMapMsg);
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "rtk_mapping");
    ros::NodeHandle private_nh("~");

    auto flag = private_nh.param<bool>("debug_flag", false);
    while (flag && ros::ok()) {
        // wait for debugger attaching.
        usleep(100 * 1000);
    }

    auto inputPointCloudTopicName = private_nh.param<std::string>("pointcloud_topic", "/velodyne_points");
    autopilot_rtk_mapping::resolution = private_nh.param<float>("resolution", 0.5f);
    autopilot_rtk_mapping::mapSize = static_cast<unsigned int>(private_nh.param<int>("map_size", 4096));
    autopilot_rtk_mapping::regionOfInterest = private_nh.param<float>("region_of_interest", 50.0f);
    autopilot_rtk_mapping::projectorName = private_nh.param<std::string>("projection", "gauss");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, inputPointCloudTopicName, 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> navSatSub(nh, "/gps/fix", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imuSub(nh, "/imu/data", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix, sensor_msgs::Imu> GnnsPointCloudPolicy;
    message_filters::Synchronizer<GnnsPointCloudPolicy> sync(GnnsPointCloudPolicy(10), pointCloudSub, navSatSub,
                                                             imuSub);
    sync.registerCallback(boost::bind(&autopilot_rtk_mapping::messageCallback, _1, _2, _3));

    autopilot_rtk_mapping::publisherPtr = std::make_shared<ros::Publisher>();
    *autopilot_rtk_mapping::publisherPtr = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    autopilot_rtk_mapping::intensityMapPublisherPtr = std::make_shared<ros::Publisher>();
    *autopilot_rtk_mapping::intensityMapPublisherPtr = nh.advertise<nav_msgs::OccupancyGrid>("/intensity_map", 1);

    ROS_INFO("Projection Method: %s", autopilot_rtk_mapping::projectorName.c_str());
    ROS_WARN("Make sure the RTK is always reliable!");

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
    auto mapPublishTimer = nh.createTimer(ros::Duration(2), autopilot_rtk_mapping::mapPublishTimerCallback);
#pragma clang diagnostic pop

    autopilot_rtk_mapping::occupancyMap = autopilot_rtk_mapping::occ_map::OccupancyGridMap(
            autopilot_rtk_mapping::mapSize * autopilot_rtk_mapping::mapSize, 0);
    autopilot_rtk_mapping::intensityMap = autopilot_rtk_mapping::occ_map::OccupancyGridMap(
            autopilot_rtk_mapping::mapSize * autopilot_rtk_mapping::mapSize, 0);

    ros::spin();

    return 0;
}
