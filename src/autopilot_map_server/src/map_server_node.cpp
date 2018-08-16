#include <ros/ros.h>
#include <autopilot_msgs/RouteNode.h>
#include <autopilot_msgs/RouteMap.h>
#include <autopilot_msgs/WGS2Map.h>
#include <autopilot_msgs/Map2WGS.h>
#include <autopilot_msgs/Location.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <visualization_msgs/MarkerArray.h>
#include "Geography/GaussLocalGeographicCS.hpp"

namespace autopilot_map_server {

    autopilot_msgs::RouteMap::Ptr routeMapPtr;

    autopilot_msgs::RouteNode::Ptr originPtr;
    std::shared_ptr<GaussLocalGeographicCS> gaussProjectorPtr;

    std::shared_ptr<ros::Publisher> areaDetectionPublisherPtr;
    std::shared_ptr<ros::Publisher> visualizationPublisherPtr;

    bool isInArea(const autopilot_msgs::Location &location, const autopilot_msgs::RouteArea area) {
        unsigned int count = 0;
        for (size_t i = 0; i < area.nodes.size() - 1; i++) {
            autopilot_msgs::RouteNode node1 = area.nodes[i];
            autopilot_msgs::RouteNode node2 = area.nodes[i + 1];
            double lat1 = node1.latitude, lon1 = node1.longitude;
            double lat2 = node2.latitude, lon2 = node2.longitude;
            if (std::abs(lat1 - lat2) > DBL_EPSILON &&
                std::min(lat1, lat2) <= location.latitude && location.latitude <= std::max(lat1, lat2)) {
                if (location.longitude > lon1 - ((lon1 - lon2) * (lat1 - location.latitude)) / (lat1 - lat2)) {
                    count++;
                }
            }
        }
        return count % 2 == 1;
    }

    autopilot_msgs::WGS2Map::Response ll2xy(const autopilot_msgs::WGS2Map::Request &request) {
        assert(gaussProjectorPtr);
        autopilot_msgs::WGS2Map::Response result;
        double x, y, z;
        gaussProjectorPtr->llh2xyz(request.latitude, request.longitude, 0, x, y, z);
        result.x = static_cast<float>(x);
        result.y = static_cast<float>(y);
        return result;
    }

    autopilot_msgs::Map2WGS::Response xy2ll(const autopilot_msgs::Map2WGS::Request &request) {
        assert(gaussProjectorPtr);
        autopilot_msgs::Map2WGS::Response result;
        double z;
        gaussProjectorPtr->xyz2llh(request.x, request.y, 0, result.latitude, result.longitude, z);
        return result;
    }

    template<class T>
    T getTagValue(const boost::property_tree::ptree &tree, const std::string &key) {
        for (auto const &v: tree) {
            auto &child = *((boost::property_tree::ptree::value_type *) &v);
            if (child.first == "tag" && child.second.get<std::string>("<xmlattr>.k") == key) {
                return child.second.get<T>("<xmlattr>.v");
            }
        }
        throw std::invalid_argument("No such tag: k=" + key);
    }

    void visualization() {
        visualization_msgs::MarkerArray markerArray;

        for (auto const &edge: routeMapPtr->edges) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "route_map";
            marker.id = static_cast<int>(markerArray.markers.size());

            marker.type = visualization_msgs::Marker::ARROW;

            geometry_msgs::Point startPoint;
            startPoint.x = edge.begin.x;
            startPoint.y = edge.begin.y;

            geometry_msgs::Point endPoint;
            endPoint.x = edge.end.x;
            endPoint.y = edge.end.y;
            marker.points.insert(marker.points.end(), startPoint);
            marker.points.insert(marker.points.end(), endPoint);

            double length = std::sqrt(std::pow(startPoint.x - endPoint.x, 2) + std::pow(startPoint.y - endPoint.y, 2));

            marker.scale.x = length / 10.0;
            marker.scale.y = length / 5.0;

            marker.color.g = 1.0;
            marker.color.a = 1.0;

            markerArray.markers.insert(markerArray.markers.end(), marker);
        }

        for (auto const &area:routeMapPtr->areas) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "route_map";
            marker.id = static_cast<int>(markerArray.markers.size());

            marker.type = visualization_msgs::Marker::LINE_STRIP;

            float centroidX = 0, centroidY = 0, minY = 0, maxY = 0;
            for (auto const &node:area.nodes) {
                geometry_msgs::Point point;
                point.x = node.x;
                point.y = node.y;
                marker.points.insert(marker.points.end(), point);
                centroidX += node.x;
                centroidY += node.y;
                minY = std::min(minY, node.y);
                maxY = std::max(maxY, node.y);
            }
            centroidX /= area.nodes.size(), centroidY /= area.nodes.size();

            marker.scale.x = 2.0;

            marker.color.r = 1.0;
            marker.color.a = 1.0;

            markerArray.markers.insert(markerArray.markers.end(), marker);

            visualization_msgs::Marker textMarker;
            textMarker.header.stamp = ros::Time::now();
            textMarker.header.frame_id = "map";
            textMarker.ns = "route_map";
            textMarker.id = static_cast<int>(markerArray.markers.size());

            textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            textMarker.scale.z = (maxY - minY) / 5;
            textMarker.pose.position.x = centroidX;
            textMarker.pose.position.y = centroidY;
            textMarker.color.r = 1.0;
            textMarker.color.a = 1.0;
            textMarker.text = area.type;
            markerArray.markers.insert(markerArray.markers.end(), textMarker);
        }

        visualizationPublisherPtr->publish(markerArray);
    }

    void
    loadRouteMap(const std::string &path, const ros::Publisher &publisher) {
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(path, pt);

        auto &routeMap = *(routeMapPtr = autopilot_msgs::RouteMapPtr(new autopilot_msgs::RouteMap()));
        std::map<std::string, autopilot_msgs::RouteNode> nodeMap;

        for (auto const &v : pt.get_child("osm")) {
            auto &child = *((boost::property_tree::ptree::value_type *) &v);
            std::string tag = child.first;
            auto id = child.second.get<std::string>("<xmlattr>.id", "");
            if (tag == "node") {
                if (id.empty()) {
                    ROS_ERROR("Route Mapping Parsing: ID is empty in a Node");
                    throw std::invalid_argument("Route Mapping Parsing: ID is empty in a Node");
                }
                ROS_INFO("Route Mapping Parsing Node: ID %s", id.c_str());

                autopilot_msgs::RouteNode node;
                node.latitude = child.second.get<double>("<xmlattr>.lat");
                node.longitude = child.second.get<double>("<xmlattr>.lon");

                auto isOrigin = false;
                try {
                    isOrigin = getTagValue<bool>(child.second, "origin");
                } catch (const std::invalid_argument &ignored) {}

                if (isOrigin) {
                    // 设置地图原点，作为经纬度参考点，注意默认地图 x 轴方向为正北，不做额外的旋转变换
                    node.x = 0;
                    node.y = 0;
                    originPtr = autopilot_msgs::RouteNode::Ptr(new autopilot_msgs::RouteNode(node));
                    gaussProjectorPtr = std::make_shared<GaussLocalGeographicCS>(
                            originPtr->latitude, originPtr->longitude);
                } else {
                    autopilot_msgs::WGS2Map::Request request;
                    request.latitude = node.latitude;
                    request.longitude = node.longitude;
                    node.x = ll2xy(request).x;
                    node.y = ll2xy(request).y;
                }

                nodeMap[id] = node;

            } else if (tag == "way") {
                ROS_INFO("Route Mapping Parsing Way: ID %s", id.c_str());
                std::vector<autopilot_msgs::RouteEdge> wayEdges;
                std::vector<autopilot_msgs::RouteNode> wayNodes;
                for (auto const &vWay: child.second) {
                    auto &wayChild = *((boost::property_tree::ptree::value_type *) &vWay);
                    if (wayChild.first == "nd") {
                        auto refNodeId = wayChild.second.get<std::string>("<xmlattr>.ref");
                        if (nodeMap.find(refNodeId) == nodeMap.end()) {
                            ROS_WARN("Route Mapping Parsing: Ref Node %s not Found!", refNodeId.c_str());
                            continue;
                        }
                        autopilot_msgs::RouteNode currentNode = nodeMap[refNodeId];
                        autopilot_msgs::RouteEdge currentEdge;
                        currentEdge.begin = currentNode;
                        wayEdges.insert(wayEdges.end(), currentEdge);
                        wayNodes.insert(wayNodes.end(), currentNode);
                        if (wayEdges.size() > 1) {
                            auto &lastEdge = wayEdges[wayEdges.size() - 2];
                            lastEdge.end = currentNode;
                            ROS_INFO("Route Mapping Parsing Way: Add Edge, from (%.06f, %.06f) to (%.06f, %.06f)",
                                     lastEdge.begin.latitude, lastEdge.begin.longitude,
                                     lastEdge.end.latitude, lastEdge.end.longitude
                            );
                        }
                    }
                }
                if (wayEdges.empty()) {
                    ROS_WARN("Empty Way? ID: %s", id.c_str());
                    continue;
                }
                // 去除最后一个无效边
                wayEdges.pop_back();

                if (wayNodes.front().latitude == wayNodes.back().latitude
                    && wayNodes.front().longitude == wayNodes.back().longitude
                    && wayNodes.front().x == wayNodes.back().x
                    && wayNodes.front().y == wayNodes.back().y) {
                    // 首尾相连是 Area
                    autopilot_msgs::RouteArea area;
                    area.id = static_cast<unsigned short>(routeMap.areas.size());
                    area.type = getTagValue<std::string>(child.second, "area_type");
                    area.nodes = wayNodes;
                    routeMap.areas.insert(routeMap.areas.end(), area);
                } else {
                    // 将该道路的边添加到路网的边集
                    routeMap.edges.insert(routeMap.edges.end(), wayEdges.begin(), wayEdges.end());
                }


            } else if (tag == "relation") {
                // 如果边联通的话，好像不需要 relation
            }
        }

        if (!originPtr) {
            ROS_ERROR("Origin Node not Found!");
            throw std::invalid_argument("Origin Node not Found!");
        }


        routeMap.header.frame_id = "map";
        routeMap.header.stamp = ros::Time::now();
        routeMap.origin = *originPtr;
        publisher.publish(routeMap);

        visualization();
    }

    bool wgs2MapServiceHandler(autopilot_msgs::WGS2Map::Request &request, autopilot_msgs::WGS2Map::Response &response) {
        response = ll2xy(request);
        return true;
    }

    bool map2WgsServiceHandler(autopilot_msgs::Map2WGS::Request &request, autopilot_msgs::Map2WGS::Response &response) {
        response = xy2ll(request);
        return true;
    }

    void onLocationUpdateCallback(const autopilot_msgs::Location::ConstPtr &msg) {
        autopilot_msgs::RouteArea currentArea;
        currentArea.id = 65535;
        currentArea.type = "none";

        for (auto const &area : routeMapPtr->areas) {
            if (isInArea(*msg, area)) {
                currentArea = area;
                break;
            }
        }

        areaDetectionPublisherPtr->publish(currentArea);
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "autopilot_map_server");
    ros::NodeHandle private_nh("~");

    auto flag = private_nh.param<bool>("debug_flag", false);
    while (flag && ros::ok()) {
        // wait for debugger attaching.
        usleep(100 * 1000);
    }

    auto mapPath = private_nh.param<std::string>("map_path", "");
    //auto mapPath = private_nh.param<std::string>("map_path", "");
    if (mapPath.empty()) {
        ROS_FATAL("Empty Map Path");
        exit(-1);
    }

    auto gridMapYamlPath = mapPath + ".yaml";
    auto routeMapOsmPath = mapPath + ".osm";

    ros::NodeHandle nh;

    auto routeMapPublisher = nh.advertise<autopilot_msgs::RouteMap>("/map/route_map", 1, true);

    autopilot_map_server::visualizationPublisherPtr = std::make_shared<ros::Publisher>();
    *autopilot_map_server::visualizationPublisherPtr =
            nh.advertise<visualization_msgs::MarkerArray>("/map/route_map_visualization", 1, true);

    autopilot_map_server::loadRouteMap(routeMapOsmPath, routeMapPublisher);

    auto wgs2MapService = nh.advertiseService("/map/wgs_to_map", autopilot_map_server::wgs2MapServiceHandler);
    auto map2WgsService = nh.advertiseService("/map/map_to_wgs", autopilot_map_server::map2WgsServiceHandler);

    autopilot_map_server::areaDetectionPublisherPtr = std::make_shared<ros::Publisher>();
    *autopilot_map_server::areaDetectionPublisherPtr =
            nh.advertise<autopilot_msgs::RouteArea>("/map/area_detection", 100);
    auto locationSubscriber = nh.subscribe("/localization/location", 100,
                                           autopilot_map_server::onLocationUpdateCallback);

    ros::AsyncSpinner spinner(0); // a thread for each CPU core.
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
