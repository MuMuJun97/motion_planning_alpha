#include <ros/ros.h>
#include <ros/package.h>
#include "ControllerRuntimeParameters.hpp"
#include <iostream>
#include <cassert>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

ControllerRuntimeParameters::ControllerRuntimeParameters(const std::string &inifilename) {
    std::string workpathvar = ros::package::getPath("autopilot_controller");
    if (workpathvar.empty()) {
        ROS_ERROR("Finding Working Path Failed");
        exit(-1);
    }
    size_t found_pos;

    boost::property_tree::ptree m_pt;
    boost::property_tree::ini_parser::read_ini(inifilename, m_pt);

    map_filename = m_pt.get<std::string>("map_filename", "");
    if ((found_pos = map_filename.find("$WORKPATH")) != std::string::npos)
        map_filename = map_filename.replace(found_pos, strlen("$WORKPATH"), workpathvar);

    log_dir = m_pt.get<std::string>("log_dir", "log/");
    if ((found_pos = log_dir.find("$WORKPATH")) != std::string::npos)
        log_dir = log_dir.replace(found_pos, strlen("$WORKPATH"), workpathvar);
    assert(log_dir.back() == '/');

    draw_step_points = m_pt.get<unsigned int>("draw_step_points", 0);

    plotter_ip = m_pt.get<std::string>("plotter_ip", "");
    plotter_port = m_pt.get<unsigned int>("plotter_port", 0);

    actuator_ip = m_pt.get<std::string>("actuator_ip", "");
    actuator_port = m_pt.get<unsigned int>("actuator_port", 0);

    actuator_start_msg = m_pt.get<std::string>("actuator_start_msg", "");
    actuator_stop_msg = m_pt.get<std::string>("actuator_stop_msg", "");

    ins_topic_name = m_pt.get<std::string>("ins_topic_name", "/localization/ins_raw");
    way_points_topic_name = m_pt.get<std::string>("way_points_topic_name", "/planner/way_points");

    load_local_way_points_map = m_pt.get<bool>("load_local_way_points_map", false);
}
