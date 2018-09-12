#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <string>
#include <cstring>
#include <signal.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "ControllerWatchdog.hpp"

using namespace std;

ControllerWatchdog *controllerWatchdog_ptr = nullptr;

void signalHandleFunc(int signal_num) {
    if (controllerWatchdog_ptr)
        controllerWatchdog_ptr->stop();
    else
        exit(0);
}

int main(int argc, char const *argv[]) {
    signal(SIGINT, signalHandleFunc);
    signal(SIGTERM, signalHandleFunc);

    std::string workpathvar = ros::package::getPath("autopilot_controller");
    if (workpathvar.empty()) {
        ROS_ERROR("Finding Working Path Failed");
        exit(-1);
    }

    boost::property_tree::ptree m_pt;
    boost::property_tree::ini_parser::read_ini(string(workpathvar) +
                                               "/tools/controllerWatchdog/controllerWatchdog.ini", m_pt);

    string instanceDir = m_pt.get<string>("instanceDir", "");
    size_t found_pos;
    if ((found_pos = instanceDir.find("$WORKPATH")) != std::string::npos)
        instanceDir = instanceDir.replace(found_pos, strlen("$WORKPATH"), workpathvar);

    string instanceName = m_pt.get<string>("instanceName", "");
    unsigned int listenPort = m_pt.get<unsigned int>("listenPort", 0);

    {
        controllerWatchdog_ptr = new ControllerWatchdog(instanceDir, instanceName, listenPort);
        controllerWatchdog_ptr->run();

        delete controllerWatchdog_ptr;
        controllerWatchdog_ptr = nullptr;
    }

    return 0;
}
