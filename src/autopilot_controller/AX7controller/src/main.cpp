#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include <GL/glut.h>
#include <GL/freeglut_ext.h>

#include <unistd.h>
#include <signal.h>

#include "AX7controller.hpp"
#include "origin_vehicle.h"
#include "draw_Controller.h"

/* 是否启用GLUT绘图 */
#define __ENABLE_GLUT_DRAW___

using namespace std;

#ifndef __ENABLE_GLUT_DRAW___
volatile bool runnigFlag = false;
#endif

void signalHandleFunc(int signal_num) {
#ifdef __ENABLE_GLUT_DRAW___
    glutLeaveMainLoop();
#else
    runnigFlag = false;
#endif
    ros::shutdown();
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandleFunc);
    signal(SIGTERM, signalHandleFunc);

    ros::init(argc, argv, "autopilot_controller");
    ros::NodeHandle private_nh("~");

    std::string workpathvar = ros::package::getPath("autopilot_controller");
    if (workpathvar.empty()) {
        ROS_ERROR("Finding Working Path Failed");
        exit(-1);
    }
    const string ini_path(workpathvar + "/config");
    origin_vehicle::loadCarPara(ini_path + "/AX7_parameters.ini");

    string controller_name = private_nh.param<string>("controller_name", "AX7Controller");
    int controller_code = private_nh.param<int>("controller_code", 0);

#ifdef __ENABLE_GLUT_DRAW___
    glutInit(&argc, argv);
#endif

    Controller *g_AX7_controller = new AX7controller(ini_path + "/AX7_runtime.ini", controller_name,
                                                     controller_code);

#ifndef __ENABLE_GLUT_DRAW___
    runnigFlag = true;
#endif

    ROS_INFO("Init ROS Spinner");
#ifdef __ENABLE_GLUT_DRAW___
    ros::AsyncSpinner spinner(0); // a thread for each CPU core.
    spinner.start();
    MyGLDispIni(g_AX7_controller, g_AX7_controller->get_controller_name());
    glutMainLoop();
#else
    ros::AsyncSpinner spinner(0); // a thread for each CPU core.
    spinner.start();
    ros::waitForShutdown();
#endif

    ros::shutdown();

    if (g_AX7_controller) {
        delete g_AX7_controller;
        g_AX7_controller = nullptr;
    }

    return 0;
}
