#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

// socket header
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <GL/glu.h>
#include <GL/glut.h>

#include "origin_vehicle.h"
#include "Controller.hpp"
#include "draw_Controller.h"

using namespace std;

void Controller::interface_init() {
    if (enableInsHandlerLogger) {
        insHandlerLogger.setLogFilePath(runtimeParameters.log_dir,
                                        controller_name + "_" + std::to_string(controller_code) +
                                        "_insHandlerLogger.txt");
    }
    if (enableLateralLog) {
        lateralLogger.setLogFilePath(runtimeParameters.log_dir,
                                     controller_name + "_" + std::to_string(controller_code) + "_lateralLog.txt");
    }
    if (enableLongitudinalLog) {
        longitudinalLogger.setLogFilePath(runtimeParameters.log_dir,
                                          controller_name + "_" + std::to_string(controller_code) +
                                          "_longitudinalLog.txt");
    }
    if (enablePathExtractLog) {
        pathExtractLogger.setLogFilePath(runtimeParameters.log_dir,
                                         controller_name + "_" + std::to_string(controller_code) +
                                         "_pathExtractLog.txt");
    }

    if (enableTrajectoryLog) {
        trajectoryLogger.setLogFilePath(runtimeParameters.log_dir,
                                        controller_name + "_" + std::to_string(controller_code) +
                                        "_trajectoryLogger.txt");
    }

    if (enablePyPlot) {
        assert(runtimeParameters.plotter_port != 0 && !runtimeParameters.plotter_ip.empty());

        PyPlotSockfd = socket(PF_INET, SOCK_DGRAM, 0);

        bzero(&PyPlotServaddr, sizeof(PyPlotServaddr));
        PyPlotServaddr.sin_family = AF_INET;
        PyPlotServaddr.sin_port = htons(runtimeParameters.plotter_port);
        PyPlotServaddr.sin_addr.s_addr = inet_addr(runtimeParameters.plotter_ip.c_str());
    }
}

void Controller::interface_close() {
    if (enablePyPlot) {
        close(PyPlotSockfd);
    }
}

void Controller::interface_apply() {
    if (enableInsHandlerLogger) {
        insHandlerLogger.log("%d %s %s %s %s\n", ins_pkg_no, ins_gen_t, ins_t0, ins_t1, ins_t2);
    }

    if (enableLateralLog) {
        lateralLogger.log("%d %.4lf %s %s %.2lf\n", ins_pkg_no, ControlStatus.e, lateral_t0,
                          lateral_t1, ControlStatus.steering);
    }

    if (enableTrajectoryLog) {
        // log e, position
        trajectoryLogger.log("%d %.4lf %.8lf %.8lf %.4lf %.4lf %.4lf", match_point_no, e,
                             Current_lat, Current_lon, CurrentX, CurrentY, CurrentS);
        // log lateral control
        trajectoryLogger.log(" %.2lf", ControlStatus.steering);
        // log longitudinal control
        trajectoryLogger.log(" %.2lf %.2lf %.2lf %.2lf", ControlStatus.tar_speed,
                             ControlStatus.cur_speed, ControlStatus.accelerate_percentage,
                             ControlStatus.brake_percentage);
        // log IMU
        trajectoryLogger.log(" %.4lf %.4lf", Current_yaw, Current_heading_speed);
        // log time
        mytimer::HHMMSSUSCStr log_t;
        mytimer::getHHMMSSUS(log_t);
        trajectoryLogger.log(" %s\n", log_t);
    }

    if (enablePyPlot) {
        sendto(PyPlotSockfd, reinterpret_cast<unsigned char *>(&ControlStatus),
               sizeof(ControlStatus), 0, reinterpret_cast<struct sockaddr *>(&PyPlotServaddr),
               sizeof(PyPlotServaddr));
    }
}

void Controller::Draw_Best_March_Point(const Path &p) {
    double x;
    double y;

//地图匹配只能用一次，不能很多地方都用。
    int no_points = m_matching.current_match_point_no;

    if (no_points <= 0 || no_points > p.size())
        return;

    y = p.ref_points[no_points].position_y;
    x = p.ref_points[no_points].position_x;

    double xx = x;
    double yy = y;

    glPointSize(8);
    glColor3d(1, 1, 1);
    glBegin(GL_POINTS);
    glVertex2f(xx, yy);
    glEnd();

    glColor3d(1, 1, 1);
    glBegin(GL_LINES);
    glVertex2f(xx - 1, yy);
    glVertex2f(xx + 1, yy);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(xx, yy - 1);
    glVertex2f(xx, yy + 1);
    glEnd();
}
