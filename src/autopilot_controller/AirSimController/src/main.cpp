#include <iostream>

#include <GL/glut.h>
#include <GL/freeglut_ext.h>

#include <unistd.h>
#include <signal.h>

#include "origin_vehicle.h"
#include "draw_Controller.h"
#include "AirSimcontroller.hpp"

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
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandleFunc);
    signal(SIGTERM, signalHandleFunc);

    const char *const workpathvar = getenv("WORKPATH");
    assert(workpathvar);
    const string ini_path(string(workpathvar) + "/autopilot_controller/AirSimController/");
    origin_vehicle::loadCarPara(ini_path + "AirSim_parameters.ini");

    const char usage[] = " [-h] [-n controllerName] [-c controllerCode]\n"
            "\n"
            "  -h                help\n"
            "  -n controllerName name the controller\n"
            "  -c controllerCode give a certain controller code\n"
            "  no arguments      use default\n";

    string controller_name("AirSimController");
    int controller_code = 0;

    int opt;
    while ((opt = getopt(argc, argv, "hn:c:")) != -1) {
        switch (opt) {
            case 'h':
                cout << usage << endl;
                return 0;
                break;
            case 'n':
                controller_name = string(optarg);
                break;
            case 'c':
                controller_code = stoi(optarg);
                break;
            default:
                cerr << "usage: " << argv[0] << usage << endl;
                return 0;
        }
    }

#ifdef __ENABLE_GLUT_DRAW___
    glutInit(&argc, argv);
#endif

    Controller *g_AirSim_controller = new AirSimcontroller(ini_path + "AirSim_runtime.ini", controller_name,
                                                           controller_code);

#ifndef __ENABLE_GLUT_DRAW___
    runnigFlag = true;
#endif

#ifdef __ENABLE_GLUT_DRAW___
    MyGLDispIni(g_AirSim_controller, g_AirSim_controller->get_controller_name());
    glutMainLoop();
#else
    while (runnigFlag) {
        sleep(1);
    }
#endif

    if (g_AirSim_controller) {
        delete g_AirSim_controller;
        g_AirSim_controller = nullptr;
    }

    return 0;
}
