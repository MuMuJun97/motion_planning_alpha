#include <iostream>
#include "AX7controller.hpp"
#include "origin_vehicle.h"

AX7controller::AX7controller(const std::string &inifilename, const std::string &controller_name,
                             const int controller_code) :
        Controller(inifilename, controller_name, controller_code) {
    l_a = origin_vehicle::LENGTH_A; //前轴到重心的距离
    l_b = origin_vehicle::LENGTH_B; //后轴到重心的距离

    xla = 10.0; //预瞄点距离 10 8

    // 1.11 0.84 0.852 0.8535 0.851
    Kff = 0.8;

    //#define KP_RATIO 1.97 1.975 1.9743
#define KP_RATIO 2.2
    Kp = 35000 * KP_RATIO; // 8

    // 0.009 0.00857
    Ki = 0.0086;
    ival_steering_limit = 90.0;

    //K_damping 初始值0.00328 0.0033
    Kd = 0.003;

    // 轮胎断面宽190mm  轮胎内圈16英寸   轮胎外圈
    Cf = origin_vehicle::CORNERING_FRONT; //前轮刚度
    Cr = origin_vehicle::CORNERING_REAR; //后轮刚度

    m = origin_vehicle::CAR_WEIGHT; //车重
    L = origin_vehicle::WHEEL_BASE; //轴距

    desired_Ux = 15.0;

    auto_driver = true;

    // steering filter window
    WINDOW_LEN = 75;
    LAST_WEIGHT = 0.12;

    // longitudinal_brake_controller
    kupitch = 0.1;

    Kp_brake = 5.5;
    Ki_brake = 0.5;    //0.18 0.165
    Kaccd_brake = 10.0;
    aggressive_offset_brake = 32.0;
    regular_offset_brake = 12.0;

    ival_brake_upper_limit = 150.0;
    ival_brake_lower_limit = 0.0;

    // longitudinal_speed_controller
    Kp_acc = 3.35;    // 75 16 20
    Ki_acc = 0.80;    // 0.5356 1.5 0.7 0.9
    Ki_acc = 0.0;
    Kaccd_acc = -4.2;
    offset_acc = 15.0;

    ival_acc_upper_limit = 13.00;
    ival_acc_lower_limit = -13.0;

    output_limit_acc = 50.0;

    /* 是否启用PYTHON绘图 */
    enablePyPlot = true;

    /* 是否启用文件输出 */
//	enableInsHandlerLogger = true;
//	enableLateralLog = true;
//	enableLongitudinalLog = true;
//	enablePathExtractLog = true;

//	enableTrajectoryLog = true;

    // must be called at last
    steering_deque.assign(WINDOW_LEN, 0.0);
    interface_init();
    if (runtimeParameters.load_local_way_points_map) {
        load_default_path();
    }

    create_pthreads();
    if (init_lcm() != 0) {
        std::cerr << "initialize lcm failed" << std::endl;
    }
}

AX7controller::~AX7controller() {
}
