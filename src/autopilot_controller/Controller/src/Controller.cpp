#include <iostream>

#include "TCPClient.hpp"
#include "math_util.h"
#include "Controller.hpp"
#include "origin_vehicle.h"

#define __ACTIVE_ENABLE_ACTUATOR__

Controller::Controller(const std::string &inifilename, const std::string &controller_name,
                       const int controller_code) :

        runtimeParameters(inifilename), controller_name(controller_name), controller_code(
        controller_code), curr_path(&recv_path[0]), next_path(&recv_path[1]), g_lcm_can(
        "udpm://239.255.76.63:7660?ttl=3"), g_lcm_feedback(
        "udpm://239.255.76.63:7662?ttl=3"), g_lcm_vehicle(
        "udpm://239.255.76.63:7663?ttl=3"), g_lcm_steering(
        "udpm://239.255.76.63:7664?ttl=3"){

    Calculate_controller_signature();
    l_a = 0.0;
    l_b = 0.0;

    e = 0.0;
    last_e = 0.0;
    match_point_no = -100;
    xla = 0.0;
    ela = 0.0;
    delta_phi = 0.0;
    delta_phi_radian = 0.0;
    K = 0.0;

    Kff = 0.0;

    d_feedback_control = 0.0;
    Kp = 0.0;

    d_integral_control = 0.0;
    Ki = 0.0;
    ival_steering = 0.0;
    ival_steering_limit = 0.0;

    st_damping_control = 0.0;
    Kd = 0.0;

    Cf = 0.0;
    Cr = 0.0;

    Wf = 0.0, Wr = 0.0;
    m = 0.0;
    L = 0.0;
    d_feedforward_control = 0.0;

    kug = 0.0;
    g = G_acc;
    Rs = 0.0;

    ay_ = 0.0;

    thita = 0.0;
    delta = 0.0;
    r = 0.0;

    beta = 0.0;
    beta_radian = 0.0;

    d_control = 0.0;

    diff_speed = 0.0;
    last_diff_speed = 0.0;

    desired_Ux = 0.0;

    auto_driver = true;

    // steering filter window
    WINDOW_LEN = 0;
    LAST_WEIGHT = 0.0;

    // longitudinal_brake_controller
    kupitch = 0.0;
    ival_upitch = 0.0;

    Kp_brake = 0.0;
    Ki_brake = 0.0;
    Kaccd_brake = 0.0;
    aggressive_offset_brake = 0.0;
    regular_offset_brake = 0.0;

    ival_brake = 0.0;
    ival_brake_upper_limit = 0.0;
    ival_brake_lower_limit = 0.0;

    // longitudinal_speed_controller
    Kp_acc = 0.0;
    Ki_acc = 0.0;
    Kd_acc = 0.0;
    Kaccd_acc = 0.0;
    offset_acc = 0.0;

    ival_acc = 0.0;
    ival_acc_upper_limit = 0.0;
    ival_acc_lower_limit = 0.0;

    output_limit_acc = 0.0;

    // 画底图控制点的步长
    draw_step_points = runtimeParameters.draw_step_points;

    // 人机接口控制
    enablePyPlot = false;

    enableInsHandlerLogger = false;
    enableLateralLog = false;
    enableLongitudinalLog = false;
    enablePathExtractLog = false;

    enableTrajectoryLog = false;

    preprocessingStatus = false;

#ifdef __ACTIVE_ENABLE_ACTUATOR__
    if (runtimeParameters.actuator_port != 0)
        writeTCPMsg2Sock(runtimeParameters.actuator_ip, runtimeParameters.actuator_port,
                         runtimeParameters.actuator_start_msg);
#endif
}

Controller::~Controller() {
    lateral_none();
    longitudinal_none();

    interface_close();
    cancel_pthreads();

#ifdef __ACTIVE_ENABLE_ACTUATOR__
    if (runtimeParameters.actuator_port != 0) {
        writeTCPMsg2Sock(runtimeParameters.actuator_ip, runtimeParameters.actuator_port,
                         runtimeParameters.actuator_stop_msg);
    }
#endif
}

void Controller::reset() {
    lateral_none();
    longitudinal_none();

    e = 0.0;   //地图匹配的最近点的距离
    last_e = 0.0;
    match_point_no = -100; //地图匹配的最近点的序号

    ival_steering = 0.0;
    ival_brake = 0.0;
    ival_acc = 0.0;

    steering_deque.assign(WINDOW_LEN, 0.0);

    actual_path.clear();
}

int32_t Controller::Calculate_controller_signature() {
    return controller_signature = (controller_code << 24) + (ins_pkg_no & 0x00ffffff);
}
