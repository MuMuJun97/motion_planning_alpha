#include <iostream>
#include <utility>

#include "Controller.hpp"
#include "origin_vehicle.h"
#include "cau_heading_steering.h"

void Controller::handle_ins_msg(const autopilot_msgs::MotionState::Ptr &motionState) {
    if (!curr_path || !curr_path->size()) {
        return;
    }
    ROS_INFO("ROS INS Message Received");
    if (enableInsHandlerLogger) {
        timeval ins_gen_tv;
        ins_gen_tv.tv_sec = static_cast<__time_t >(motionState->header.stamp.sec);
        ins_gen_tv.tv_usec = static_cast<__suseconds_t >(motionState->header.stamp.nsec * 1000);
        mytimer::extractHHMMSSUS(&ins_gen_tv, ins_gen_t);

        mytimer::getHHMMSSUS(ins_t0);
    }

    set_current_ins_info(motionState);
    // 本应从CAN总线获取总体车速，这里使用IMU数据代替
    car_speed = Current_Speed * 3.6;
    
    if (enableInsHandlerLogger)
        mytimer::getHHMMSSUS(ins_t1);

    Controller_preprocessing(curr_path);


    if (enableInsHandlerLogger)
        mytimer::getHHMMSSUS(ins_t2);

    Controller_apply();
    interface_apply();
}

void Controller::handle_planner_nav_control_points_msg(const autopilot_msgs::WayPoints::Ptr &wayPoints) {
    if (enablePathExtractLog)
        mytimer::getHHMMSSUS(path_t0);

    if (can_start_auto == 0 && can_stop_auto == 1)
        return;

    if (wayPoints->points.size() != wayPoints->speeds.size()) {
        ROS_ERROR("The size of points does not match for that of speeds");
        return;
    }
    // 验证 way points 有效性
    for (auto const &point : wayPoints->points) {
        if (!point.longitude || !point.latitude) {
            ROS_ERROR("Way Point Invalid!!! lat: %f, lon: %f", point.latitude, point.longitude);
            return;
        }
    }
    if (wayPoints->points.size() < 0 || wayPoints->points.size() > 2000) {
        std::cerr << "Number of nav control points out of range!!! " << std::endl;
        std::cerr << "received " << wayPoints->points.size() << " points" << std::endl;
        return;
    } else if (wayPoints->points.size() == 1 && wayPoints->speeds.front() < 1.0) {
        // 规划层停车信号
        for (int i = 0; i < curr_path->size(); ++i)
            curr_path->ref_points[i].speed_desired_Uxs = 0.0;
    } else {
        extract_path_from_msg(wayPoints, next_path);
        if (enablePathExtractLog)
            mytimer::getHHMMSSUS(path_t1);
        int no_points = cau_all_output_from_single_spline_realtime(*next_path, next_path->size());
        if (no_points < 10)
            // return;
        if (enablePathExtractLog)
            mytimer::getHHMMSSUS(path_t2);
        switch_path_buffer();
        if (enablePathExtractLog)
            mytimer::getHHMMSSUS(path_t3);
        if (enablePathExtractLog) {
            timeval path_gen_tv;
            path_gen_tv.tv_sec = static_cast<__time_t >(wayPoints->header.stamp.sec);
            path_gen_tv.tv_usec = static_cast<__suseconds_t >(wayPoints->header.stamp.nsec * 1000);
            mytimer::extractHHMMSSUS(&path_gen_tv, path_gen_t);

            pathExtractLogger.log("%s %s %s %s %s\n", path_gen_t, path_t0, path_t1, path_t2, path_t3);
        }
    }
}

void Controller::handle_CAN_value_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                      const obu_lcm::CAN_value *msg) {
    car_speed = msg->car_speed;
    engine_RPM = msg->engine_RPM;

    wheel_speed_FL = msg->wheel_speed_FL;
    wheel_speed_FR = msg->wheel_speed_FR;
    wheel_speed_BL = msg->wheel_speed_BL;
    wheel_speed_BR = msg->wheel_speed_BR;
}

void Controller::handle_CAN_status_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                       const obu_lcm::CAN_status *msg) {
    at_status = msg->at_status;
    flashing_status = msg->flashing_status;
    engine_status = msg->engine_status;
    brake_status = msg->brake_status;
    hand_brake_status = msg->hand_brake_status;
}

void Controller::handle_steering_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf,
                                                   const std::string &chan,
                                                   const obu_lcm::steering_feedback_info *msg) {
    steer_angle = -msg->steering_angle;
    steer_angle_speed = msg->steering_angle_speed;

//	g_AX7_controller->wheel_steering_angle = g_AX7_controller->steer_angle / 15;

    if (steer_angle < 0)
        wheel_steering_angle = steer_angle / origin_vehicle::LEFT_STEERING_RATIO;
    else
        wheel_steering_angle = steer_angle / origin_vehicle::RIGHT_STEERING_RATIO;
}

void Controller::handle_accelerate_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf,
                                                     const std::string &chan,
                                                     const obu_lcm::accelerate_feedback_info *msg) {

}

void Controller::handle_brake_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf,
                                                const std::string &chan, const obu_lcm::brake_feedback_info *msg) {

}

void Controller::handle_gears_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf,
                                                const std::string &chan, const obu_lcm::gears_feedback_info *msg) {

}

int Controller::init_lcm() {
    /////////////////////////ins接收////////////////////////////////////
    ROS_INFO("Subscribe ROS INS MESSAGE");
    //subscriber = rosNodeHandle.subscribe("/localization/motion_state", 1, &Controller::handle_ins_msg, this);
    insSubscriber = rosNodeHandle.subscribe(runtimeParameters.ins_topic_name, 100, &Controller::handle_ins_msg, this);

    /////////////////////////车量接收////////////////////////////////////
    if (!g_lcm_can.good())
        return -1;
    if (!g_lcm_feedback.good())
        return -1;

    ////////////////////CAN 控制数据发送/////////////////////////////////
    if (!g_lcm_steering.good())
        return -1;

    if (!g_lcm_vehicle.good())
        return -1;

    ///////////////////规划数据接收/////////////////////////////////
    wayPointsSubscriber = rosNodeHandle.subscribe(runtimeParameters.way_points_topic_name, 10,
                                                  &Controller::handle_planner_nav_control_points_msg, this);

    ///////////////////控制数据发送/////////////////////////////////
    throttle_pub = rosNodeHandle.advertise<std_msgs::Float64>("player_throttle", 10);
    brake_pub = rosNodeHandle.advertise<std_msgs::Float64>("player_brake", 10);
    steering_pub = rosNodeHandle.advertise<std_msgs::Float64MultiArray>("player_steering", 10);

    ////////////////////CAN 控制数据发送/////////////////////////////////
    g_lcm_can.subscribe("CAN_value", &Controller::handle_CAN_value_lcm, this);
    g_lcm_can.subscribe("CAN_status", &Controller::handle_CAN_status_lcm, this);

    g_lcm_feedback.subscribe("steering_feedback_info",
                             &Controller::handle_steering_feedback_info_lcm, this);
    g_lcm_feedback.subscribe("accelerate_feedback_info",
                             &Controller::handle_accelerate_feedback_info_lcm, this);
    g_lcm_feedback.subscribe("brake_feedback_info", &Controller::handle_brake_feedback_info_lcm,
                             this);
    g_lcm_feedback.subscribe("gears_feedback_info", &Controller::handle_gears_feedback_info_lcm,
                             this);

    return 0;
}
