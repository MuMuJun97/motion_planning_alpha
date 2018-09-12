#include "ECU.hpp"
#include "origin_vehicle.h"
#include "math_util.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <imutransform.hpp>

Geography::CSTransfer<GaussLocalGeographicCS> ECU::csTransfer;

ECU::ECU() {
    ins_pkg_no = -1;
    // 车辆的坐标
    Current_lon = 0.0, Current_lat = 0.0, Current_height = 0.0;  //经纬度坐标
    CurrentX = 0.0, CurrentY = 0.0, CurrentZ = 0.0;   //局部坐标
    CurrentS = 0.0;  //行驶里程

    Yaw_C = 0.0, speed_C = 0.0;
    position_x_C = 0.0, position_y_C = 0.0, speed_desired_Uxs_C = 0.0;
    steering_angle_C = 0.0;

    Current_yaw = 0.0; // 头指向和北方向。
    Current_roll = 0.0;
    Current_pitch = 0.0;

    Current_Speed = 0.0;
    Current_Speed_Lateral = 0.0;
    Current_Speed_Longitudinal = 0.0;
    Current_Speed_Down = 0.0;

    Current_roll_speed = 0.0;
    Current_pitch_speed = 0.0;
    Current_heading_speed = 0.0;

    Current_Acceleration_Pattern = 0.0; //加速度模式
    Current_Acceleration = 0.0; //加速度
    Current_Acceleration_Lateral = 0.0;
    Current_Acceleration_Longitudinal = 0.0;

    // CAN总线上的车辆传感器信息
    wheel_speed_FL = 0.0;
    wheel_speed_FR = 0.0;
    wheel_speed_BL = 0.0;
    wheel_speed_BR = 0.0;

    car_speed = 0.0;
    steer_angle = 0.0;
    steer_angle_speed = 0.0;
    engine_RPM = 0.0;

    at_status = 0.0;
    flashing_status = 0.0;
    engine_status = 0.0;
    brake_status = 0.0;
    hand_brake_status = 0.0;
    wheel_steering_angle = 0.0;

    // 车辆的基本参数
    wheel_base = origin_vehicle::WHEEL_BASE;
    front_track = origin_vehicle::LENGTH_A;
    front_wheel_wide = 1;
    back_wheel_wide = 1;
    front_wheel_direction = Current_yaw + steer_angle / origin_vehicle::STEERING_R;

    Current_gears = 0.0;                //档位
    Driving_status = 0.0;                //人工驾驶或自动驾驶，或者网络驾驶

    auto_driver = true;
    can_start_auto = 1;
    can_stop_auto = 0;
}

void ECU::set_current_ins_info(const autopilot_msgs::MotionState::Ptr &motionState) {
    ins_pkg_no = motionState->header.seq;

    NaviPoint nd_AX7;

    this->Current_lat = motionState->gps.latitude;
    this->Current_lon = motionState->gps.longitude;
    this->Current_height = motionState->gps.altitude;

    csTransfer.ll2xy(this->Current_lat, this->Current_lon, this->CurrentX, this->CurrentY);
    nd_AX7.position_x = this->CurrentX;
    nd_AX7.position_y = this->CurrentY;
    nd_AX7.position_z = this->CurrentZ = this->Current_height;

    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(motionState->imu.orientation, quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    this->Current_yaw = nd_AX7.heading = normalizeDegree(to_degrees(-yaw));
    this->Current_pitch = to_degrees(pitch);
    this->Current_roll = to_degrees(roll);

    this->Current_roll_speed = to_degrees(motionState->imu.angular_velocity.x);
    this->Current_pitch_speed = to_degrees(motionState->imu.angular_velocity.y);
    this->Current_heading_speed = to_degrees(motionState->imu.angular_velocity.z);

    this->Current_Speed_Lateral = motionState->odom.twist.twist.linear.y;
    this->Current_Speed_Longitudinal = motionState->odom.twist.twist.linear.x;
    this->Current_Speed_Down = motionState->odom.twist.twist.linear.z;

    this->Current_Speed = nd_AX7.speed_desired_Uxs = sqrt(
            Current_Speed_Lateral * Current_Speed_Lateral
            + Current_Speed_Longitudinal * Current_Speed_Longitudinal
            + Current_Speed_Down * Current_Speed_Down);

//  lateral_accelerate +右-左
    this->Current_Acceleration_Lateral = motionState->imu.linear_acceleration.x;
    this->Current_Acceleration_Longitudinal = motionState->imu.linear_acceleration.y;
    this->Current_Acceleration = hypot(Current_Acceleration_Lateral,
                                       Current_Acceleration_Longitudinal);

//记录当前的车辆行驶轨迹
    if (actual_path.empty()) {
        nd_AX7.s = 0.0;
        actual_path.push_back(nd_AX7);
    } else {
        double l = length_two_points(actual_path.back().position_x, actual_path.back().position_y,
                                     nd_AX7.position_x, nd_AX7.position_y);
        nd_AX7.s = actual_path.back().s + l;

        if (l > 1.0) {
            if (nd_AX7.s - actual_path.front().s >= origin_vehicle::MAP_BUFFER_PADDING_S)
                actual_path.pop_front();

            actual_path.push_back(nd_AX7);
        }
    }
    this->CurrentS = nd_AX7.s;
}
