#ifndef _ECU_HPP__
#define _ECU_HPP__

#include <ros/ros.h>
#include <autopilot_msgs/MotionState.h>

#include <deque>
#include "Path.hpp"
#include "Geography.hpp"
#include "GaussLocalGeographicCS.hpp"
#include "MecatorLocalGeographicCS.hpp"
#include "ins_info.hpp"

struct ECU {
protected:
    static Geography::CSTransfer<GaussLocalGeographicCS> csTransfer;
public:
    bool auto_driver;
    int can_start_auto, can_stop_auto;

    // 车辆的基本参数
    double wheel_base;   // 轴距
    double front_track;
    double front_wheel_direction; //轮向
    double front_wheel_wide; //轮碾直径按0.5m算
    double back_wheel_wide;

    // CAN总线上的车辆传感器信息
    double wheel_speed_FL;
    double wheel_speed_FR;
    double wheel_speed_BL;
    double wheel_speed_BR;

    double car_speed;
    double steer_angle;
    double steer_angle_speed;
    double engine_RPM;

    int32_t at_status;
    int32_t flashing_status;
    int32_t engine_status;
    int32_t brake_status;
    int32_t hand_brake_status;

    double wheel_steering_angle;

    // 车辆的坐标原点
    int ins_pkg_no;
    double Current_lon, Current_lat, Current_height;  //经纬度坐标
    double CurrentX, CurrentY, CurrentZ;   //局部坐标
    double CurrentS;  //行驶里程

    double Yaw_C, position_x_C, position_y_C, speed_C;
    double speed_desired_Uxs_C;
    double steering_angle_C;

    double Current_yaw; // 头指向和北方向。
    double Current_roll;
    double Current_pitch;

    double Current_Speed;
    double Current_Speed_Lateral;
    double Current_Speed_Longitudinal;
    double Current_Speed_Down;

    double Current_roll_speed;
    double Current_pitch_speed;
    double Current_heading_speed;

    double Current_Acceleration_Pattern; //加速度模式
    double Current_Acceleration; //加速度
    double Current_Acceleration_Lateral;
    double Current_Acceleration_Longitudinal;

    int Current_gears;     //档位
    int Driving_status;     //人工驾驶或自动驾驶，或者网络驾驶

public:
    ECU();

    virtual ~ECU() {
    }

    void set_current_ins_info(const autopilot_msgs::MotionState::Ptr &motionState);

    virtual void Draw_Org();

    virtual void DrawCar(double x, double y, double yaw, double steer_angle);

    virtual void DrawCar_e(double x, double y, double x_c, double y_c, double yaw, double yaw_c);

    virtual void draw_road_path(const Path &p, int every, int point_size, float line_width, float r,
                                float g, float b);

    virtual void draw_road_lane(const Path &lane, double lane_width, int every, float line_width,
                                float r, float g, float b);

    virtual void DrawPath_ChangeLane();

    virtual void Draw_Virtual_lane(const Path &v_p, double lane_width, int every, float r = 0.0,
                                   float g = 1.0, float b = 0.0);

public:
    // 当前执行参考规划好的轨迹
    Path frame_path;
    // 记录当前车辆运行的轨迹
    std::deque<NaviPoint> actual_path;

};

#endif /*_ECU_HPP__*/
