#ifndef _CONTROLLER_HPP__
#define _CONTROLLER_HPP__

#include <ros/ros.h>
#include <autopilot_msgs/MotionState.h>
#include <autopilot_msgs/WayPoints.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>
#include <cstdint>

#include <pthread.h>
#include <netinet/in.h>

#include "ControllerRuntimeParameters.hpp"
#include "ECU.hpp"
#include "MyLogger.hpp"
#include "mytimer.hpp"
#include "MapMatching.hpp"
#include "Path.hpp"

// lcm headers
#include "ins_info.hpp"
#include "nav_control_points.hpp"
#include "nav_points.hpp"
#include "CAN_status.hpp"
#include "CAN_value.hpp"
#include "accelerate_control_info.hpp"
#include "accelerate_feedback_info.hpp"
#include "brake_control_info.hpp"
#include "brake_feedback_info.hpp"
#include "gears_control_info.hpp"
#include "gears_feedback_info.hpp"
#include "steering_control_info.hpp"
#include "steering_feedback_info.hpp"

class Controller : public ECU {
protected:
    const ControllerRuntimeParameters runtimeParameters;
    int controller_code;
    std::string controller_name;
    int32_t controller_signature;

    //  论文中所需要的各种控制参数
    double l_a; //前轴到重心的距离
    double l_b; //后轴到重心的距离
    double e;   //地图匹配的最近点的距离
    double last_e;
    int match_point_no; //地图匹配的最近点的序号
    double xla; //预瞄点距离
    double ela; //前向预测
    double delta_phi; //预描点角度
    double delta_phi_radian;
    double K;   // 当前轨迹的曲率

    double Kff; // K_feedforword

    double d_feedback_control;  //feedback control value
    double Kp;  //k_feedback

    double Ki;  // integral control, 消除静差
    double ival_steering;
    double ival_steering_limit;

    double d_integral_control;

    double st_damping_control;
    double Kd; //K_damping;

    // 轮胎断面宽190mm  轮胎内圈16英寸   轮胎外圈
    double Cf; //前轮刚度
    double Cr; //后轮刚度

    double Wf, Wr; // 前后轮称重
    double m; //车重 公斤
    double L; //轴距
    double d_feedforward_control;

    double kug; //K_feedforward;
    double g;   //重力加速度
    double Rs;   //转动半径   方向盘转角和轴距的 实时半径

    double ay_;  //纵向控制中根据加速度公式反推的理论加速度
    double r;    //yaw rate 转向角加速度
    double thita;  //俯仰角
    double delta;  //车轮转角 取绝对值

    double beta;  //vehicle sideslip
    double beta_radian;

    double d_control;

protected:
    void Calculate_delta_phi_by_table(double Current_Yaw_, double Yaw_C, double *p_delta_phi_,
                                      double *p_delta_phi_radian);

    void Calculate_beta();

    void Calculate_RS();

    void Calculate_K();

    int Calculate_e_sign(double x, double y, double yaw, double x_c, double y_c, double yaw_c);

    double Calculate_e(double x, double y, double yaw, double x_c, double y_c, double yaw_c);

    void Calculate_Mapping_parameters(const Path &p);

    void Calculate_control_parameters();

    int32_t Calculate_controller_signature();

    bool preprocessingStatus;

public:
    Path recv_path[2];
    // 控制器的输入 用于输入的buffer
    Path *curr_path, *next_path;

    void load_default_path();

    int extract_path_from_msg(const autopilot_msgs::WayPoints::Ptr &wayPoints, Path *target_path_ptr);

    void switch_path_buffer();

    MapMatching m_matching;

    bool Controller_preprocessing(const Path *p);

    void Controller_apply();

public:
    // 画底图控制点的步长
    unsigned int draw_step_points;

    void Draw_Best_March_Point(const Path &p);

    // socket struct
    bool enablePyPlot;

    int PyPlotSockfd;
    struct sockaddr_in PyPlotServaddr;
    struct {
        double lateral_ff = 0.0f;
        double lateral_P = 0.0f;
        double lateral_I = 0.0f;
        double lateral_Damping = 0.0f;
        double steering = 0.0f;
        double e = 0.0f;
        double cur_speed = 0.0f;
        double tar_speed = 0.0f;
        double accelerate_percentage = 0.0f;
        double brake_percentage = 0.0f;
    } ControlStatus;

    bool enableInsHandlerLogger;
    bool enableLateralLog;
    bool enableLongitudinalLog;
    bool enablePathExtractLog;

    bool enableTrajectoryLog;

    MyLogger insHandlerLogger;
    MyLogger lateralLogger;
    MyLogger longitudinalLogger;
    MyLogger pathExtractLogger;

    MyLogger trajectoryLogger;

    mytimer::HHMMSSUSCStr ins_gen_t, ins_t0, ins_t1, ins_t2;
    mytimer::HHMMSSUSCStr lateral_t0, lateral_t1;
    mytimer::HHMMSSUSCStr longitudinal_t0, longitudinal_t1;
    mytimer::HHMMSSUSCStr path_gen_t, path_t0, path_t1, path_t2, path_t3;

    void interface_init();

    void interface_close();

    void interface_apply();

// 横向控制算法
protected:
    void lateral_damping_control(double);

    void lateral_feedback_control();

    void lateral_integral_control();

    void lateral_feedforward_control();

    // steering filter window
    int WINDOW_LEN;
    double LAST_WEIGHT;
    std::deque<double> steering_deque;

    void lateral_lowpass(double *steering_angle);

public:
    void lateral_none();

    void lateral_control_output(const Path &p);

//纵向控制算法
protected:
    // 速度偏差
    double diff_speed;
    double last_diff_speed;

    ///纵向控制参数
    //总纵向力输出
    double desired_Ux;

    double kupitch;
    double ival_upitch;

    // 对pitch进行积分，处理上下坡的情况
    double longitudinal_integral_upitch();

    double Kp_brake;
    double Ki_brake;
    double Kaccd_brake;
    double aggressive_offset_brake, regular_offset_brake;
    double ival_brake;
    double ival_brake_upper_limit, ival_brake_lower_limit;

    void longitudinal_brake_controller(const int enable, const double cur_speed,
                                       const double tar_speed, const double diff_speed, const double diff_a,
                                       obu_lcm::brake_control_info &brake_msg);

    double Kp_acc;
    double Ki_acc;
    double Kd_acc;
    double Kaccd_acc;
    double offset_acc;

    double ival_acc;
    double ival_acc_upper_limit, ival_acc_lower_limit;

    double output_limit_acc;

    void longitudinal_speed_controller(double tar_speed, const double speed_limit);

public:
    void longitudinal_none();

    void longitudinal_control_output(const Path &p);

protected:
    ros::NodeHandle rosNodeHandle;
    ros::Subscriber insSubscriber;
    ros::Subscriber wayPointsSubscriber;

    ros::Publisher throttle_pub;
    ros::Publisher brake_pub;
    ros::Publisher steering_pub;
    // used for input
    lcm::LCM g_lcm_can;
    lcm::LCM g_lcm_feedback;
    // used for output
    lcm::LCM g_lcm_steering;
    lcm::LCM g_lcm_vehicle;

    // lcm handlers
    void handle_ins_msg(const autopilot_msgs::MotionState::Ptr &motionState);

    void handle_CAN_value_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                              const obu_lcm::CAN_value *msg);

    void handle_CAN_status_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                               const obu_lcm::CAN_status *msg);

    void handle_steering_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                           const obu_lcm::steering_feedback_info *msg);

    void handle_accelerate_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf,
                                             const std::string &chan, const obu_lcm::accelerate_feedback_info *msg);

    void handle_brake_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                        const obu_lcm::brake_feedback_info *msg);

    void handle_gears_feedback_info_lcm(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                        const obu_lcm::gears_feedback_info *msg);

    void handle_planner_nav_control_points_msg(const autopilot_msgs::WayPoints::Ptr& wayPoints);

    int init_lcm();

    pthread_mutex_t controlMutex;
    pthread_cond_t controlWaitCond;

    // used for lcm
    pthread_t can_th, planner_th, feedback_th;
    // used for other purposes
    pthread_t control_sub_th;

    // used for lcm
    static void *ThreadFunc_can(void *param);

    static void *ThreadFunc_feedback(void *param);

    // used for other purposes
    static void *ThreadFunc_control_subThread(void *param);

    void create_pthreads();

    void cancel_pthreads();

public:
    void reset();

    Controller(const std::string &inifilename, const std::string &controller_name,
               const int controller_code);

    virtual ~Controller();

    std::string get_controller_name() const {
        return controller_name;
    }

    int get_controller_code() const {
        return controller_code;
    }

    double get_e() const {
        return e;
    }
};

#endif  /*_CONTROLLER_HPP__*/
