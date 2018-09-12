#include <cmath>

#include "Controller.hpp"
#include "math_util.h"

#define __ENABLE_LONGITUDINAL_CONTROLLER__
#define __ENABLE_LONGITUDINAL_OUTPUT__

// 控制强制失效的距离
constexpr double ERROR_DISTANCE_LIMIT_ = (1.0);
constexpr double HEADING_RATE_LIMIT_ = (500.0);

using namespace std;

void Controller::longitudinal_none() {
    obu_lcm::accelerate_control_info accelerate_msg;
    obu_lcm::brake_control_info brake_msg;

    accelerate_msg.accelerate_percentage = 0.0;
    accelerate_msg.signature = controller_signature;

    brake_msg.brake_percentage = 100.0;
    brake_msg.signature = controller_signature;

#ifdef __ENABLE_LONGITUDINAL_CONTROLLER__
    g_lcm_vehicle.publish("accelerate_control_info", &accelerate_msg);
    g_lcm_vehicle.publish("brake_control_info", &brake_msg);
    // ROS
    std_msgs::Float64 temp_throttle;
    temp_throttle.data = accelerate_msg.accelerate_percentage;
    steering_pub.publish(temp_throttle);
    std_msgs::Float64 temp_brake;
    temp_brake.data = brake_msg.brake_percentage;
    steering_pub.publish(temp_brake);
#endif
}

// enable = 0（不刹车）/ 1（正常刹车） / -1 （稍微多刹一点） / -2 （直接刹死）
void Controller::longitudinal_brake_controller(const int enable, const double cur_speed,
                                               const double tar_speed, const double diff_speed, const double diff_a,
                                               obu_lcm::brake_control_info &brake_msg) {
    if (enable == 0) {
        ival_brake = 0.0;
        brake_msg.brake_percentage = 0.0;
        return;
    } else if (enable == -2) { /*处理急刹车*/
        brake_msg.brake_percentage = 100.0;
        return;
    }

    ival_brake += diff_speed;

    if (ival_brake > ival_brake_upper_limit)
        ival_brake = ival_brake_upper_limit;
    else if (ival_brake < ival_brake_lower_limit)
        ival_brake = ival_brake_lower_limit;

    const double offset = (
            (enable == -1 || tar_speed <= 1.0) ? aggressive_offset_brake : regular_offset_brake);

    brake_msg.brake_percentage = Kp_brake * diff_speed + Ki_brake * ival_brake + offset;

    if (brake_msg.brake_percentage > 100.0)
        brake_msg.brake_percentage = 100.0;

    brake_msg.brake_percentage += Kaccd_brake * (diff_a < 0.0 ? diff_a : 0.0);
    if (cur_speed < 9.0 && tar_speed > 1.0)
        brake_msg.brake_percentage *= (cur_speed / 9.0);

    if (brake_msg.brake_percentage < 0.0)
        brake_msg.brake_percentage = 0.0;

    if (cur_speed < 3.6 && tar_speed <= 1.0)
        brake_msg.brake_percentage = 100.0;

    brake_msg.signature = controller_signature;
}

double Controller::longitudinal_integral_upitch() {
    // 考虑到了坡道情况下重力的影响
    // 上坡时Current_pitch为+，tar_speed加上一个值，下坡时Current_pitch为-，tar_speed减去一个值
    const double upitch = kupitch * 0.2 * g * 3.6 * sin(to_radians(Current_pitch));

    // 如果upitch与ival_upitch符号相反则积分值置0
    if (upitch * ival_upitch < -0.001)
        ival_upitch = 0.0;
    else if (fabs(Current_pitch) > 0.5)
        ival_upitch += upitch;

    if (ival_upitch > 10.0)
        ival_upitch = 10.0;
    else if (ival_upitch < -3.0)
        ival_upitch = -3.0;

    return ival_upitch;
}

void Controller::longitudinal_speed_controller(double tar_speed, const double speed_limit) {
    obu_lcm::accelerate_control_info accelerate_msg;
    obu_lcm::brake_control_info brake_msg;

    int brake_enable = 1;

    if (tar_speed > speed_limit)
        tar_speed = speed_limit;
    else if (tar_speed < 0.0)
        tar_speed = 0.0;

    if (fabs(e) > ERROR_DISTANCE_LIMIT_) {
#ifdef    __ENABLE_LONGITUDINAL_OUTPUT__
        printf("Error distance is too much far, brake instantly!!!\n");
#endif
        tar_speed = 0.0f;
    } else
        tar_speed *= (1.0 - fabs(e) / ERROR_DISTANCE_LIMIT_);

    if (tar_speed >= 5.0)
        tar_speed += longitudinal_integral_upitch();
    else
        ival_upitch = 0.0;

    const double cur_speed = car_speed;
    //目标加速度，根据匀加速直线运动的速度和路程公式，可以推出在s=8m内达到速度的最合适的加速度
    //a_tar = (Utar^2 - Current_Speed^2) / (2 * s)
    const double Utar = tar_speed / 3.6;
    const double a_tar = (Utar + Current_Speed) * (Utar - Current_Speed) * 0.0625;

    ControlStatus.cur_speed = cur_speed;
    ControlStatus.tar_speed = tar_speed;

    double diff_a;
    if (a_tar > 0.0)
        diff_a =
                a_tar > Current_Acceleration_Longitudinal ?
                0.0 : Current_Acceleration_Longitudinal - a_tar;
    else
        diff_a =
                a_tar < Current_Acceleration_Longitudinal ?
                0.0 : Current_Acceleration_Longitudinal - a_tar;

    diff_speed = tar_speed - cur_speed;

    if (diff_speed > -5.0 && tar_speed > 0.1) {
        if (cur_speed > -0.5 && cur_speed < 0.5) {
            //不做处理
        } else if (diff_speed < 0.0)
            ival_acc = ival_acc + diff_speed * 10;
        else
            ival_acc = ival_acc + diff_speed;

        //limit ival
        if (ival_acc > ival_acc_upper_limit)
            ival_acc = ival_acc_upper_limit;
        else if (ival_acc < ival_acc_lower_limit)
            ival_acc = ival_acc_lower_limit;

        accelerate_msg.accelerate_percentage = Kp_acc * diff_speed + Ki_acc * ival_acc + offset_acc
                                               + Kd_acc * (diff_speed - last_diff_speed);

        // 不刹车
        brake_enable = 0;
    } else {
        ival_acc = 0.0;
        accelerate_msg.accelerate_percentage = offset_acc;
        // 刹车
        brake_enable = 1;
    }

//	if (Current_heading_speed < -HEADING_RATE_LIMIT_
//			|| Current_heading_speed > HEADING_RATE_LIMIT_) {
//#ifdef	__ENABLE_LONGITUDINAL_OUTPUT__
//		printf("Dangerous heading angular velocity, slow down.\n");
//#endif
//		ival_acc = 0.0;
//		accelerate_msg.accelerate_percentage = offset_acc;
//		// 稍稍刹车多一点
//		brake_enable = -1;
//	}

    if (accelerate_msg.accelerate_percentage > output_limit_acc)
        accelerate_msg.accelerate_percentage = output_limit_acc;
    else if (accelerate_msg.accelerate_percentage < 0.0)
        accelerate_msg.accelerate_percentage = 0.0;

    accelerate_msg.accelerate_percentage += Kaccd_acc * (diff_a > 0.0 ? diff_a : 0.0);
    accelerate_msg.signature = controller_signature;

    longitudinal_brake_controller(brake_enable, cur_speed, tar_speed, -diff_speed, diff_a,
                                  brake_msg);

    ControlStatus.accelerate_percentage = accelerate_msg.accelerate_percentage;
    ControlStatus.brake_percentage = brake_msg.brake_percentage;

#ifdef  __ENABLE_LONGITUDINAL_OUTPUT__
    printf(
            "Ucur=%2.2lf,Utar=%2.2lf,P=%2.2lf,I=%2.2lf,D=%2.2lf,ax=%2.2lf,atar=%2.2lf,T=%.2lf,B=%2.2lf\n",
            cur_speed, tar_speed, Kp_acc * diff_speed, Ki_acc * ival_acc,
            Kd_acc * (diff_speed - last_diff_speed), Current_Acceleration_Longitudinal, a_tar,
            accelerate_msg.accelerate_percentage, brake_msg.brake_percentage);
#endif

#ifdef __ENABLE_LONGITUDINAL_CONTROLLER__
    g_lcm_vehicle.publish("accelerate_control_info", &accelerate_msg);
    g_lcm_vehicle.publish("brake_control_info", &brake_msg);
    // ROS
    std_msgs::Float64 temp_throttle;
    temp_throttle.data = accelerate_msg.accelerate_percentage;
    steering_pub.publish(temp_throttle);
    std_msgs::Float64 temp_brake;
    temp_brake.data = brake_msg.brake_percentage;
    steering_pub.publish(temp_brake);
#endif
}

void Controller::longitudinal_control_output(const Path &p) {
//	constexpr double tarspeeds[] = { 20.0, 40.0, 60.0, 40.0, 20.0, 0.0 };
//	const int idx = ((int) CurrentS / 200) % (sizeof(tarspeeds) / sizeof(double));
//	longitudinal_speed_controller(tarspeeds[idx], 80.0);
//	longitudinal_speed_controller(40.0, 60.0);
    longitudinal_speed_controller(speed_desired_Uxs_C, 40.0);
}
