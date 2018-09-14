#include <cstdio>
#include <cmath>

#include "steering_angle.h"
#include "math_util.h"
#include "origin_vehicle.h"
#include "Controller.hpp"

#define __ENABLE_LATERAL_CONTROLLER__
#define __ENABLE_LATERAL_OUTPUT__
//#define __USE_PID_CONTROLLRE__

using namespace std;

void Controller::lateral_none() {
    obu_lcm::steering_control_info steering_msg;
    steering_msg.steering_angle_speed = 11.0;
    steering_msg.steering_angle = 0.0;
    steering_msg.signature = controller_signature;

#ifdef __ENABLE_LATERAL_CONTROLLER__
    g_lcm_steering.publish("steering_control_info", &steering_msg);
    // ROS
    std_msgs::Float64MultiArray temp_steering;
    temp_steering.data.push_back(steering_msg.steering_angle);
    temp_steering.data.push_back(steering_msg.steering_angle_speed);
    steering_pub.publish(temp_steering);
#endif
}

void Controller::Calculate_delta_phi_by_table(double current_heading, double head_c,
                                              double *p_delta_phi, double *p_delta_phi_radian) {
    int index = static_cast<int>(current_heading / 90.0);
    int index_C = static_cast<int>(head_c / 90.0);

    if (abs(index - index_C) == 0)
        *p_delta_phi = current_heading - head_c;
    if (abs(index - index_C) == 1)
        *p_delta_phi = current_heading - head_c;
    if (index == 0 && index_C == 3)
        *p_delta_phi = current_heading - head_c + 360.0;
    if (index == 3 && index_C == 0)
        *p_delta_phi = current_heading - head_c - 360.0;

    if (abs(index - index_C) == 2) {
        if (index == 0 && index_C == 2) {
            if (fabs(current_heading - head_c) > 180.0)
                *p_delta_phi = -(current_heading + 360.0 - head_c);
            else
                *p_delta_phi = head_c - current_heading;
        }
        if (index == 2 && index_C == 0) {
            if (fabs(current_heading - head_c) > 180.0)
                *p_delta_phi = 360.0 + head_c - current_heading;
            else
                *p_delta_phi = head_c - current_heading;
        }
        if (index == 1 && index_C == 3) {
            if (fabs(current_heading - head_c) > 180.0)
                *p_delta_phi = -(current_heading + 360.0 - head_c);
            else
                *p_delta_phi = head_c - current_heading;
        }
        if (index == 3 && index_C == 1) {
            if (fabs(current_heading - head_c) > 180.0)
                *p_delta_phi = head_c + 360.0 - current_heading;
            else
                *p_delta_phi = -(current_heading - head_c);
        }
    } else
        *p_delta_phi = -*p_delta_phi;

    *p_delta_phi_radian = to_radians(*p_delta_phi);
}

void Controller::Calculate_beta() {
    double beta_temp;
    if (Current_Speed_Longitudinal <= 0.01)
        beta_temp = 0;
    else
        // -pi/2~pi/2
        beta_temp = atan(Current_Speed_Lateral / Current_Speed_Longitudinal);

    beta = to_degrees(beta_temp);
    beta_radian = beta_temp;
}

void Controller::Calculate_RS() {
    Rs = 10000.0;
}

void Controller::Calculate_K() {
    K = tan(to_radians(steering_angle_C)) / L;

#define K_LIMIT_ (30.0)
    if (K < -K_LIMIT_)
        K = -K_LIMIT_;
    else if (K > K_LIMIT_)
        K = K_LIMIT_;
}

void Controller::lateral_feedforward_control() {
    Wf = (l_b / L) * m * g;
    Wr = (l_a / L) * m * g;

    kug = Wf / Cf - Wr / Cr;

    d_feedforward_control = (L + kug * Current_Speed_Longitudinal * Current_Speed_Longitudinal / g)
                            / Rs;
}

void Controller::lateral_feedback_control() {
    ela = e + xla * sin(delta_phi_radian);
    d_feedback_control = 2.0 * (Kp / Cf) * ela;
}

void Controller::lateral_integral_control() {
    ival_steering += e;

    if (ival_steering > ival_steering_limit)
        ival_steering = ival_steering_limit;
    else if (ival_steering < -ival_steering_limit)
        ival_steering = -ival_steering_limit;

    d_integral_control = Ki * ival_steering;
}

void Controller::lateral_damping_control(const double fabs_steering) {
    r = Current_heading_speed;
//	Calculate_beta();
//	Calculate_K();

// d_delta_phi = r - Current_Speed_Longitudinal * K * (cos(delta_phi_radian) - tan(beta_radian) * sin(delta_phi_radian));
// d_damping_control = -Kd * d_delta_phi;
    double da_ratio = Kd * r * log2(Current_Speed + 1.0);
    // fabs_steering在30以内不适用比例的方式
    if (fabs_steering < 30.0) {
        st_damping_control = da_ratio * 500.0;
        if (fabs(st_damping_control) > fabs_steering - 3.0)
            st_damping_control = copysign(fabs_steering - 3.0, da_ratio);
        return;
    }

#define DA_RATIO_ (0.50)
    if (da_ratio > DA_RATIO_)
        da_ratio = DA_RATIO_;
    else if (da_ratio < -DA_RATIO_)
        da_ratio = -DA_RATIO_;

    st_damping_control = da_ratio * fabs_steering;
}

void Controller::lateral_lowpass(double *steering_angle) {
    double mean_steering = 0.0;

    steering_deque.pop_front();
    const double weight = (1.0 - LAST_WEIGHT) / static_cast<double>(WINDOW_LEN - 1);
    for (deque<double>::const_iterator it = steering_deque.cbegin(); it != steering_deque.cend();
         ++it)
        mean_steering += (*it * weight);

    steering_deque.push_back(*steering_angle);
    mean_steering += (steering_deque.back() * LAST_WEIGHT);

    *steering_angle = mean_steering;
}

#ifndef __USE_PID_CONTROLLRE__

void Controller::lateral_control_output(const Path &p) {
    lateral_feedback_control();
    lateral_integral_control();

    d_control = d_feedback_control + d_integral_control;

    // 参考轨迹一定大于20各点。最好为 200
    obu_lcm::steering_control_info steering_msg;

    if (d_control < 0)
        steering_msg.steering_angle = Kff * steering_angle_C
                                      + d_control * origin_vehicle::LEFT_STEERING_RATIO;
    else
        steering_msg.steering_angle = Kff * steering_angle_C * origin_vehicle::STEERING_RLR
                                      + d_control * origin_vehicle::RIGHT_STEERING_RATIO;

    lateral_damping_control(fabs(steering_msg.steering_angle));
    steering_msg.steering_angle += st_damping_control;

    // A mean filter to avoid oscillation
    lateral_lowpass(&(steering_msg.steering_angle));

    /// zd 将steering限制在-525~+561 －左+右
    if (steering_msg.steering_angle > origin_vehicle::MAX_STEERING_ANGLE)
        steering_msg.steering_angle = origin_vehicle::MAX_STEERING_ANGLE;
    else if (steering_msg.steering_angle < origin_vehicle::MIN_STEERING_ANGLE)
        steering_msg.steering_angle = origin_vehicle::MIN_STEERING_ANGLE;

    // 若总体速度大于10m/s则直接限制输出转向角
    if (Current_Speed > 10.0) {
        if (steering_msg.steering_angle > 0.0)
            steering_msg.steering_angle *= (320.0 - 4.0 * Current_Speed)
                                           / origin_vehicle::MAX_STEERING_ANGLE;
        else
            steering_msg.steering_angle *= (-320.0 + 4.0 * Current_Speed)
                                           / origin_vehicle::MIN_STEERING_ANGLE;
    }

    // 对参数赋值
    ControlStatus.lateral_ff = Kff * steering_angle_C;
    ControlStatus.lateral_P = d_feedback_control * origin_vehicle::STEERING_R;

    ControlStatus.lateral_I = d_integral_control * origin_vehicle::STEERING_R;
    ControlStatus.lateral_Damping = st_damping_control;
    ControlStatus.steering = steering_msg.steering_angle;

#ifdef __ENABLE_LATERAL_OUTPUT__
    printf("e=%- 3.02lf ff=%- 3.02f P=%- 4.02lf I=%- 4.02lf da=%- 4.02lf steer=%-.02lf\n", e,
            ControlStatus.lateral_ff, ControlStatus.lateral_P, ControlStatus.lateral_I,
            ControlStatus.lateral_Damping, steering_msg.steering_angle);
#endif

    steering_msg.steering_angle_speed = 11.0;
    steering_msg.signature = controller_signature;

#ifdef __ENABLE_LATERAL_CONTROLLER__
    g_lcm_steering.publish("steering_control_info", &steering_msg);
    // ROS
    std_msgs::Float64MultiArray temp_steering;
    temp_steering.data.push_back(steering_msg.steering_angle);
    temp_steering.data.push_back(steering_msg.steering_angle_speed);
    steering_pub.publish(temp_steering);
#endif
}

#else
void Controller::lateral_control_output(const Path& p) {
    constexpr double K_pid_p = 28.0;
    constexpr double K_pid_i = 0.0086;
    constexpr double K_pid_d = 5000.0;

    double P = 0.0;
    {
        P = K_pid_p * e;
    }
    double I = 0.0;
    {
        ival_steering += e;

        if (ival_steering > ival_steering_limit)
            ival_steering = ival_steering_limit;
        else if (ival_steering < -ival_steering_limit)
            ival_steering = -ival_steering_limit;

        I = K_pid_i * ival_steering;
    }
    double D = 0.0;
    {
        D = K_pid_d * (e - last_e);
    }

    obu_lcm::steering_control_info steering_msg;
    steering_msg.steering_angle = P + I + D;

    /// zd 将steering限制在-525~+561 －左+右
    if (steering_msg.steering_angle > origin_vehicle::MAX_STEERING_ANGLE)
        steering_msg.steering_angle = origin_vehicle::MAX_STEERING_ANGLE;
    else if (steering_msg.steering_angle < origin_vehicle::MIN_STEERING_ANGLE)
        steering_msg.steering_angle = origin_vehicle::MIN_STEERING_ANGLE;

    // 对参数赋值
    ControlStatus.lateral_ff = 0.0;
    ControlStatus.lateral_P = P;

    ControlStatus.lateral_I = I;
    ControlStatus.lateral_Damping = D;
    ControlStatus.steering = steering_msg.steering_angle;

#ifdef __ENABLE_LATERAL_OUTPUT__
    printf("e=%- 3.02lf P=%- 4.02lf I=%- 4.02lf D=%- 4.02lf steer=%-.02lf\n", e,
            ControlStatus.lateral_P, ControlStatus.lateral_I,
            ControlStatus.lateral_Damping, steering_msg.steering_angle);
#endif

    steering_msg.steering_angle_speed = 11.0;
    steering_msg.signature = controller_signature;

#ifdef __ENABLE_LATERAL_CONTROLLER__
    g_lcm_steering.publish("steering_control_info", &steering_msg);
#endif
}
#endif
