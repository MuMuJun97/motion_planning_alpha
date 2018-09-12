#include <iostream>
#include <cmath>

#include "Controller.hpp"
#include "math_util.h"
#include "cau_heading_steering.h"
#include "steering_angle.h"

using namespace std;

int Controller::extract_path_from_msg(const autopilot_msgs::WayPoints::Ptr &wayPoints,
                                      Path *target_path_ptr) {
    NaviPoint nd_AX7;
    double l, total_s = 0.0;
    double last_x, last_y;

    target_path_ptr->ref_points.clear();

    // 2nd, add nav points according to msg
    for (int i = 0; i < wayPoints->points.size(); i++) {
        // 根据 GPS
        double lat = wayPoints->points[i].latitude;
        double lon = wayPoints->points[i].longitude;

        double x = 0, y = 0;
        csTransfer.ll2xy(lat, lon, x, y);

        nd_AX7.position_x = x;
        nd_AX7.position_y = y;

        nd_AX7.speed_desired_Uxs = wayPoints->speeds[i];

        target_path_ptr->ref_points.push_back(nd_AX7);
    }

    // finally, calculate l and s for each navi_point
    total_s = 0.0;
    last_x = target_path_ptr->ref_points.front().position_x;
    last_y = target_path_ptr->ref_points.front().position_y;

    for (std::vector<NaviPoint>::iterator it = next_path->ref_points.begin();
         it != next_path->ref_points.end(); ++it) {
        l = length_two_points(it->position_x, it->position_y, last_x, last_y);
        if (l > 20.0)
            continue;

        total_s += l;
        it->s = total_s;

        last_x = it->position_x;
        last_y = it->position_y;
    }

    return target_path_ptr->size();
}

void Controller::switch_path_buffer() {
    std::swap(curr_path, next_path);
    m_matching.init();
}

int Controller::Calculate_e_sign(double x, double y, double yaw, double x_c, double y_c,
                                 double yaw_c) {
    double xa = x_c, ya = y_c;
    double xb = x_c + 10 * sin(to_radians(yaw_c));
    double yb = y_c + 10 * cos(to_radians(yaw_c));
    double xc = x, yc = y;

    // 在一条直线上
    double f = (xb - xa) * (yc - ya) - (xc - xa) * (yb - ya);

    if (f > 0)
        return -1;
    else
        return 1;
}

double Controller::Calculate_e(double x, double y, double yaw, double x_c, double y_c,
                               double yaw_c) {
    double xa = x_c, ya = y_c;
    double xb = x_c + 1 * sin(to_radians(yaw_c));
    double yb = y_c + 1 * cos(to_radians(yaw_c));
    double xc = x, yc = y;

    // 在一条直线上
    double f = (xb - xa) * (yc - ya) - (xc - xa) * (yb - ya);
    double length = sqrt((xa - xb) * (xa - xb) + (ya - yb) * (ya - yb));

    double offset_value = fabs(f / length);

    // 在匹配点左边输出+，右边输出-
    if (f > 0.0)
        return offset_value;
    else if (f == 0.0)
        return 0.0;
    else
        return -offset_value;
}

void Controller::Calculate_Mapping_parameters(const Path &p) {
    position_x_C = p.ref_points[match_point_no].position_x;
    position_y_C = p.ref_points[match_point_no].position_y;

    Yaw_C = p.ref_points[match_point_no].heading;
    //	由于车辆巨大的惯性，目标速度取8m/3m后的点的目标速度
    if (match_point_no + 80 < int(p.ref_points.size()))
        speed_desired_Uxs_C = p.ref_points[match_point_no + 80].speed_desired_Uxs;
    else if (match_point_no + 30 < int(p.ref_points.size()))
        speed_desired_Uxs_C = p.ref_points[match_point_no + 30].speed_desired_Uxs;
    else
        speed_desired_Uxs_C = p.ref_points[match_point_no].speed_desired_Uxs;

    steering_angle_C = p.ref_points[match_point_no].steering_ks;

//    e = sqrt( (position_x_C - CurrentX) * (position_x_C - CurrentX) +
//              (position_y_C - CurrentY) * (position_y_C - CurrentY) );

    Calculate_delta_phi_by_table(Current_yaw, Yaw_C, &delta_phi, &delta_phi_radian);

    e = Calculate_e(CurrentX, CurrentY, Current_yaw, position_x_C, position_y_C, Yaw_C);
    ControlStatus.e = e;

//    if (fabs(last_e - e) > 1.6 && last_e * e < 0.0) {
//        steering_angle_C = 0.0;
//        actual_path.clear();
//    }
}

void Controller::Calculate_control_parameters() {
    delta = steer_angle / origin_vehicle::STEERING_R;
}

bool Controller::Controller_preprocessing(const Path *p) {
    Calculate_controller_signature();

    double min_error = 100.0;

    Calculate_control_parameters();

    match_point_no = m_matching.MapMarch_Min_Distance_motion_planning(CurrentX, CurrentY,
                                                                      Current_yaw, *p, min_error);

    Calculate_Mapping_parameters(*p);

    if (match_point_no >= p->size() - 10) {
//	if (match_point_no >= 14000) {
        cerr << "match_point_no=" << match_point_no << ". There is no navigation point, exit"
             << endl;
        ROS_INFO("Goal Reached.");
        curr_path->ref_points.clear();
        next_path->ref_points.clear();
        return preprocessingStatus = false;
    }

    return preprocessingStatus = true;
}

void Controller::Controller_apply() {
    if (!preprocessingStatus || (can_start_auto == 0 && can_stop_auto == 1)) {
        lateral_none();
        longitudinal_none();
        return;
    }

    pthread_mutex_lock(&controlMutex);
    pthread_cond_signal(&controlWaitCond);
    pthread_mutex_unlock(&controlMutex);

    // 纵向控制部分5Hz
    //        count_hz++;
    //        if (count_hz >= 20) {
    //            controller_ptr->longitudinal_control_output(
    //                    *(controller_ptr->curr_path));
    //            count_hz = 0;
    //        }
    //在当前规划的结果中有大于20个控制点的情况下，才能执行
    if (enableLateralLog)
        mytimer::getHHMMSSUS(lateral_t0);

    lateral_control_output(*curr_path);
    last_e = e;

    if (enableLateralLog)
        mytimer::getHHMMSSUS(lateral_t1);

    preprocessingStatus = false;
}

void Controller::load_default_path() {
    recv_path[0].read_navi_file(runtimeParameters.map_filename);
    cau_all_output_from_single_spline(recv_path[0], recv_path[0].size(), desired_Ux);

    recv_path[1] = recv_path[0];
}
