#ifndef _NAVPOINT_HPP__
#define _NAVPOINT_HPP__

#include <vector>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "nav_points.hpp"

class NaviPoint {

public:
    double position_x;
    double position_y;
    double position_z;

//  double lat;
//  double lon;
//  double height;

//	double speed;
//	double lateral_speed;
//	double longitudinal_speed;
//	double down_speed;

//	double roll;
//	double pitch;
    double heading;

//	double accelerataion;
//	double lateral_accelerate;
//	double longitudinal_accelerate;
//	double down_accelerate;

//	double roll_speed;
//	double pitch_speed;
//  double heading_speed;

    double steering_ks;
//	double steering_angle_speed;

    double speed_desired_Uxs;
//	double acceleration_desired_Axs;

public:
//	int point_no;
    double s;
//  double lateral_offset;
//  double k_s;
//	double d_lateral_offset;
//	double d2_lateral_offset;

public:
    void copyfrom(const NaviPoint &src);

    void copyto(NaviPoint &dst);

    void copyto(obu_lcm::nav_points *msg_p);

    void copyfrom(const obu_lcm::nav_points *const msg_p);

    NaviPoint &operator=(const NaviPoint &src);

    static double length_of_two_navipoint(const NaviPoint &p1,
                                          const NaviPoint &p2);

    NaviPoint() {
        position_x = 0.0;
        position_y = 0.0;
        position_z = 0.0;

//		lat = 0.0;
//		lon = 0.0;
//		height = 0.0;
//
//		speed = 0.0;
//		lateral_speed = 0.0;
//		longitudinal_speed = 0.0;
//		down_speed = 0.0;

//		roll = 0.0;
//		pitch = 0.0;
        heading = 0.0;

//		accelerataion = 0.0;
//		lateral_accelerate = 0.0;
//		longitudinal_accelerate = 0.0;
//		down_accelerate = 0.0;

//		roll_speed = 0.0;
//		pitch_speed = 0.0;
//      heading_speed = 0.0;

        steering_ks = 0.0;
//		steering_angle_speed = 0.0;

        speed_desired_Uxs = 0.0;
//		acceleration_desired_Axs = 0.0;

//		point_no = 0.0;
        s = 0.0;
//      lateral_offset = 0.0;
//		d_lateral_offset = 0.0;
//		d2_lateral_offset = 0.0;
//      k_s = 0.0;  // k(s) rad
    }

};

#endif /*_NAVPOINT_HPP__*/
