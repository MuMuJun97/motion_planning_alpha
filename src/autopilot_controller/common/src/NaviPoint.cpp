#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include "NaviPoint.hpp"
#include "ECU.hpp"

using namespace std;

void NaviPoint::copyfrom(const NaviPoint &src) {
    position_x = src.position_x;
    position_y = src.position_y;
    position_z = src.position_z;

//    lat = src.lat;
//    lon = src.lon;
//    height = src.height;

//    speed = src.speed;
//    lateral_speed = src.lateral_speed;
//    longitudinal_speed = src.longitudinal_speed;
//    down_speed = src.down_speed;

//    roll = src.roll;
//    pitch = src.pitch;
    heading = src.heading;

//    accelerataion = src.accelerataion;
//    lateral_accelerate = src.lateral_accelerate;
//    longitudinal_accelerate = src.longitudinal_accelerate;
//    down_accelerate = src.down_accelerate;

//    roll_speed = src.roll_speed;
//    pitch_speed = src.pitch_speed;
//    heading_speed = src.heading_speed;

    steering_ks = src.steering_ks;
//    k_s = src.k_s;
    speed_desired_Uxs = src.speed_desired_Uxs;
//    acceleration_desired_Axs = src.acceleration_desired_Axs;

//    point_no = src.point_no;
    s = src.s;
}

void NaviPoint::copyto(NaviPoint &dst) {
    dst.position_x = position_x;
    dst.position_y = position_y;
    dst.position_z = position_z;

//    dst.lat = lat;
//    dst.lon = lon;
//    dst.height = height;

//    dst.lateral_speed = lateral_speed;
//    dst.longitudinal_speed = longitudinal_speed;
//    dst.down_speed = down_speed;

//    dst.roll = roll;
//    dst.pitch = pitch;
    dst.heading = heading;

//    dst.lateral_accelerate = lateral_accelerate;
//    dst.longitudinal_accelerate = longitudinal_accelerate;
//    dst.down_accelerate = down_accelerate;

//    dst.roll_speed = roll_speed;
//    dst.pitch_speed = pitch_speed;
//    dst.heading_speed = heading_speed;

    dst.steering_ks = steering_ks;
//    dst.k_s = k_s;
    dst.speed_desired_Uxs = speed_desired_Uxs;
//    dst.acceleration_desired_Axs = acceleration_desired_Axs;

//    dst.point_no = point_no;
    dst.s = s;
}

NaviPoint &NaviPoint::operator=(const NaviPoint &src) {
    this->position_x = src.position_x;
    this->position_y = src.position_y;
    this->position_z = src.position_z;

//    this->lat = src.lat;
//    this->lon = src.lon;
//    this->height = src.height;

//    this->speed = src.speed;
//    this->lateral_speed = src.lateral_speed;
//    this->longitudinal_speed = src.longitudinal_speed;
//    this->down_speed = src.down_speed;

//    this->roll = src.roll;
//    this->pitch = src.pitch;
    this->heading = src.heading;

//    this->accelerataion = src.accelerataion;
//    this->lateral_accelerate = src.lateral_accelerate;
//    this->longitudinal_accelerate = src.longitudinal_accelerate;
//    this->down_accelerate = src.down_accelerate;

//    this->roll_speed = src.roll_speed;
//    this->pitch_speed = src.pitch_speed;
//    this->heading_speed = src.heading_speed;

    this->steering_ks = src.steering_ks;
//    this->k_s = src.k_s;
    this->speed_desired_Uxs = src.speed_desired_Uxs;
//    this->acceleration_desired_Axs = src.acceleration_desired_Axs;

//    this->point_no = src.point_no;
    this->s = src.s;

    return *this;
}

void NaviPoint::copyto(obu_lcm::nav_points *msg_p) {
    msg_p->p_x = position_x;
    msg_p->p_y = position_y;

    msg_p->speed_desired_Uxs = speed_desired_Uxs;
}

void NaviPoint::copyfrom(const obu_lcm::nav_points *msg_p) {
    position_x = msg_p->p_x;
    position_y = msg_p->p_y;

    speed_desired_Uxs = msg_p->speed_desired_Uxs;
}

double NaviPoint::length_of_two_navipoint(const NaviPoint &p1, const NaviPoint &p2) {
    return sqrt(
            (p1.position_x - p2.position_x) * (p1.position_x - p2.position_x)
            + (p1.position_y - p2.position_y) * (p1.position_y - p2.position_y));

}
