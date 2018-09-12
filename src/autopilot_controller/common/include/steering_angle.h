#ifndef _STEER_ANGLE_H__
#define _STEER_ANGLE_H__

#include "spline.h"

double cau_steering_angle_from_ks(const tk::spline &s_x, const tk::spline &s_y, double cs,
                                  double Ux, double &rr, double every1, double every2);

#endif /*_STEER_ANGLE_H__*/
