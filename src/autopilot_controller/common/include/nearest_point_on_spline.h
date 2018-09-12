#ifndef _NEARST_POINT_ON_SPLINE_H__
#define _NEARST_POINT_ON_SPLINE_H__

#include "NaviPoint.hpp"
#include "spline.h"

double getClosestPointOnSpline(const tk::spline &sp_x, const tk::spline &sp_y,
                               const NaviPoint &testPoint, double s1, double s2, double s3,
                               double lower_limit, double upper_limit, double resolution = 1000,
                               int maxIterations = 20);

#endif /*_NEARST_POINT_ON_SPLINE_H__*/
