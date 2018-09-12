#ifndef _HEADING_H__
#define _HEADING_H__

#include <cmath>
#include <vector>
#include "spline.h"

//void  delta_heading(double h1,
//                    double h2,
//                    double& delta_h);

// 求取相关点的头指向。
int get_heading_h(double x1, double y1, double x2, double y2, double &h2,
                  double &h_r, int num);

// 求取相关点的头指向。
int get_heading(double x1, double y1, double x2, double y2, double &h2);

int get_steering_angle_h(double h1, double h2, double s, double &st,
                         double &st_r);

int get_steering_angle(double h1, double h2, double s, double &st);

double cau_heading_angle_from_ks(const tk::spline &s_x, const tk::spline &s_y,
                                 double cs, double every);

#endif /*_HEADING_H__*/
