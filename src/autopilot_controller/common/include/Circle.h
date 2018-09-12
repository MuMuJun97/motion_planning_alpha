#ifndef _CIRCLE_H__
#define _CIRCLE_H__

#include <math.h>
#include <vector>

#include "NaviPoint.hpp"

// 点的位置
class Circle {
public:
    // 传感器相对坐标
    double R;
    double ks;
};

int get_circle(double x1, double y1, double x2, double y2, double x3, double y3,
               double &x, double &y, double &r, double &ks);

int sign_circle_e(double x, double y, double x1, double y1, double x2,
                  double y2);

int fit_circle(std::vector<NaviPoint> points, int num, double &A, double &B,
               double &R, double &ks);

#endif  /*_CIRCLE_H__*/
