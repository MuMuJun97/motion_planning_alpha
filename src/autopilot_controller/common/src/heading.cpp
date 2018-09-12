#include <math.h>
#include <vector>
#include <iostream>

#include "origin_vehicle.h"
#include "math_util.h"
#include "heading.h"
#include "Point.h"

using namespace std;

//测试值
/*xx = 1, yy = 1.414,  l=2, t_d = 30;
 xx = -1, yy = 1.414, l=2, t_d = -30;
 xx = 1, yy = -1.414, l=2, t_d = 150;
 xx = -1, yy = 1.414, l=2, t_d = 210;

 xx = 0, yy = 10,  l=10, t_d = 0;
 xx = 0, yy = -10, l=10, t_d = 180;
 xx = 10, yy = 0,  l=10, t_d = 90;
 xx = -10, yy = 0, l=10, t_d = 270;*/

// 求取相关点的头指向。
int get_heading_h(double x1, double y1, double x2, double y2, double &h2,
                  double &h_r, int num) {
    double xxx;
    double xx = (x2 - x1);
    double yy = (y2 - y1);

    double length = sqrt(xx * xx + yy * yy);

    if (length == 0)
        length = 0.000000001;

    double theta = 0.0;
    double theta_d = 0.0;

    xxx = fabs(xx);
    // atan 计算的角度范围为 +-180度， 而且xx和yy有符号问题。需要测试一下。
    if (xx > 0 && yy > 0) // 1
    {
        theta = asin(xxx / length);
        theta_d = to_degrees(theta);
    }
    if (xx < 0 && yy > 0) //4
    {
        theta = asin(xxx / length);
        theta_d = 360 - to_degrees(theta);
    }
    if (xx > 0 && yy < 0) //2
    {
        theta = asin(xxx / length);
        theta_d = 180 - to_degrees(theta);
    }
    if (xx < 0 && yy < 0) //3
    {
        theta = asin(xxx / length);
        theta_d = 180 + to_degrees(theta);
    }

    if (xx == 0 && yy > 0)
        theta_d = 0;
    if (xx == 0 && yy < 0)
        theta_d = 180;
    if (xx > 0 && yy == 0)
        theta_d = 90;
    if (xx < 0 && yy == 0)
        theta_d = 270;

    h2 = theta_d;

    if (fabs(h2 - h_r) > 5) {
        cout << " i " << num << " x " << xx << " y " << yy << " l " << length
             << " the " << theta << " th " << h2 << " r " << h_r << endl;
    }

    h_r = theta_d;

    return 1;
}

// 求取相关点的头指向。
int get_heading(double x1, double y1, double x2, double y2, double &h2) {
    double xxx;
    double xx = (x2 - x1);
    double yy = (y2 - y1);

    double length = sqrt(xx * xx + yy * yy);

    if (length == 0)
        length = 0.000000001;

    double theta = 0.0;
    double theta_d = 0.0;

    xxx = fabs(xx);
    // atan 计算的角度范围为 +-180度， 而且xx和yy有符号问题。需要测试一下。
    if (xx > 0 && yy > 0) // 1
    {
        theta = asin(xxx / length);
        theta_d = to_degrees(theta);
    }
    if (xx < 0 && yy > 0) //4
    {
        theta = asin(xxx / length);
        theta_d = 360 - to_degrees(theta);
    }
    if (xx > 0 && yy < 0) //2
    {
        theta = asin(xxx / length);
        theta_d = 180 - to_degrees(theta);
    }
    if (xx < 0 && yy < 0) //3
    {
        theta = asin(xxx / length);
        theta_d = 180 + to_degrees(theta);
    }

    if (xx == 0 && yy > 0)
        theta_d = 0;
    if (xx == 0 && yy < 0)
        theta_d = 180;
    if (xx > 0 && yy == 0)
        theta_d = 90;
    if (xx < 0 && yy == 0)
        theta_d = 270;

    h2 = theta_d;

    return 1;
}

double cau_heading_angle_from_ks(const tk::spline &s_x, const tk::spline &s_y,
                                 double cs, double every) {
    double heading = 0;
    double x, y;
    double xx, yy;

    x = s_x(cs);
    y = s_y(cs);

    xx = s_x(cs + every);
    yy = s_y(cs + every);

    get_heading(x, y, xx, yy, heading);
    /// zd 不知道为什么需要heading - 1;
    return heading - 1;
}

