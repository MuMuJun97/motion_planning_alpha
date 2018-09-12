#include <cmath>

#include "origin_vehicle.h"
#include "math_util.h"
#include "Circle.h"
#include "spline.h"

using namespace std;

double cau_steering_angle_from_ks(const tk::spline &s_x, const tk::spline &s_y,
                                  double cs, double Ux, double &rr, double every1, double every2) {
    double xx, yy;
    double xxx, yyy;
    double xxxx, yyyy;
    double x_r, y_r;  // 构建出的圆的圆心
    double r;  // 构建出的圆的半径
    double e;  // r的符号
    double ks; // ks = 1 / r
    double ks_L;
    double steering_ks;
    double Kug;
    double Wf, Wr;

    double L = origin_vehicle::WHEEL_BASE;
    double g = G_acc;
    double Cf = origin_vehicle::CORNERING_FRONT;
    double Cr = origin_vehicle::CORNERING_REAR;
    double l_a = origin_vehicle::LENGTH_A;
    double l_b = origin_vehicle::LENGTH_B;
    double m = origin_vehicle::CAR_WEIGHT;

    xx = s_x(cs + every1);
    yy = s_y(cs + every1);

    xxx = s_x(cs);
    yyy = s_y(cs);

    xxxx = s_x(cs + every2);
    yyyy = s_y(cs + every2);

    get_circle(xxx, yyy, xx, yy, xxxx, yyyy, x_r, y_r, r, ks);

    e = sign_circle_e(xxx, yyy, xx, yy, xxxx, yyyy);

    ks = e * ks;

    Wf = (l_b / L) * m * g;
    Wr = (l_a / L) * m * g;

    Kug = Wf / Cf - Wr / Cr;

    ks_L = (L + Kug * Ux * Ux / g) * ks;
    steering_ks = origin_vehicle::STEERING_R * to_degrees(ks_L);

    /// zd 不知道为什么需要steering_ks = (int) (steering_ks + 0.5 + 3);
    steering_ks = int(steering_ks + 0.5);

    rr = e * r;

    return steering_ks;
}
