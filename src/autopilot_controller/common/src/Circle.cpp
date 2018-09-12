#include "Circle.h"

#include <vector>
#include <iostream>
#include <cmath>

#include "math_util.h"
#include "heading.h"
#include "Point.h"

int get_circle(double x1, double y1, double x2, double y2, double x3, double y3,
               double &x, double &y, double &r, double &ks) {
    double a, b, c, d, e, f, g;

    a = 2 * (x2 - x1);
    b = 2 * (y2 - y1);

    d = 2 * (x3 - x2);
    e = 2 * (y3 - y2);

    c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
    f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;

    g = b * d - e * a;   // b*d / e*a  =1

    if (fabs(g) <= 0.00000001)
        g = 0.00000001;

    x = (b * f - e * c) / g;
    y = (d * c - a * f) / g;

    r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

    if (fabs(g) > 0.00000001)
        ks = 1 / r;
    else
        ks = 0;

    return 1;
}

int sign_circle_e(double x, double y, double x1, double y1, double x2,
                  double y2) {
    double xa = x1, ya = y1;
    double xb = x2, yb = y2;
    double xc = x, yc = y;

    // 在一条直线上
    double f = (xb - xa) * (yc - ya) - (xc - xa) * (yb - ya);

    if (f > 0)
        return -1;
        //   else if(f == 0) return 0;
    else
        return 1;
}

//对数据进行拟合
int fit_circle(std::vector<NaviPoint> points, int num, double &A, double &B,
               double &R, double &ks) {
    int i;
    double X1, X2, X3, Y1, Y2, Y3, X1Y1, X1Y2, X2Y1;
    double C, D, E, G, H, N;
    double a, b, c;

    //拟合数据数量判断
    if (num < 3) {
        std::cout << "Error: fit data number is less than 3!" << std::endl;
        return 0;
    }

    X1 = X2 = X3 = Y1 = Y2 = Y3 = X1Y1 = X1Y2 = X2Y1 = 0;

    for (i = 0; i < num; i++) {
        double x = points[i].position_x;
        double y = points[i].position_y;

        X1 = X1 + x;
        Y1 = Y1 + y;
        X2 = X2 + x * x;
        Y2 = Y2 + y * y;
        X3 = X3 + x * x * x;
        Y3 = Y3 + y * y * y;
        X1Y1 = X1Y1 + x * y;
        X1Y2 = X1Y2 + x * y * y;
        X2Y1 = X2Y1 + x * x * y;
    }

    N = num;
    C = N * X2 - X1 * X1;
    D = N * X1Y1 - X1 * Y1;
    E = N * X3 + N * X1Y2 - (X2 + Y2) * X1;
    G = N * Y2 - Y1 * Y1;
    H = N * X2Y1 + N * Y3 - (X2 + Y2) * Y1;

    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * X1 + b * Y1 + X2 + Y2) / N;

    A = a / (-2);
    B = b / (-2);
    R = sqrt(a * a + b * b - 4 * c) / 2;

    if (R != 0)
        ks = 1 / R;
    else
        ks = 0;

    return 0;
}
