#include "Point.h"

#include <math.h>
#include <vector>
#include <iostream>

#include "origin_vehicle.h"
#include "math_util.h"

using namespace std;

// 计算 |p1 p2| X |p1 p|
float get_cross(const Point &p1, const Point &p2, const Point &p) {
    float cross = (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
    return cross;
}

// 计算 |p1 p2| X |p1 p|
float get_cross_xx(const Point &p1, const Point &p2, const Point &p) {
    float cross = (p2.xx - p1.xx) * (p.yy - p1.yy)
                  - (p.xx - p1.xx) * (p2.yy - p1.yy);
    return cross;
}

//判断点是否在5X5 以原点为左下角的正方形内（便于测试）
bool Point::is_in_rect(Point &p1, Point &p2, Point &p3, Point &p4) {
    Point p;
    p.x = x;
    p.y = y;

    bool in_1 = get_cross(p1, p2, p) * get_cross(p3, p4, p) >= 0;
    bool in_2 = get_cross(p1, p2, p) * get_cross(p3, p4, p) >= 0;

    bool inbox = in_1 && in_2;

    return inbox;
}

//判断点是否在5X5 以原点为左下角的正方形内（便于测试）
bool Point::is_in_rect_xx(Point &p1, Point &p2, Point &p3, Point &p4) {
    Point p;
    p.xx = xx;
    p.yy = yy;

    bool in_1 = get_cross_xx(p1, p2, p) * get_cross_xx(p3, p4, p) >= 0;
    bool in_2 = get_cross_xx(p1, p2, p) * get_cross_xx(p3, p4, p) >= 0;

    bool inbox = in_1 && in_2;

    return inbox;
}

//判断两点是否相同
bool samep(Point p1, Point p2) {
    if ((p1.x == p2.x) && (p1.y == p2.y)) {
        return 1;
    } else {
        return 0;
    }
}

//求两点距离
double dist(Point p1, Point p2) {
    double ret = 0;

    ret = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));

    return ret;
}

/*
 已知两点，求连线中点坐标
 ---------------------------------
 两点(x1,y1),(x2,y2)的连线中点坐标
 ((x1+x2)/2,(y1+y2)/2)
 */
Point midpoint(Point p1, Point p2) {
    Point ret;
    ret.x = (p1.x + p2.x) / 2;
    ret.y = (p1.y + p2.y) / 2;
    return ret;
}
