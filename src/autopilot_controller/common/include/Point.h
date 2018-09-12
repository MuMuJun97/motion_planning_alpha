#ifndef _POINT_H__
#define _POINT_H__

#include <cmath>
#include <vector>

// 点的位置
class Point {
public:
    // 传感器相对坐标
    float x;
    float y;
    float z;

    // 绝对坐标
    float xx;
    float yy;
    float zz;

public:
    bool is_in_rect(Point &p1, Point &p2, Point &p3, Point &p4);

    bool is_in_rect_xx(Point &p1, Point &p2, Point &p3, Point &p4);

    Point() {
        x = y = z = 0;
        xx = yy = zz = 0;
    }

    ~Point() {

    }

    Point &operator=(const Point &src) {
        this->x = src.x;
        this->y = src.y;
        this->z = src.z;

        this->xx = src.xx;
        this->yy = src.yy;
        this->zz = src.zz;

        return *this;
    }
};

bool samep(Point p1, Point p2);

double dist(Point p1, Point p2);

Point midpoint(Point p1, Point p2);

float get_cross(const Point &p1, const Point &p2, const Point &p);

float get_cross_xx(const Point &p1, const Point &p2, const Point &p);

#endif /*_POINT_H__*/
