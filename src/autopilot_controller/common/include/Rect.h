#ifndef _RECT_H__
#define _RECT_H__

#include <cmath>
#include <vector>

#include "Point.h"

class Rect {
public:
    float width;
    float height;
    float x, y;   // left-top

public:
    Point p1, p2, p3, p4;

    bool is_point_in(const Point &p) const;

    bool is_point_in_xx(const Point &p) const;

    bool cross_rect(const Rect &r) const;

    bool cross_rect_xx(const Rect &r) const;

    Rect() {
        x = y = 0;
        width = height = 0;
    }

    ~Rect() {

    }

    Rect &operator=(const Rect &src) {

        this->p1 = src.p1;
        this->p2 = src.p2;
        this->p3 = src.p3;
        this->p4 = src.p4;

        this->width = src.width;
        this->height = src.height;
        this->x = src.x;
        this->y = src.y;

        return *this;

    }

    static bool cross_line(double left, double right, double y, double top,
                           double bottom, double x);

    static bool cross_in_two_rect(const Rect &r1, const Rect &r2);

    static bool cross_in_two_rect_xx(const Rect &r1, const Rect &r2);

//    static bool cross_two_rect(const Rect& r1, const Rect& r2);
//    static bool cross_two_rect_xx(const Rect& r1, const Rect& r2);

};

#endif /*_RECT_H__*/
