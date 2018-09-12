#include "Rect.h"

#include "math_util.h"
#include "origin_vehicle.h"
#include "Point.h"

//对于一个点是否在矩形内的判断。
//只需要判断该点是否在上下两条边和左右两条边之间就行，判断一个点是否在两条线段之间夹着，就转化成，
//判断一个点是否在某条线段的一边上，就可以利用叉乘的方向性，来判断夹角是否超过了180度 如下图：
//只要判断(p1 p2 X p1 p ) * (p3 p4 X p3 p1)  >= 0 就说明p在p1p2,p3p4中间夹着，同理计算另两边就可以了
bool Rect::is_point_in(const Point &p) const {
    bool inbox = get_cross(p1, p2, p) * get_cross(p3, p4, p) >= 0
                 && get_cross(p2, p3, p) * get_cross(p4, p1, p) >= 0;
    return inbox;
}

bool Rect::is_point_in_xx(const Point &p) const {
    bool inbox = get_cross_xx(p1, p2, p) * get_cross_xx(p3, p4, p) >= 0
                 && get_cross_xx(p2, p3, p) * get_cross_xx(p4, p1, p) >= 0;
    return inbox;
}

//如果两个矩形相交，则必然存在线条交叉，而能交叉的线条只有横线和竖线，
//两根横线或两根竖线都不可能交叉。
//所以，这个问题就转化成寻找是否存在交叉的横线与竖线。（横线重叠情况？）
//另外，A线与B线交叉等价于B线与A线交叉，
//所以，只要写一个函数就足够用了，
//多调用几次，反正计算机是专门做简单而又烦琐的工作的。

//下面是这个函数：判断一条横线和一条竖线是否交叉。
//该函数的参数分别是：横线左、横线右，横线Y，竖线上，竖线下，竖线X。
bool Rect::cross_line(double left, double right, double y, double top,
                      double bottom, double x) {
    return (top < y) && (bottom > y) && (left < x) && (right > x);
}

//判断两个矩形是否相交, 变为判断矩形的四个顶端，是否会在另外一个矩形的内部
bool Rect::cross_in_two_rect(const Rect &r1, const Rect &r2) {
    bool b1, b2, b3, b4, b5, b6, b7, b8;

    b1 = r1.is_point_in(r2.p1);
    b2 = r1.is_point_in(r2.p2);
    b3 = r1.is_point_in(r2.p3);
    b4 = r1.is_point_in(r2.p4);

    b5 = r2.is_point_in(r1.p1);
    b6 = r2.is_point_in(r1.p2);
    b7 = r2.is_point_in(r1.p3);
    b8 = r2.is_point_in(r1.p4);

    return b1 || b2 || b3 || b4 || b5 || b6 || b7 || b8;
}

//判断两个矩形是否相交, 变为判断矩形的四个顶端，是否会在另外一个矩形的内部
bool Rect::cross_in_two_rect_xx(const Rect &r1, const Rect &r2) {
    bool b1, b2, b3, b4, b5, b6, b7, b8;

    b1 = r1.is_point_in_xx(r2.p1);
    b2 = r1.is_point_in_xx(r2.p2);
    b3 = r1.is_point_in_xx(r2.p3);
    b4 = r1.is_point_in_xx(r2.p4);

    b5 = r2.is_point_in_xx(r1.p1);
    b6 = r2.is_point_in_xx(r1.p2);
    b7 = r2.is_point_in_xx(r1.p3);
    b8 = r2.is_point_in_xx(r1.p4);

    return b1 || b2 || b3 || b4 || b5 || b6 || b7 || b8;
}

bool Rect::cross_rect(const Rect &r) const {
    return cross_in_two_rect(*this, r);
}

bool Rect::cross_rect_xx(const Rect &r) const {
    return cross_in_two_rect_xx(*this, r);
}
