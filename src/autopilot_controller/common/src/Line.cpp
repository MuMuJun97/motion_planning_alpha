#include <math.h>
#include <vector>
#include <iostream>

#include "Line.h"
#include "math_util.h"
#include "Point.h"

/*
 已知两点，求连线方程
 -------------------------------------
 两点(x1,y1),(x2,y2)的连线方程表示成Y=ax+b
 a=(y1-y2)/(x1-x2)

 b= y2 - x2*a
 或者
 b= y1- x1*a
 -------------------------------------

 */
Line line2p(Point p1, Point p2) {
    Line ret;
    if ((p1.x == p2.x) && (p1.y = p2.y)) {
        ret.k = 0;
        ret.b = 0;
    } else {
        ret.k = (p1.y - p2.y) / (p1.x - p2.x);
        ret.b = p1.y - p1.x * ret.k;
    }
    return ret;
}

//判断两直线是否相交
bool iscross(Line l1, Line l2) {
    if (l1.k == l2.k) {
        return 0;
    } else {
        return 1;
    }
}

/*
 已知两直线，求交点坐标
 两条直线

 y1=ax+b
 y2=cx+d
 求交点
 a<>c时有交点（不平行）
 y1=y2时
 => ax+b=cx+d
 => x=(d-b)/(a-c)

 x1=x2 时
 y=(ad-cb)/(a-c)

 交点坐标是( (d-b)/(a-c),(ad-cb)/(a-c))
 */

Point crosspoint(Line l1, Line l2) {
    Point ret;
    if (!iscross(l1, l2)) {
        ret.x = 0;
        ret.y = 0;
    } else {
        ret.x = (l2.b - l1.b) / (l1.k - l2.k);
        ret.y = (l1.k * l2.b - l2.k * l1.b) / (l1.k - l2.k);
    }
    return ret;
}

/*
 求斜率为k的直线,转过n弧度后，且经过某一点的方程
 */
Line linepoint(Point p, double k, double n) //过某点直线方程
{
    Line ret;
    ret.k = tan(atan(k) + n);
    ret.b = p.y - ret.k * p.x;
    return ret;
}

/*
 -------------------------------------
 直线Y=ax+b 与X轴夹角是
 atan((-b)/a)
 转90度只要加上90度(3.14/2)就可以了
 ---------------------------------
 */
int cau_circle(Point p1, Point p2, Point p3, double &circler, Point &circlep) {
    Point m1, m2, m3;
    Line l1, l2, l3;
    Line lm1, lm2, lm3;
    double dis2, dis3;

    if (samep(p1, p2))
        return -1;

//    cout<<"第1点坐标是: ("<<p1.x <<"," <<p1.y <<") " << endl;
//    cout<<"第2点坐标是: ("<<p2.x <<"," <<p2.y <<") " << endl;
//    cout<<"第3点坐标是: ("<<p3.x <<"," <<p3.y <<") " << endl;

    m1 = midpoint(p1, p2);
    m2 = midpoint(p2, p3);
    m3 = midpoint(p3, p1);

//    cout<<"p1,p2中点坐标是: ("<<m1.x <<"," <<m1.y <<") " << endl;
//    cout<<"p2,p3中点坐标是: ("<<m2.x <<"," <<m2.y <<") " << endl;
//   cout<<"p3,p1中点坐标是: ("<<m3.x <<"," <<m3.y <<") " << endl;

    l1 = line2p(p1, p2);
    l2 = line2p(p2, p3);
    l3 = line2p(p3, p1);

//   cout<<"P1,P2连线方程是: Y=("<< (float) l1.k <<")X+(" << (float) l1.b <<") " << endl;
//   cout<<"P2,P3连线方程是: Y=("<< (float) l2.k <<")X+(" << (float) l2.b <<") " << endl;
//   cout<<"P3,P1连线方程是: Y=("<< (float) l3.k <<")X+(" << (float) l3.b <<") " << endl;

    lm1 = linepoint(m1, l1.k, PI / 2);
    lm2 = linepoint(m2, l2.k, PI / 2);
    lm3 = linepoint(m3, l3.k, PI / 2);

//   cout<<"P1,P2连线的垂线方程是: Y=(" << (float) lm1.k << ")X+("<< (float) lm1.b << ") " << endl;
//   cout<<"P2,P3连线的垂线方程是: Y=(" << (float) lm2.k << ")X+("<< (float) lm2.b << ") " << endl;
//   cout<<"P3,P1连线的垂线方程是: Y=(" << (float) lm3.k << ")X+("<< (float) lm3.b << ") " << endl;

    circlep = crosspoint(lm1, lm2);
    circler = dist(circlep, p1);

    dis2 = dist(circlep, p2);
    dis3 = dist(circlep, p3);

//   cout<< "圆心坐标是: (" << circlep.x << "," << circlep.y << ") " << endl;
//   cout<< "半径是: (" << circler << ") " << endl;
//   cout<< "到另两点的距离是: (" << dis2 << "," << dis3 << ") " << endl;

    return 0;
}

int cau_ks(Point p1, Point p2, Point p3, double &k_s, double &steering_d) {
    Point m1, m2, m3;
    Line l1, l2, l3;
    Line lm1, lm2, lm3;
    double dis2, dis3;
    double circler;
    Point circlep;

    double ks_value, ks_e;
    double steering;
    double f;

    if (samep(p1, p2))
        return -1;

//    cout<<"第1点坐标是: ("<<p1.x <<"," <<p1.y <<") " << endl;
//    cout<<"第2点坐标是: ("<<p2.x <<"," <<p2.y <<") " << endl;
//    cout<<"第3点坐标是: ("<<p3.x <<"," <<p3.y <<") " << endl;

    m1 = midpoint(p1, p2);
    m2 = midpoint(p2, p3);
    m3 = midpoint(p3, p1);

//    cout<<"p1,p2中点坐标是: ("<<m1.x <<"," <<m1.y <<") " << endl;
//    cout<<"p2,p3中点坐标是: ("<<m2.x <<"," <<m2.y <<") " << endl;
//   cout<<"p3,p1中点坐标是: ("<<m3.x <<"," <<m3.y <<") " << endl;

    l1 = line2p(p1, p2);
    l2 = line2p(p2, p3);
    l3 = line2p(p3, p1);

//   cout<<"P1,P2连线方程是: Y=("<< (float) l1.k <<")X+(" << (float) l1.b <<") " << endl;
//   cout<<"P2,P3连线方程是: Y=("<< (float) l2.k <<")X+(" << (float) l2.b <<") " << endl;
//   cout<<"P3,P1连线方程是: Y=("<< (float) l3.k <<")X+(" << (float) l3.b <<") " << endl;

    lm1 = linepoint(m1, l1.k, PI / 2);
    lm2 = linepoint(m2, l2.k, PI / 2);
    lm3 = linepoint(m3, l3.k, PI / 2);

//   cout<<"P1,P2连线的垂线方程是: Y=(" << (float) lm1.k << ")X+("<< (float) lm1.b << ") " << endl;
//   cout<<"P2,P3连线的垂线方程是: Y=(" << (float) lm2.k << ")X+("<< (float) lm2.b << ") " << endl;
//   cout<<"P3,P1连线的垂线方程是: Y=(" << (float) lm3.k << ")X+("<< (float) lm3.b << ") " << endl;

    circlep = crosspoint(lm1, lm2);
    circler = dist(circlep, p1);

    dis2 = dist(circlep, p2);
    dis3 = dist(circlep, p3);

//   cout<< "圆心坐标是: (" << circlep.x << "," << circlep.y << ") " << endl;
//   cout<< "半径是: (" << circler << ") " << endl;
//   cout<< "到另两点的距离是: (" << dis2 << "," << dis3 << ") " << endl;
    if (circler != 0)
        ks_value = 1 / circler;
    else
        return -1;

// 左右方向是相对前进方向的,只要指定了前进方向就可以知道左右(比如指定前进方向是从直线的起点到终点).
// 判断点在直线的左侧还是右侧是计算几何里面的一个最基本算法.使用矢量来判断.

// 定义：平面上的三点P1(x1,y1),P2(x2,y2),P3(x3,y3)的面积量：
// S(P1,P2,P3)=|y1 y2 y3|= (x1-x3)*(y2-y3)-(y1-y3)*(x2-x3)
// 当P1P2P3逆时针时S为正的，当P1P2P3顺时针时S为负的。

// 令矢量的起点为A，终点为B，判断的点为C，
// 如果S（A，B，C）为正数，则C在矢量AB的左侧；
// 如果S（A，B，C）为负数，则C在矢量AB的右侧；
// 如果S（A，B，C）为0，则C在直线AB上。

    //x0, y0 在直线（x1, y1), (x2,y2)的左侧或者右侧，左侧为正， 右侧为负
    // p1, p2;
//   f = (x1-x0)*(y2-y0)-(y1-y0)*(x2-x0);

    f = (p1.x - circlep.x) * (p2.y - circlep.y)
        - (p1.y - circlep.y) * (p2.x - circlep.x);

    if (f > 0)
        ks_e = -1;
    else if (f == 0)
        ks_e = 0;
    else
        ks_e = 1;

    k_s = ks_e * ks_value;

    // 角度需要控制在 +/- 180度之内
    steering = atan(k_s * origin_vehicle::WHEEL_BASE);

    //  cout << " steering " << steering;
    steering_d = to_degrees(steering) * origin_vehicle::STEERING_R;

    return 0;
}
