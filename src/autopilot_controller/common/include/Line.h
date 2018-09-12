#ifndef _LINE_H__
#define _LINE_H__

#include <cmath>
#include <vector>

//直线方程结构
struct Line {
    double k;    //斜率
    double b;    //截距
    Line() {
        k = b = 0;
    }

    ~Line() {

    }

    Line &operator=(const Line &src) {
        this->k = src.k;
        this->b = src.b;

        return *this;
    }
};

#endif /*_LINE_H__*/
