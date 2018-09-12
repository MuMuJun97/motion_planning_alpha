#include <math.h>

#include "math_util.h"
#include "NaviPoint.hpp"
#include "spline.h"

double clamp(double value, double lowerLimit, double upperLimit) {
    if (value < lowerLimit) { return lowerLimit; }
    else if (value > upperLimit) { return upperLimit; }
    else { return value; }
}

double getClosestPointOnSpline(const tk::spline &sp_x,
                               const tk::spline &sp_y,
                               const NaviPoint &testPoint,
                               double s1,
                               double s2,
                               double s3,
                               double lower_limit,
                               double upper_limit,
                               double resolution = 1000,
                               int maxIterations = 20) {

    double s[3];  // The estimates
    double Ds[3]; // The distances squared to the estimates
    double sk, skLast; // sk is the "hopefully" converging value generated, skLast is the previous one
    double Ps[4]; // The function P(s) evaluated for the 4 values

    // For gradient and curviture approximation
    double width = (upper_limit - lower_limit) / resolution; // step would be 1/1000 of a spline length

    // The range of the parameter value for a spline segment * proportion of it is used to test for an exit condition
    double termCond = (upper_limit - lower_limit) / resolution;

    NaviPoint p0, p1, p2;
    NaviPoint pt1, pt2, pt3;

    s[0] = s1;
    s[1] = s2;
    s[2] = s3;

    for (int i = 0; i < maxIterations; i++) {
        // its typically done in under 10
        p0.position_x = sp_x(s[0]);
        p0.position_y = sp_y(s[0]);
        p1.position_x = sp_x(s[1]);
        p1.position_y = sp_y(s[1]);
        p2.position_x = sp_x(s[2]);
        p2.position_y = sp_y(s[2]);

        Ds[0] = NaviPoint::length_of_two_navipoint(p0, testPoint);  // 返回这个向量的长度的平方（只读）。
        Ds[1] = NaviPoint::length_of_two_navipoint(p1, testPoint);
        Ds[2] = NaviPoint::length_of_two_navipoint(p2, testPoint);

        // Quadratic Minimization Bit
        sk = 0.5 * ((s[1] * s[1] - s[2] * s[2]) * Ds[0] +
                    (s[2] * s[2] - s[0] * s[0]) * Ds[1] +
                    (s[0] * s[0] - s[1] * s[1]) * Ds[2]) /
             ((s[1] - s[2]) * Ds[0]
              + (s[2] - s[0]) * Ds[1]
              + (s[0] - s[1]) * Ds[2]);

        // modify
        if (__builtin_isnan(sk)) {
            // denominator = 0, how unfortunate
            //printf ("isnan %d %f\n", i, skLast);
            sk = skLast; // keep going?
            return skLast;
            //return true;
        }

        // Newton Bit
        sk = clamp(sk, lower_limit + width, upper_limit - width);
        // so can interpolate points for Newtons method

        double grad, curv; // 1st 2nd derivatives

        pt1.position_x = sp_x(sk - width);
        pt1.position_y = sp_y(sk - width);
        pt2.position_x = sp_x(sk);
        pt2.position_y = sp_y(sk);
        pt3.position_x = sp_x(sk + width);
        pt3.position_y = sp_y(sk + width);

        double Ds_pt1 = NaviPoint::length_of_two_navipoint(pt1, testPoint);
        double Ds_pt2 = NaviPoint::length_of_two_navipoint(pt2, testPoint);
        double Ds_pt3 = NaviPoint::length_of_two_navipoint(pt3, testPoint);

        // its typically done in under 10
        double g1 = (Ds_pt2 - Ds_pt1) / width;
        double g2 = (Ds_pt3 - Ds_pt2) / width;

        grad = (Ds_pt3 - Ds_pt1) / (2 * width);

        curv = (g2 - g1) / width;

        if (curv != 0.0) {
            sk = sk - grad / curv;
            sk = clamp(sk, lower_limit, upper_limit);
        }

        // termination criteria
        // difference between skLast and sk <= range of s over the segment x small constant
        if (i > 0) {
            if (fabs(sk - skLast) <= termCond) {
                //printf ("exit condition met %d %f %f\n", i, Math::Abs(sk - skLast), termCond);
                return sk;
            }
        }
        skLast = sk;

        // chose the best 3 from their Ps values (the closest ones we keep)
        // general Ps equation
        // Ps =   ((s-s2)*(s-s3))/((s1-s2)*(s1-s3)) * Ds1 +
        //        ((s-s1)*(s-s3))/((s2-s1)*(s2-s3)) * Ds2 +
        //        ((s-s1)*(s-s2))/((s3-s1)*(s3-s2)) * Ds3;

        Ps[0] = ((s[0] - s[1]) * (s[0] - s[2])) / ((s[0] - s[1]) * (s[0] - s[2])) * Ds[0];

        Ps[1] = ((s[1] - s[0]) * (s[1] - s[2])) / ((s[1] - s[0]) * (s[1] - s[2])) * Ds[1];

        Ps[2] = ((s[2] - s[0]) * (s[2] - s[1])) / ((s[2] - s[0]) * (s[2] - s[1])) * Ds[2];

        Ps[3] = ((sk - s[1]) * (sk - s[2])) / ((s[0] - s[1]) * (s[0] - s[2])) * Ds[0] +
                ((sk - s[0]) * (sk - s[2])) / ((s[1] - s[0]) * (s[1] - s[2])) * Ds[1] +
                ((sk - s[0]) * (sk - s[1])) / ((s[2] - s[0]) * (s[2] - s[1])) * Ds[2];


        // find the worest one
        int biggest = 0;
        for (int i = 1; i < 4; i++) {
            if (Ps[i] > Ps[biggest]) {
                biggest = i;
            }
        }

        if (biggest <= 2) {
            // update one of the estimates
            // equations will blow up if any of the estimates are the same
            s[biggest] = sk;
            // make them unique values
            for (int i = 0; i < 3; i++) {
                for (int j = i + 1; j < 3; j++) {
                    if (s[i] == s[j]) {
                        if (s[j] < 0.5) {
                            s[j] = s[j] + 0.0001;
                        } else {
                            s[j] = s[j] - 0.0001;
                        }
                    }
                }
            }
        }
    }

    return sk;
}

