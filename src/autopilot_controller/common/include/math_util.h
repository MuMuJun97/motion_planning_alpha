#ifndef _MATH_UTIL_H__
#define _MATH_UTIL_H__

#include <cmath>
#include <cassert>
#include "Geography.hpp"
#include "origin_vehicle.h"

#ifndef PI
#define PI (M_PI)
#endif

#ifndef G_acc //重力加速度
// 广州   9.7833
// 上海   9.7964
// 郑州   9.7966
// 北京   9.8015
// 沈阳   9.8035
// 哈尔滨 9.8066
#define G_acc (9.78)
#endif

#define TWOPI_INV (0.5 / M_PI)
#define TWOPI     (2.0 * M_PI)

static inline double sq(double v) {
    return v * v;
}

static inline double sgn(double v) {
    return (v >= 0) ? 1 : -1;
}

// random number between [0, 1)
static inline float randf() {
    return ((float) rand()) / (RAND_MAX + 1.0);
}

static inline float signed_randf() {
    return randf() * 2 - 1;
}

// return a random integer between [0, bound)
static inline int irand(int bound) {
    int v = (int) (randf() * bound);
    assert(v >= 0);
    assert(v < bound);
    return v;
}

/** valid only for v > 0 **/
static inline double mod2pi_positive(double vin) {
    double q = vin * TWOPI_INV + 0.5;
    int qi = (int) q;

    return vin - qi * TWOPI;
}

/** Map v to [-PI, PI] **/
static inline double mod2pi(double vin) {
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

/** Return vin such that it is within PI degrees of ref **/
static inline double mod2pi_ref(double ref, double vin) {
    return ref + mod2pi(vin - ref);
}

static inline int theta_to_int(double theta, int max) {
    theta = mod2pi_ref(PI, theta);
    int v = (int) (theta / (2 * PI) * max);

    if (v == max)
        v = 0;

    assert(v >= 0 && v < max);
    return v;
}

static inline int imin(int a, int b) {
    return (a < b) ? a : b;
}

static inline int imax(int a, int b) {
    return (a > b) ? a : b;
}

static inline int64_t imin64(int64_t a, int64_t b) {
    return (a < b) ? a : b;
}

static inline int64_t imax64(int64_t a, int64_t b) {
    return (a > b) ? a : b;
}

static inline int iclamp(int v, int minv, int maxv) {
    return imax(minv, imin(v, maxv));
}

static inline double fclamp(double v, double minv, double maxv) {
    return fmax(minv, fmin(v, maxv));
}

static inline double fclamp_360(double v) {
    if (v > 360.0)
        v -= 360.0;
    else if (v < 0.0)
        v += 360.0;

    return v;
}

static inline double length_two_points(double x, double y, double xx,
                                       double yy) {
    double x_xx = x - xx;
    double y_yy = y - yy;

    return sqrt(x_xx * x_xx + y_yy * y_yy);
}

static inline double streeing_to_ks(double steering_angle) {
    return tan(to_radians(steering_angle / origin_vehicle::STEERING_R))
           / origin_vehicle::WHEEL_BASE;
}

static inline double speed_from_ll(double lon_speed, double lat_speed) {
    return sqrt(lon_speed * lon_speed + lat_speed * lat_speed);
}

static inline double calculate_vertical_direction(double current_head) {
    double vertical_direction = current_head + 90.0;

    if (vertical_direction > 360.0)
        vertical_direction -= 360.0;
    else if (vertical_direction < 0.0)
        vertical_direction += 360.0;

    return vertical_direction;
}

#endif //_MATH_UTIL_H__

