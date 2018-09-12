#include "imutransform.hpp"
#include <cmath>

namespace imutransform {
static double gRoll;  // 横滚角
static double gPitch; // 俯仰角
static double gYaw;   // 航向角 heading

// 方向余弦变量，用于产生方向余弦矩阵
static double gsinYaw, gcosYaw;
static double gsinPitch, gcosPitch, gtanPitch;
static double gsinRoll, gcosRoll;

// 更新方向余弦数值
void cosUpdate(const double Roll, const double Pitch, const double Yaw) {
    gRoll = Roll;
    gPitch = Pitch;
    gYaw = Yaw;

    gsinRoll = sin(gRoll);
    gcosRoll = cos(gRoll);

    gsinPitch = sin(gPitch);
    gcosPitch = cos(gPitch);
    gtanPitch = tan(gPitch);

    gsinYaw = sin(gYaw);
    gcosYaw = cos(gYaw);
}

// 将地理坐标（n系）中的速度转换为导航坐标（b系）中的速度
// 输入： vn n系中速度在N方向的分量
// 输入： ve n系中速度在E方向的分量
// 输入： vd n系中速度在D方向的分量
// 输出： vx b系中速度在X轴的分量
// 输出： vy b系中速度在Y轴的分量
// 输出： vz b系中速度在Z轴的分量
void Vn2Vb(const double vn, const double ve, const double vd,
           double& vx, double& vy, double& vz) {
    vx = vn * gcosYaw * gcosPitch + ve * gsinYaw * gcosPitch - vd * gsinPitch;
    vy = vn * (gcosYaw * gsinPitch * gsinRoll - gsinYaw * gcosRoll)
         + ve * (gsinYaw * gsinPitch * gsinRoll + gcosYaw * gcosRoll)
         + vd * gcosPitch * gsinRoll;
    vz = vn * (gcosYaw * gsinPitch * gcosRoll + gsinYaw * gsinRoll)
         + ve * (gsinYaw * gsinPitch * gcosRoll - gcosYaw * gsinRoll)
         + vd * gcosPitch * gcosRoll;
}

// 将地理坐标（n系）中的加速度转换为导航坐标（b系）中的加速度
void An2Ab(const double an, const double ae, const double ad,
           double& ax, double& ay, double& az) {
    Vn2Vb(an, ae, ad, ax, ay, az);
}

// 将载体坐标（b系）中角速度转换为欧拉角速度
// 输入： ARx b系中角速度在X轴的分量
// 输入： ARy b系中角速度在Y轴的分量
// 输入： ARz b系中角速度在Z轴的分量
// 输出： wr ω_roll  横滚角速度 roll_speed
// 输出： wp ω_pitch 俯仰角速度 pitch_speed
// 输出： wy ω_yaw   航向角速度 heading_speed
void Wb2Wn(const double ARx, const double ARy, const double ARz,
           double& wr, double& wp, double& wy) {
    wr = ARx + ARz * gcosRoll * gtanPitch - ARy * gsinRoll * gtanPitch;
    wp = gcosRoll * ARy - gsinRoll * ARz;
    wy = (gsinRoll * ARy + gcosRoll * ARz) / gcosPitch;
}


void Quaternion2EulerianAngle(const double w, const double x, const double y, const double z,
                              double& Roll, double& Pitch, double& Yaw) {
    const double ysqr = y * y;

    // Roll (x-axis rotation)
    const double t0 = +2.0 * (w * x + y * z);
    const double t1 = +1.0 - 2.0 * (x * x + ysqr);
    Roll = atan2(t0, t1);

    // Pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    if (t2 > 1.0)
        t2 = 1;
    else if (t2 < -1.0)
        t2 = -1.0;
    Pitch = asin(t2);

    // Yaw (z-axis rotation)
    const double t3 = +2.0 * (w * z + x * y);
    const double t4 = +1.0 - 2.0 * (ysqr + z * z);
    Yaw = atan2(t3, t4);
}

void EulerianAngle2Quaternion(const double Roll, const double Pitch, const double Yaw,
                              double& w, double& x, double& y, double& z) {
    const double t0 = cos(Yaw * 0.5);
    const double t1 = sin(Yaw * 0.5);
    const double t2 = cos(Roll * 0.5);
    const double t3 = sin(Roll * 0.5);
    const double t4 = cos(Pitch * 0.5);
    const double t5 = sin(Pitch * 0.5);

    w = t0 * t2 * t4 + t1 * t3 * t5;
    x = t0 * t3 * t4 - t1 * t2 * t5;
    y = t0 * t2 * t5 + t1 * t3 * t4;
    z = t1 * t2 * t4 - t0 * t3 * t5;
}

}
