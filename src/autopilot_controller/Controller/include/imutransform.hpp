#ifndef _IMUTRANSFORM_HPP__
#define _IMUTRANSFORM_HPP__

namespace imutransform {
/***
* 机体坐标系（b系）介绍：
* 机体坐标系（b）和机体固联，坐标原点在机体重心
* x轴沿机体的纵横方向，指向正前方
* y轴沿机体的横轴方向，指向正右方
* z轴沿机体竖直向下
* x,y,z构成右手坐标系
***/

// 更新方向余弦数值
void cosUpdate(const double Roll, const double Pitch, const double Yaw);

// 将地理坐标（n系）中的速度转换为导航坐标（b系）中的速度
// 输入： vn n系中速度在N方向的分量
// 输入： ve n系中速度在E方向的分量
// 输入： vd n系中速度在D方向的分量
// 输出： vx b系中速度在X轴的分量
// 输出： vy b系中速度在Y轴的分量
// 输出： vz b系中速度在Z轴的分量
void Vn2Vb(const double vn, const double ve, const double vd,
           double& vx, double& vy, double& vz);
// 将地理坐标（n系）中的加速度转换为导航坐标（b系）中的加速度
void An2Ab(const double an, const double ae, const double ad,
           double& ax, double& ay, double& az);

// 将载体坐标（b系）中角速度转换为欧拉角速度
// 输入： ARx b系中角速度在X轴的分量
// 输入： ARy b系中角速度在Y轴的分量
// 输入： ARz b系中角速度在Z轴的分量
// 输出： wr ω_roll  横滚角速度 roll_speed
// 输出： wp ω_pitch 俯仰角速度 pitch_speed
// 输出： wy ω_yaw   航向角速度 heading_speed
void Wb2Wn(const double ARx, const double ARy, const double ARz,
           double& wr, double& wp, double& wy);

void Quaternion2EulerianAngle(const double w, const double x, const double y, const double z,
                              double& Roll, double& Pitch, double& Yaw);
void EulerianAngle2Quaternion(const double Roll, const double Pitch, const double Yaw,
                              double& w, double& x, double& y, double& z);

}

#endif /* _IMUTRANSFORM_HPP__ */
