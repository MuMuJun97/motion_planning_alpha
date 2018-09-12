#ifndef _ROTATEMATRIX_H__
#define _ROTATEMATRIX_H__

#include <cmath>
#include "math_util.h"
#include "Point.h"

void tranform_to_local_c(Point p, Point &p_local, double roll, double pitch,
                         double heading, double dx, double dy, double dz);

//计算旋转矩阵的函数
double GetAverage(double *pfValues, int nCount);

//计算算数平均值的函数
void CalRotateMatrix_c(double dbPhi, double dbOmega, double dbKappa,
                       double *pdValues);

//计算系数矩阵转置与常数项乘积矩阵的函数
void CalConstMatrix(double *pGeox1, double *pGeoy1, double *pGeoz1,
                    double *pGeox2, double *pGeoy2, double *pGeoz2, int nPtCount,
                    double *pfRotateMat, double dbK, double *pfCoffMat);

void Rotate_Point_by_Matrix(double Geox1, double Geoy1, double Geoz1,  // p0
                            double Geox2, double Geoy2, double Geoz2,  // p
                            double *pfRotateMat, double *p_x, double *p_y, double *p_z);

//计算系数矩阵转置的乘积矩阵的函数
void CalCoffMatrix(double *pGeox1, double *pGeoy1, double *pGeoz1, int nPtCount,
                   double *pfRotateMartix, double *pfCoffMat);

//  入口函数
bool SevenParameterSolve(double *pGeox1, double *pGeoy1, double *pGeoz1,
                         double *pGeox2, double *pGeoy2, double *pGeoz2, int nPtCount,
                         double *pdX, double *pdY, double *pdZ, double *pdK, double *pdPhi,
                         double *pdOmige, double *pdKappa);

void tranform_to_global(double yaw, double pitch, double roll,  //传感器指向
                        double x0, double y0, double z0,    //传感器绝对坐标
                        double x, double y, double z,     //点的相对坐标（传感器坐标系下）
                        double &xx, double &yy, double &zz);  //点的绝对坐标（全局坐标系下）  //Output

#endif /*_ROTATEMATRIX_H__*/
