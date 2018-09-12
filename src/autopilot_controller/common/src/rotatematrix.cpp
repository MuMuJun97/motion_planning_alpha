#include <string.h>
#include <stdio.h>
#include "rotatematrix.h"

// 变换到局部坐标系
// Z 是顶端，  X 是侧方，  Y 是前方纵深。
void tranform_to_local_fun(Point p, Point &p_local, double roll, double pitch,
                           double heading, double dx, double dy, double dz) {
//	unsigned int i, j;
    double dbPhi, dbOmega, dbKappa;
    double Values[9];
    double pp[3], p0[3], result[3];

    dbPhi = to_radians(roll);      //Phi   是横滚角 roll
    dbOmega = to_radians(pitch);   //Omega 是俯仰角 pitch
    dbKappa = to_radians(-heading); //Kappa 是航向角 heading

    // CalRotateMatrix(dbPhi, dbOmega, dbKappa, m);
    CalRotateMatrix_c(dbPhi, dbOmega, dbKappa, Values);
    pp[0] = p.x;
    pp[1] = p.y;
    pp[2] = p.z;
    p0[0] = dx;
    p0[1] = dy;
    p0[2] = dz;

    //result = mul_matrix(p, m) + p0;
    Rotate_Point_by_Matrix(pp[0], pp[1], pp[2],  // p0
                           p0[0], p0[1], p0[2],  // p
                           Values, &(result[0]), &(result[1]), &(result[2]));

    p_local.xx = result[0];
    p_local.yy = result[1];
    p_local.zz = result[2];

}

//计算旋转矩阵的函数
double GetAverage(double *pfValues, int nCount) {
    double dbSum = 0;
    for (int i = 0; i < nCount; i++) {
        dbSum += pfValues[i];
    }

    return dbSum / nCount;
}

//计算算数平均值的函数
void CalRotateMatrix_c(double dbPhi, double dbOmega, double dbKappa,
                       double *pdValues) {
    pdValues[0] = cos(dbPhi) * cos(dbKappa)
                  - sin(dbPhi) * sin(dbOmega) * sin(dbKappa);
    pdValues[1] = -cos(dbPhi) * sin(dbKappa)
                  - sin(dbPhi) * sin(dbOmega) * cos(dbKappa);
    pdValues[2] = -sin(dbPhi) * cos(dbOmega);
    pdValues[3] = cos(dbOmega) * sin(dbKappa);
    pdValues[4] = cos(dbOmega) * cos(dbKappa);
    pdValues[5] = -sin(dbOmega);
    pdValues[6] = sin(dbPhi) * cos(dbKappa)
                  + cos(dbPhi) * sin(dbOmega) * sin(dbKappa);
    pdValues[7] = -sin(dbPhi) * sin(dbKappa)
                  + cos(dbPhi) * sin(dbOmega) * cos(dbKappa);
    pdValues[8] = cos(dbPhi) * cos(dbOmega);
}

//计算系数矩阵转置与常数项乘积矩阵的函数
void Rotate_Point_by_Matrix(double Geox1, double Geoy1, double Geoz1,  // p0
                            double Geox2, double Geoy2, double Geoz2,  // p
                            double *pfRotateMat, double *p_x, double *p_y, double *p_z) {
    double dbTemp1 = pfRotateMat[0] * Geox1 + pfRotateMat[1] * Geoy1
                     + pfRotateMat[2] * Geoz1;
    double dbTemp2 = pfRotateMat[3] * Geox1 + pfRotateMat[4] * Geoy1
                     + pfRotateMat[5] * Geoz1;
    double dbTemp3 = pfRotateMat[6] * Geox1 + pfRotateMat[7] * Geoy1
                     + pfRotateMat[8] * Geoz1;

    *p_x = Geox2 + dbTemp1;
    *p_y = Geoy2 + dbTemp2;
    *p_z = Geoz2 + dbTemp3;
}

//计算系数矩阵转置与常数项乘积矩阵的函数
void CalConstMatrix(double *pGeox1, double *pGeoy1, double *pGeoz1,  // p0
                    double *pGeox2, double *pGeoy2, double *pGeoz2,  // p
                    int nPtCount, double *pfRotateMat, double dbK, double *pfCoffMat) {
    double dbTempX = 0;
    double dbTempY = 0;
    double dbTempZ = 0;
    double dbTempW = 0;

    for (int i = 0; i < nPtCount; i++) {
        double dbTemp1 = pfRotateMat[0] * pGeox2[i] + pfRotateMat[1] * pGeoy2[i]
                         + pfRotateMat[2] * pGeoz2[i];
        double dbTemp2 = pfRotateMat[3] * pGeox2[i] + pfRotateMat[4] * pGeoy2[i]
                         + pfRotateMat[5] * pGeoz2[i];
        double dbTemp3 = pfRotateMat[6] * pGeox2[i] + pfRotateMat[7] * pGeoy2[i]
                         + pfRotateMat[8] * pGeoz2[i];

        double dbLx = pGeox1[i] - dbK * dbTemp1;
        double dbLy = pGeoy1[i] - dbK * dbTemp2;
        double dbLz = pGeoz1[i] - dbK * dbTemp3;

        dbTempX += (pGeox2[i] * dbLx + pGeoy2[i] * dbLy + pGeoz2[i] * dbLz);
        dbTempY += (pGeox2[i] * dbLz - pGeoz2[i] * dbLx);
        dbTempZ += (pGeoy2[i] * dbLz - pGeoz2[i] * dbLy);
        dbTempW += (pGeox2[i] * dbLy - pGeoy2[i] * dbLx);
    }

    pfCoffMat[0] = 0;
    pfCoffMat[1] = 0;
    pfCoffMat[2] = 0;

    pfCoffMat[3] = dbTempX;
    pfCoffMat[4] = dbTempY;
    pfCoffMat[5] = dbTempZ;
    pfCoffMat[6] = dbTempW;

}

//计算系数矩阵转置的乘积矩阵的函数
void CalCoffMatrix(double *pGeox1, double *pGeoy1, double *pGeoz1, int nPtCount,
                   double *pfRotateMartix, double *pfCoffMat) {
    //计算矩阵元素
    pfCoffMat[0] = nPtCount;
    pfCoffMat[8] = nPtCount;
    pfCoffMat[16] = nPtCount;

    double dbTemp1 = 0;
    double dbTemp2 = 0;
    double dbTemp3 = 0;
    double dbTemp4 = 0;
    double dbTemp5 = 0;
    double dbTemp6 = 0;
    double dbTemp7 = 0;
    double dbTemp8 = 0;
    double dbTemp9 = 0;
    double dbTemp10 = 0;

    for (int i = 0; i < nPtCount; i++) {
        dbTemp1 += (pGeox1[i] * pGeox1[i] + pGeoy1[i] * pGeoy1[i]
                    + pGeoz1[i] * pGeoz1[i]);
        dbTemp2 += (pGeox1[i] * pGeox1[i] + pGeoz1[i] * pGeoz1[i]);
        dbTemp3 += (pGeox1[i] * pGeoy1[i]);
        dbTemp4 += (pGeoy1[i] * pGeoz1[i]);
        dbTemp5 += (pGeox1[i] * pGeoy1[i]);
        dbTemp6 += (pGeoy1[i] * pGeoy1[i] + pGeoz1[i] * pGeoz1[i]);
        dbTemp7 += (pGeox1[i] * pGeoz1[i]);
        dbTemp8 += (pGeoy1[i] * pGeoz1[i]);
        dbTemp9 += (pGeox1[i] * pGeoz1[i]);
        dbTemp10 += (pGeox1[i] * pGeox1[i] + pGeoy1[i] * pGeoy1[i]);
    }

    pfCoffMat[24] = dbTemp1;

    pfCoffMat[32] = dbTemp2;
    pfCoffMat[33] = dbTemp3;
    pfCoffMat[34] = dbTemp4;

    pfCoffMat[39] = dbTemp5;
    pfCoffMat[40] = dbTemp6;
    pfCoffMat[41] = -dbTemp7;

    pfCoffMat[46] = dbTemp8;
    pfCoffMat[47] = -dbTemp9;
    pfCoffMat[48] = dbTemp10;
}

//把相对坐标系转化为绝对坐标系
void tranform_to_global(double yaw, double pitch, double roll,  //传感器指向
                        double x0, double y0, double z0,    //传感器绝对坐标
                        double x, double y, double z,     //点的相对坐标（传感器坐标系下）
                        double &xx, double &yy, double &zz)  //点的绝对坐标（全局坐标系下）  //Output
{
    double dbPhi, dbOmega, dbKappa;
    double Values[9], result[3];

    dbPhi = to_radians(roll);    //Phi   是横滚角 roll
    dbOmega = to_radians(pitch); //Omega 是俯仰角 pitch
    dbKappa = to_radians(-yaw); //Kappa 是航向角 heading

    CalRotateMatrix_c(dbPhi, dbOmega, dbKappa, Values);

    //result = mul_matrix(p, m) + p0;
    Rotate_Point_by_Matrix(
            x, y, z,  // p
            x0, y0, z0, // p0
            Values,
            &(result[0]), &(result[1]), &(result[2]));

    // result = p + p0;
    xx = result[0];
    yy = result[1];
    zz = result[2];
}

//  入口函数
/*
 bool SevenParameterSolve(double* pGeox1,
 double* pGeoy1,
 double* pGeoz1,
 double* pGeox2,
 double* pGeoy2,
 double* pGeoz2,
 int nPtCount,
 double* pdX,
 double* pdY,
 double* pdZ,
 double* pdK,
 double* pdPhi,
 double* pdOmige,
 double* pdKappa)
 {
 if (nPtCount < 3)
 {
 *pdX = 0;
 *pdY = 0;
 *pdZ = 0;
 *pdK = 1;
 *pdPhi = 0;
 *pdOmige = 0;
 *pdKappa = 0;
 return false;
 }

 //计算坐标的平均值
 double dbAvgX1 = 0;
 double dbAvgY1 = 0;
 double dbAvgZ1 = 0;
 double dbAvgX2 = 0;
 double dbAvgY2 = 0;
 double dbAvgZ2 = 0;

 dbAvgX1 = GetAverage(pGeox1,nPtCount);
 dbAvgY1 = GetAverage(pGeoy1,nPtCount);
 dbAvgZ1 = GetAverage(pGeoz1,nPtCount);

 //计算原始坐标系统的中心坐标
 dbAvgX2 = GetAverage(pGeox2,nPtCount);
 dbAvgY2 = GetAverage(pGeoy2,nPtCount);
 dbAvgZ2 = GetAverage(pGeoz2,nPtCount);

 //计算重心化的坐标
 int i = 0;
 for (i = 0; i < nPtCount; i ++)
 {
 pGeox1[i] -= dbAvgX1;
 pGeoy1[i] -= dbAvgY1;
 pGeoz1[i] -= dbAvgZ1;

 pGeox2[i] -= dbAvgX2;
 pGeoy2[i] -= dbAvgY2;
 pGeoz2[i] -= dbAvgZ2;
 }

 //变换参数初始值
 *pdX = 0;
 *pdY = 0;
 *pdZ = 0;
 *pdK = 1;
 *pdPhi = 0;
 *pdOmige = 0;
 *pdKappa = 0;

 //增量
 double dX = *pdX;
 double dY = *pdY;
 double dZ = *pdZ;
 double dK = *pdK;
 double dPhi = *pdPhi;
 double dOmige = *pdOmige;
 double dKappa = *pdKappa;

 double adfRotateMatrix[9];
 memset(adfRotateMatrix,0,sizeof(double)*9);

 //计算旋转矩阵
 double adfRotateMatrixInit[9];
 CalRotateMatrix_c(dPhi,dOmige,dKappa,adfRotateMatrixInit);

 //常数项矩阵
 double adfConstMatrix[7];
 memset(adfConstMatrix,0,sizeof(double)*7);

 //系数矩阵转置乘积矩阵 7*7
 double adfCoffMatrix[49];
 memset(adfCoffMatrix,0,sizeof(double)*49);

 //解矩阵
 double adfSolveMat[7];
 memset(adfSolveMat,0,sizeof(double)*7);

 int nCount = 0;

 while(1)
 {
 //计算旋转矩阵
 CalRotateMatrix(dPhi,dOmige,dKappa,adfRotateMatrix);

 //计算常数项
 CalConstMatrix(pGeox1,pGeoy1,pGeoz1,pGeox2,pGeoy2,pGeoz2,nPtCount,
 adfRotateMatrix,dK,adfConstMatrix);

 //计算系数矩阵
 CalCoffMatrix(pGeox2,pGeoy2,pGeoz2,nPtCount,adfRotateMatrix,adfCoffMatrix);

 //法方程求解
 MatrixInverse(adfCoffMatrix,7);
 MatrixMult(adfCoffMatrix,adfConstMatrix,adfSolveMat,7,7,1);

 //计算待定参数的新值
 dX = dX + adfSolveMat[0];
 dY = dY + adfSolveMat[1];
 dZ = dZ + adfSolveMat[2];
 dK = dK * (1 + adfSolveMat[3]);
 dPhi = dPhi + adfSolveMat[4];
 dOmige = dOmige + adfSolveMat[5];
 dKappa = dKappa + adfSolveMat[6];

 nCount++;

 if (fabs(adfSolveMat[4]) < 3e-7 && fabs(adfSolveMat[5]) < 3e-7 && fabs(adfSolveMat[6]) < 3e-7)
 {
 break;
 }

 memset(adfConstMatrix,0,sizeof(double)*7);
 memset(adfRotateMatrix,0,sizeof(double)*9);
 memset(adfCoffMatrix,0,sizeof(double)*49);
 memset(adfSolveMat,0,sizeof(double)*7);

 }

 *pdK = dK;
 *pdPhi = dPhi;
 *pdOmige = dOmige;
 *pdKappa = dKappa;

 CalRotateMatrix(dPhi,dOmige,dKappa,adfRotateMatrix);

 double dbTemp1[3];
 double dbTemp2[3];
 dbTemp1[0] = dbAvgX2;
 dbTemp1[1] = dbAvgY2;
 dbTemp1[2] = dbAvgZ2;
 MatrixMult(adfRotateMatrix,dbTemp1,dbTemp2,3,3,1);

 //平移量单独计算

 dX = dbAvgX1 - dK*dbTemp2[0];
 dY = dbAvgY1 - dK*dbTemp2[1];
 dZ = dbAvgZ1 - dK*dbTemp2[2];
 *pdX = dX;
 *pdY = dY;
 *pdZ = dZ;

 printf("%f,%f,%f,%.15f,%.15f,%.15f,%.15f\n",dX,dY,dZ,dOmige,dPhi,dKappa,dK);


 printf("迭代次数：%d\n",nCount);

 return true;
 }
 */
