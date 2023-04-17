/**
 * @file filter.c/h
 * @author 何清华；周鸣阳
 * @brief 包含多种滤波器
 * @version 1.0
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022 SPR
 * 
 */

#include "filter.h"

//卡尔曼滤波
/*        
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好        
*/
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    static double x_last;

    double x_mid = x_last;
    double x_now;

    static double p_last=1;

    double p_mid;
    double p_now;
    double kg;

    x_mid = x_last;                           //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q;                       //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    kg = p_mid / (p_mid + R);                 //kg为kalman filter，R为噪声
    x_now = x_mid + kg * (ResrcData - x_mid); //估计出的最优值

    p_now = (1 - kg) * p_mid; //最优值对应的covariance

    p_last = p_now; //更新covariance值
    x_last = x_now; //更新系统状态值

    return x_now;
}

int FilterI=0;
//ADNum为获得的AD数
//n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
int GlideFilterAD(int value_buf[],int n,int ADNum)
{
    int sum=0; 
    value_buf[FilterI++]=ADNum;
    if(FilterI==n) FilterI=0; //先进先出，再求平均值
    for(int count=0;count<n;count++)
        sum+=value_buf[count];
    return (int)(sum/n);
}
