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

#ifndef ALGORITHM_H
#define ALGORITHM_H

/**
 * @brief 卡尔曼滤波
 * 
 * @param[in] ResrcData 
 * @param[in] ProcessNiose_Q 
 * @param[in] MeasureNoise_R 
 * @param[in] InitialPrediction 
 * @return double 
 */
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);

/**
 * @brief 
 * 
 * @param[in] value_buf 
 * @param[in] n 
 * @param[in] ADNum 
 * @return int 
 */
int GlideFilterAD(int value_buf[],int n,int ADNum);

#endif
