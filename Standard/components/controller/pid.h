#ifndef PID_H
#define PID_H

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
  * @brief          pid结构数据初始化
  * 
  * @param[out]     pid: PID结构数据指针
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
 */
extern void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float max_out, float max_iout);

/**
 * @brief          pid计算
 * 
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
extern float PID_calc(pid_type_def *pid, float ref, float set);

extern float chassis_follow_PID_calc(pid_type_def *pid, float get, float set, float error_delta);

#endif
