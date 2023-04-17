/**
 * @file pid.c/h
 * @author 何清华
 * @brief pid实现函数，包括初始化，PID计算函数
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "pid.h"
#include "main.h"
#include "user_lib.h"

extern float variate;
#define LimitMax(input, max)   \
	{                          \
		if (input > max)       \
		{                      \
			input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
			input = -max;      \
		}                      \
	}

/**
  * @brief          pid结构数据初始化
  * @param[out]     pid: PID结构数据指针
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
 */
void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float max_out, float max_iout)
{
	if (pid == NULL)
	{
		return;
	}
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
float PID_calc(pid_type_def *pid, float ref, float set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;

	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);

	variate+=0.005;
	return pid->out;
}

/**
 * @brief 云台PID计算函数
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta 误差微分项，直接从传感器读取，不计算
 * @return fp32
 */
float chassis_follow_PID_calc(pid_type_def *pid, float get, float set, float error_delta)
{
  float err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->fdb = get;
  pid->set = set;

  err = set - get;
  pid->error[0] = rad_format(err);
  pid->Pout = pid->Kp * pid->error[0];
  pid->Iout += pid->Ki * pid->error[0];
  pid->Dout = pid->Kd * error_delta;
  LimitMax(pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  LimitMax(pid->out, pid->max_out);
  return pid->out;
}
