/**
 * @file pid.c/h
 * @author ���廪
 * @brief pidʵ�ֺ�����������ʼ����PID���㺯��
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
  * @brief          pid�ṹ���ݳ�ʼ��
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
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
 * @brief          pid����
 * @param[out]     pid: PID�ṹ����ָ��
 * @param[in]      ref: ��������
 * @param[in]      set: �趨ֵ
 * @return         pid���
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
 * @brief ��̨PID���㺯��
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta ���΢���ֱ�ӴӴ�������ȡ��������
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
