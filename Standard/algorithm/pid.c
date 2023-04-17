#include "algorithm.h"

//位置式pid参数初始化
void PID_init(PID_ABS_t *pid, float Kp, float Ki, float Kd, float Max_error, float Max_output)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Max_error = Max_error;
	pid->Max_output = Max_output;
}
//位置式pid算法
void PID_Calc(PID_ABS_t *pid, float current_parament, float target_parament)
{
	pid->error[0] = pid->error[1];
	pid->error[1] = target_parament - current_parament;
	pid->error[2] += pid->error[1];

	if (pid->error[2] > pid->Max_error)
		pid->error[2] = pid->Max_error;
	else if (pid->error[2] < -pid->Max_error)
		pid->error[2] = -pid->Max_error;

	pid->Output = pid->Kp * pid->error[1] + pid->Ki * pid->error[2] + pid->Kd * (pid->error[1] - pid->error[0]);

	if (pid->Output >= pid->Max_output)
		pid->Output = pid->Max_output;
	else if (pid->Output <= -pid->Max_output)
		pid->Output = -pid->Max_output;
}
