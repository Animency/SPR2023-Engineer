#ifndef __ALGORITHM_H
#define __ALGORITHM_H


#include "stm32f4xx_hal.h"

/****************PID*******************/
typedef __packed struct
{
	float Kp;
	float Ki;
	float Kd;
	float error[3];
	float Max_error;
	float Output;
	float Max_output;
} PID_ABS_t; //PID结构体声明
void PID_init(PID_ABS_t *pid, float Kp, float Ki, float Kd, float Max_error, float Max_output);
void PID_Calc(PID_ABS_t *pid, float current_parament, float target_parament);
float Camera_PID_Position(float new_inc,float error_v[],float inc[]);

/****************Filter***************/
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R, double InitialPrediction);
int GlideFilterAD(int value_buf[],int n,int ADNum);

/****************FSM***************/
typedef struct 
{
    int CurState;          //当前状态
    int event;             //事件
    int NextState;         //下一个状态
    void (*eventActFun)(); //函数指针
} FsmTable;
/*状态机类型*/
typedef struct
{
    int curState;          //当前状态
    FsmTable *pFsmTable; //状态表
    int size;              //表的项数
    int transfer_flag;
} FSM;
void FSM_Regist(FSM *pFsm, FsmTable *pTable);
void FSM_EventHandle(FSM *pFsm, int event);

#endif

