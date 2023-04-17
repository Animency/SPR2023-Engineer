/**
 * @file gimbal_task.c/h
 * @author 何清华,周鸣阳
 * @brief 云台控制任务线程
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
 /**************************************/
 /******      ID使用说明：        ******/
 /******   0: 横轴平移电机        ******/
 /******   1：竖轴平移电机        ******/
 /******   2：机械臂水平旋转电机  ******/
 /******   3：机械臂竖直俯仰电机  ******/
 /******   4：侧方抬升电机        ******/
 /******   5：侧方抬升电机        ******/
 /**************************************/
 
 
 /******     2:向上角度增加     ********/
 /******    3:逆时针角度增加    ********/
 /******       4:电机正装       *******/
 /******       5:电机反装       *******/
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "arm_math.h"
#include "board_talk.h"
#include "tim.h"
#include "usart.h"
#include "chassis_task.h"
#include "gimbal_pid.h"

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }
	
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化，遥控器指针初始化，云台电机指针初始化，执行云台数据更新
 * @retval         none
 */
void gimbal_init(gimbal_control_t *gimbal_init);

void gimbal_feedback_update(gimbal_control_t *gimbal_move_update);

void gimbal_3508_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,float current_target_transfer);

void gimbal_3508_angle_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,int angle_target_transfer);

/*云台模式设置*/
void gimbal_set_mode(gimbal_control_t *gimbal_move_mode);
/*云台五个电机的控制量设置*/
void gimbal_heng(gimbal_control_t *gimbal_heng);
void gimbal_shu(gimbal_control_t *gimbal_shu);
void gimbal_xuan(gimbal_control_t *gimbal_xuan);
void gimbal_fan(gimbal_control_t *gimbal_fan);
void gimbal_tai(gimbal_control_t *gimbal_tai);
void gimbal_duoji(gimbal_control_t *gimbal_duoji);
/*机械臂角度闭环*/
void gimbal_jiaodubihuan_mechanical_arm(gimbal_control_t *gimbal_jiaodubihuan_mechanical_arm);
/*在不抬升时保持抬起角度不变*/
void gimbal_tai_keep_function(gimbal_control_t *gimbal_tai_keep_function);
/*简易连招设计*/
void gimbal_ore_convert(gimbal_control_t *gimbal_gimbal_ore_convert);
/*底盘运动时的云台保持*/
void gimbal_keep(gimbal_control_t *gimbal_keep);
/*定义电机角度变量*/
long long gimbal_0_angle = 0,gimbal_1_angle=0,gimbal_2_angle=0,gimbal_3_angle=0,gimbal_4_angle=0;
/*定义抬升角度变量*/
long long gimbal_1_angle_tai=0,gimbal_2_angle_tai=0;
/*定义各电机控制变量*/
float gimbal_heng_set=0,gimbal_shu_set=0,gimbal_xuan_set=0,gimbal_fan_set=0,gimbal_tai_set=0,gimbal_tai_keep=0;
/*定义舵机控制变量*/
int duoji=1000; //2000为最大值	
/*定义角度暂时值用于连招操作*/
float temporary_angle = 0;
float gimabl_1_angle_change = 50,gimabl_2_angle_change = 0,gimabl_3_angle_change = 1,gimabl_4_angle_change = 0,gimabl_5_angle_change = 0,gimbal_angle_tai_change = 50;
//云台控制所有相关数据
gimbal_control_t gimbal_control;
/*舵机控制变量设置*/
float REMOTE_CONTROL_CHANNEL_DUOJI = 30;
/*pid测试用变量*/
float pid_test_1 = 0;  //输入数据
float pid_test_2 = 0;  //输出数据

void gimbal_task(void const *pvParameters)
{
  vTaskDelay(20);
  //gimbal_init(&gimbal_control);
  while (1)
  {	
		gimbal_init(&gimbal_control);
		gimbal_set_mode(&gimbal_control);
		gimbal_feedback_update(&gimbal_control);
		CAN2_cmd_gimbal(gimbal_control .horizontal_scroll_motor[0].give_current ,gimbal_control .horizontal_scroll_motor[1].give_current,
		gimbal_control .horizontal_scroll_motor[2].give_current,gimbal_control .horizontal_scroll_motor[3].give_current); //gimbal_control .horizontal_scroll_motor .give_current
		CAN2_cmd_gimbal_tai(gimbal_control .horizontal_scroll_motor[4].give_current ,gimbal_control .horizontal_scroll_motor[5].give_current,0,0);
    vTaskDelay(20);
  }
}	
//云台初始化
void gimbal_init(gimbal_control_t *gimbal_init)
{
	unsigned int i=0;
	if (gimbal_init == NULL)
  {
    return;
  }
	//获取遥控器数据指针
	gimbal_init->gimbal_rc_ctrl= get_remote_control_point();
	
	for (i=0;i<6;++i)
	{
		//初始化云台水平移动电机数据指针
	gimbal_init->horizontal_scroll_motor[i] .gimbal_motor_measure  = get_gimbal_motor_measure_point(i);
		//初始化云台水平电机最大最低速度
	gimbal_init->horizontal_scroll_motor[i] .speed_max =NORMAL_MAX_GIMBAL_SPEED;
	gimbal_init->horizontal_scroll_motor[i] .speed_min =-NORMAL_MAX_GIMBAL_SPEED;
	}
	//云台电机速度环PID初始化
  PID_init(&gimbal_init->horizontal_scroll_motor[0] .gimbal_motor_speed_pid ,p_heng, 0,d_heng, 13000, 3000);
  PID_init(&gimbal_init->horizontal_scroll_motor[1] .gimbal_motor_speed_pid ,p_shu, i_shu,d_shu, 13000, 3000);
	PID_init(&gimbal_init->horizontal_scroll_motor[2] .gimbal_motor_speed_pid ,p_arm_horizon, 0,d_arm_horizon, max_out_arm_horizon, max_i_out_arm_horizon);
	PID_init(&gimbal_init->horizontal_scroll_motor[3] .gimbal_motor_speed_pid ,p_arm_vertical, 0,d_arm_vertical, max_out_arm_vertical, max_i_out_arm_vertical);
	PID_init(&gimbal_init->horizontal_scroll_motor[4] .gimbal_motor_speed_pid ,p_tai, 0,d_tai, 80000, 3000);
	PID_init(&gimbal_init->horizontal_scroll_motor[5] .gimbal_motor_speed_pid ,p_tai, 0,d_tai, 80000, 3000);
	//云台电机角度环PID初始化
	PID_init(&gimbal_init->horizontal_scroll_motor[0] .gimbal_motor_angle_pid ,p_heng_angle, 0,d_heng_angle, max_out_heng_angle, 0.2);
	PID_init(&gimbal_init->horizontal_scroll_motor[1] .gimbal_motor_angle_pid ,p_shu_angle, 0,d_shu_angle, max_out_shu_angle, 0.2);
	PID_init(&gimbal_init->horizontal_scroll_motor[2] .gimbal_motor_angle_pid ,p_arm_horizon_angle, 0,d_arm_horizon_angle, max_out_arm_horizon_angle, max_i_out_arm_horizon_angle);
	PID_init(&gimbal_init->horizontal_scroll_motor[3] .gimbal_motor_angle_pid ,p_arm_vertical_angle, 0,d_arm_vertical_angle, max_out_arm_vertical_angle, max_i_out_arm_vertical_angle);
	PID_init(&gimbal_init->horizontal_scroll_motor[4] .gimbal_motor_angle_pid ,p_tai_angle, 0,d_tai_angle, p_tai_angle_maxout, 0.2);
	PID_init(&gimbal_init->horizontal_scroll_motor[5] .gimbal_motor_angle_pid ,p_tai_angle, 0,d_tai_angle, p_tai_angle_maxout, 0.2);
	
	
	//云台数据更新
	gimbal_feedback_update(gimbal_init);
}
//云台模式设置
void gimbal_set_mode(gimbal_control_t *gimbal_move_mode)
{
	if (gimbal_move_mode == NULL)
  {
    return;
  }
	//左边摇杆为上操作云台
	 if (SWITCH_LEFT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
    {
//			//在使用角度闭环时注释
//			gimbal_heng(&gimbal_control);
			gimbal_shu(&gimbal_control);
//			gimbal_xuan(&gimbal_control);    
//			gimbal_fan(&gimbal_control);
			gimbal_jiaodubihuan_mechanical_arm(&gimbal_control);
			gimbal_tai_keep_function(&gimbal_control);
			 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
    }
		//左边摇杆为下操作抬升与舵机
  else if (SWITCH_LEFT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{   
			gimbal_tai(&gimbal_control);
		  gimbal_duoji(&gimbal_control);
	}
	else if (SWITCH_LEFT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{
			gimbal_ore_convert(&gimbal_control);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
	}
	//当底盘运动时
	else if (SWITCH_LEFT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{
		gimbal_keep(&gimbal_control);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
	}
}
void gimbal_feedback_update(gimbal_control_t *gimbal_move_update)
{
	int i=0;
	if (gimbal_move_update == NULL)
  {
    return;
  }
	//地盘电机速度电流更新
  for (i=0;i<6;++i)
	{
		gimbal_move_update->horizontal_scroll_motor[i] .motor_speed_current  = MOTOR_RPM_TO_SPEED * gimbal_move_update ->horizontal_scroll_motor[i] .gimbal_motor_measure->speed_rpm;
		//gimbal_move_update->horizontal_scroll_motor[i] .motor_speed_current  = MOTOR_RPM_TO_SPEED * gimbal_move_update ->horizontal_scroll_motor[i] .gimbal_motor_measure->speed_rpm;
	}
}
/**
*@brief         控制循环，根据速度控制设定值，计算电机电流值，进行控制
*@param[in]			*gimbal_move_control_loop: 云台电机数据指针
*@param[in]			i:电机编号
*@param[in]			current_target_transfer：云台电机目标速度值
**/
void gimbal_3508_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,float current_target_transfer)
{
	 fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
	float current_target=0;
    temp = fabs(gimbal_move_control_loop->horizontal_scroll_motor[i] .speed_set);
	current_target=current_target_transfer;//遥控器模式使用
    if (max_vector < temp)
    {
      max_vector = temp;
    }

  if (max_vector > NORMAL_MAX_GIMBAL_SPEED)
  {
    vector_rate = NORMAL_MAX_GIMBAL_SPEED / max_vector;
      gimbal_move_control_loop->horizontal_scroll_motor[i] .speed_set  *= vector_rate;
  }

  // calculate pid
  //计算pid
    PID_calc(&gimbal_move_control_loop->horizontal_scroll_motor[i].gimbal_motor_speed_pid , gimbal_move_control_loop->horizontal_scroll_motor[i] .motor_speed_current     , current_target );
		pid_test_1=gimbal_move_control_loop->horizontal_scroll_motor[1].gimbal_motor_measure->speed_rpm;
  //功率控制
  // chassis_power_control(chassis_move_control_loop);

  //赋值电流值
    gimbal_move_control_loop->horizontal_scroll_motor[i] .give_current  = (int16_t)(gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_speed_pid .out );   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
}
/**
*@brief         控制循环，根据角度控制设定值，计算电机电流值，进行控制
*@param[in]			*gimbal_move_control_loop: 云台电机数据指针
*@param[in]			i:电机编号
*@param[in]			angle_target_transfer：云台电机目标角度值
**/
void gimbal_3508_angle_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,int angle_target_transfer)
{
	 fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
	float current_target=0,angle_out=0;
    temp = fabs(gimbal_move_control_loop->horizontal_scroll_motor[i] .speed_set);
	current_target=angle_target_transfer;//遥控器模式使用

  //计算pid
	  angle_out=PID_calc(&gimbal_move_control_loop->horizontal_scroll_motor[i].gimbal_motor_angle_pid , gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_measure->angle , current_target );
    PID_calc(&gimbal_move_control_loop->horizontal_scroll_motor[i].gimbal_motor_speed_pid , gimbal_move_control_loop->horizontal_scroll_motor[i] .motor_speed_current, angle_out );
  //功率控制
  // chassis_power_control(chassis_move_control_loop);

  //赋值电流值
    gimbal_move_control_loop->horizontal_scroll_motor[i] .give_current  = (int16_t)(gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_speed_pid .out );   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
}

//电机角度闭环控制
void gimbal_jiaodubihuan_mechanical_arm(gimbal_control_t *gimbal_jiaodubihuan_mechanical_arm)
{
	gimbal_0_angle+=gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_RIGHT_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-600);
	gimbal_1_angle+=-gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-500);
	gimbal_2_angle+=gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [YAW_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-500);
	gimbal_3_angle+=gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-500);
	gimbal_3508_angle_control_loop(&gimbal_control,0,gimbal_0_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,1,gimbal_1_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,2,gimbal_2_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,3,gimbal_3_angle);
}

//电机抬升角度闭环控制
void gimbal_jiaoduhuan(gimbal_control_t *gimbal_jiaoduhuan)
{
	gimbal_1_angle_tai+= gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]/(REMOTE_CONTROL_CHANNEL_LONG-600);
	gimbal_2_angle_tai+=-gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]/(REMOTE_CONTROL_CHANNEL_LONG-600);//+gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]/660
	gimbal_3508_angle_control_loop(&gimbal_control,4,gimbal_1_angle_tai);
	gimbal_3508_angle_control_loop(&gimbal_control,5,gimbal_2_angle_tai);
}

//横向移动控制
void gimbal_heng(gimbal_control_t *gimbal_heng)
{
	gimbal_heng_set=-gimbal_heng->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_RIGHT_CHANNEL]*GIMBAL_VEL;
	gimbal_3508_control_loop(&gimbal_control,0,gimbal_heng_set);
	gimbal_heng_set=0;
	
}

//竖向移动控制
void gimbal_shu(gimbal_control_t *gimbal_shu)
{
	gimbal_shu_set=-gimbal_shu->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]*GIMBAL_VEL;
	gimbal_3508_control_loop(&gimbal_control,1,gimbal_shu_set);
	gimbal_shu_set=0;
}

//小臂旋转控制
void gimbal_xuan(gimbal_control_t *gimbal_xuan)
{
	gimbal_xuan_set=-gimbal_xuan->gimbal_rc_ctrl ->rc .ch [YAW_CHANNEL]*GIMBAL_VEL*0.8;
	gimbal_3508_control_loop(&gimbal_control,2,gimbal_xuan_set);
	gimbal_xuan_set=0;
}

//小臂翻转控制
void gimbal_fan(gimbal_control_t *gimbal_fan)
{
	gimbal_fan_set=gimbal_fan->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]*GIMBAL_VEL*0.8;
	gimbal_3508_control_loop(&gimbal_control,3,gimbal_fan_set);
	gimbal_fan_set=0;
}

//云台抬升控制
void gimbal_tai(gimbal_control_t *gimbal_tai)
{
//  //使用双环时注释
//	if (gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]>100||gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]<-100)
//	gimbal_tai_set=gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]*GIMBAL_VEL*0.8*(-1);
//	else
//	gimbal_tai_set=gimbal_tai_keep;	
//	gimbal_3508_control_loop(&gimbal_control,4,gimbal_tai_set);
//	gimbal_3508_control_loop(&gimbal_control,5,-gimbal_tai_set);
	pid_test_2 = gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_RIGHT_CHANNEL]*GIMBAL_VEL;
	gimbal_jiaoduhuan(&gimbal_control);
}

//舵机控制
void gimbal_duoji(gimbal_control_t *gimbal_duoji)
{
	if(gimbal_duoji->gimbal_rc_ctrl->rc.ch[GIMBAL_MODE_RIGHT_CHANNEL] == 0)
	{
		duoji = 1500;
	}
	else if(gimbal_duoji->gimbal_rc_ctrl->rc.ch[GIMBAL_MODE_RIGHT_CHANNEL] >100)
	{
		duoji = 2500;
	}
	else if(gimbal_duoji->gimbal_rc_ctrl->rc.ch[GIMBAL_MODE_RIGHT_CHANNEL] < -100)
	{
		duoji = 500;
	}
	//堵转限制
//	else if(duoji >= 2000)
//	{
//		duoji = 2000;
//	}
//	else if(duoji <= 0)
//	{
//		duoji = 0;
//	}
//	else
//	{
//		duoji = 1000;
//	}
//			//duoji += gimbal_duoji->gimbal_rc_ctrl->rc.ch[GIMBAL_ZUOCE_CHANNEL_LEFT_RIGHT]/REMOTE_CONTROL_CHANNEL_DUOJI;
	   // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
}
void gimbal_tai_keep_function(gimbal_control_t *gimbal_tai_keep_function)
{
	gimbal_3508_angle_control_loop(&gimbal_control,4,gimbal_1_angle_tai);
	gimbal_3508_angle_control_loop(&gimbal_control,5,gimbal_2_angle_tai);
}

//递入矿石连招
void gimbal_ore_convert(gimbal_control_t *gimbal_gimbal_ore_convert)
{
	if((gimbal_1_angle_tai >= 100) && (temporary_angle != gimbal_1_angle_tai+gimbal_angle_tai_change))
	{
		temporary_angle = gimbal_1_angle_tai;
		gimbal_1_angle_tai -= gimbal_angle_tai_change ;
		gimbal_2_angle_tai += gimbal_angle_tai_change ;
		gimbal_3_angle += gimabl_3_angle_change;
		gimbal_1_angle -= gimabl_1_angle_change;
	}
	gimbal_3508_angle_control_loop(&gimbal_control,1,gimbal_1_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,4,gimbal_1_angle_tai);
	gimbal_3508_angle_control_loop(&gimbal_control,5,gimbal_2_angle_tai);
	gimbal_3508_angle_control_loop(&gimbal_control,3,gimbal_3_angle);
}
//底盘运动时的云台保持
void gimbal_keep(gimbal_control_t *gimbal_keep)
{
	gimbal_3508_angle_control_loop(&gimbal_control,0,gimbal_0_angle);
	gimbal_3508_control_loop(&gimbal_control,1,gimbal_shu_set);
	gimbal_shu_set=0;
	gimbal_3508_angle_control_loop(&gimbal_control,2,gimbal_2_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,3,gimbal_3_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,4,gimbal_1_angle_tai);
	gimbal_3508_angle_control_loop(&gimbal_control,5,gimbal_2_angle_tai);
}