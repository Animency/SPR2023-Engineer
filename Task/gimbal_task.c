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
 /******      编号使用说明：        ******/  //可+1转换为ID
 /******   0: 横轴平移电机        ******/
 /******   1：竖轴平移电机        ******/
 /******   2：机械臂水平旋转电机  ******/   //已修改为6020且发送ID为3，标识符为0x1ff,接收ID为0x207
 /******   3：机械臂竖直俯仰电机  ******/
 /******   4：侧方抬升电机        ******/
 /******   5：侧方抬升电机        ******/
 /******   6：存矿抬升电机        ******/
 /**************************************/
 
 
 /******     2:向上角度增加     ********/
 /******    3:逆时针角度增加    ********/
 /******       4:电机正装       *******/
 /******       5:电机反装       *******/
 
  /******     T1-C1 舵机    ********/
 /******      T1-C2 气泵    ********/
 /******      T1-C3 气泵       *******/
 /******       5:电机反装       *******/
 
 /*************************************************************************/                      
 /******                     遥控器通道说明(左通道)                  ******/  
 /******       3 臂竖直旋转电机 3508 204                             ******/
 /******               |||                                           ******/
 /******               |||                                           ******/
 /******               |||                                           ******/   
 /******    ===========|||============  2 臂水平旋转电机 6020 207    ******/
 /******               |||                                           ******/
 /******               |||                                           ******/
 /******               |||                                           ******/
 /*************************************************************************/
 
 
 /**************************************************************************/                      
 /******                      遥控器通道说明(右通道)                  ******/  
 /******       1 竖轴平移电机  3508 202                               ******/
 /******               |||                                            ******/
 /******               |||                                            ******/
 /******               |||                                            ******/   
 /******    ===========|||============   0: 横轴平移电机 3508 201     ******/
 /******               |||                                            ******/
 /******               |||                                            ******/
 /******               |||                                            ******/
 /**************************************************************************/
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
#include "detect_task.h"
#include "Engineer_behaviour.h"
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

	//从Engineer_behaviour中取回各电机角度控制变量
extern float target_can2_201_angle;
extern float target_can2_202_angle;
extern float target_can2_204_angle;
extern float target_can2_205_angle;
extern float target_can2_206_angle;
extern float target_can2_207_angle_6020;
extern int target_can2_208_angle;	
extern float relative_angle_to_mechanical_arm;
extern mouse_keyboard_state_flag_t mouse_keyboard_state_flag;
extern RC_ctrl_t rc_ctrl;
extern uint8_t coordinates_control_flag;
extern int low_speed;
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化，遥控器指针初始化，云台电机指针初始化，执行云台数据更新
 * @retval         none
 */
void gimbal_init(gimbal_control_t *gimbal_init);
	//电机pid初始化
static void gimbal_pid_init(gimbal_control_t *gimbal_pid_init);
void gimbal_feedback_update(gimbal_control_t *gimbal_move_update);
void gimbal_mouse_keyboard_identify(void);
static void gimbal_3508_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,float current_target_transfer);
extern float PIID_calc(pid_type_def *pid, float ref, float set);
static void gimbal_3508_angle_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,int angle_target_transfer);
static void gimbal_6020_angle_control_loop(gimbal_control_t *gimbal_6020_control_loop,int16_t i,float current_target_transfer);
/*云台模式设置*/
void gimbal_set_mode(gimbal_control_t *gimbal_move_mode);
/*云台五个电机的控制量设置*/
static void gimbal_heng(gimbal_control_t *gimbal_heng);
static void gimbal_shu(gimbal_control_t *gimbal_shu);
static void gimbal_xuan(gimbal_control_t *gimbal_xuan);
static void gimbal_fan(gimbal_control_t *gimbal_fan);
static void gimbal_tai(gimbal_control_t *gimbal_tai);
static void gimbal_duoji(gimbal_control_t *gimbal_duoji);
static void gimbal_ore(gimbal_control_t *gimbal_ore);
/*机械臂角度闭环*/
static void gimbal_jiaodubihuan_mechanical_arm(gimbal_control_t *gimbal_jiaodubihuan_mechanical_arm);
/*电机抬升角度环*/
static void gimbal_jiaoduhuan(gimbal_control_t *gimbal_jiaoduhuan);
/*底盘运动时的云台保持*/
static void gimbal_keep(gimbal_control_t *gimbal_keep);
/*定义各电机控制变量*/
float gimbal_heng_set=0,gimbal_shu_set=0,gimbal_xuan_set=0,gimbal_fan_set=0,gimbal_tai_set=0,gimbal_tai_keep=0;
/*电机抬升角度环*/
static void gimbal_jiaoduhuan(gimbal_control_t *gimbal_jiaoduhuan);
/*电机循环控制标志位设置*/
//static void gimbal_control_loop_flag(uint16_t flag_location, *gimbal_control_loop_flag_t control_flag_1);

/*定义舵机控制变量*/
int duoji=1500; //2000为最大值	
/*定义存矿控制变量*/

//云台控制所有相关数据
gimbal_control_t gimbal_control;
/*气泵控制*/
uint8_t Pump_Flag,Pump_Control_Flag;
uint16_t Pump_Value[2] = {0,19999};
uint8_t key_c_flag;
uint8_t key_b_flag;
/*pid测试用变量*/
float pid_test_1 = 0;  //输入数据
float pid_test_2 = 0;  //输出数据
/*连招状态机取回函数*/
extern void fsm_run(void);
extern void fsm_init(void);
void gimbal_task(void const *pvParameters)
{
  vTaskDelay(20);
	fsm_init();
	gimbal_pid_init(&gimbal_control);
  while (1)
  {	
		gimbal_init(&gimbal_control);
		gimbal_set_mode(&gimbal_control);
		gimbal_feedback_update(&gimbal_control);
		if (toe_is_error(DBUS_TOE))
    {
      CAN_cmd_chassis(0,0,0,0);
			CAN2_cmd_gimbal(0,0,0,0);
			CAN2_cmd_gimbal_tai(0,0,0,0);
    }
		else
		{
		
		CAN2_cmd_gimbal(gimbal_control .horizontal_scroll_motor[0].give_current ,gimbal_control .horizontal_scroll_motor[1].give_current,
		0,gimbal_control.horizontal_scroll_motor[3].give_current); //gimbal_control .horizontal_scroll_motor .give_current
		CAN2_cmd_gimbal_tai(gimbal_control .horizontal_scroll_motor[4].give_current ,gimbal_control .horizontal_scroll_motor[5].give_current,gimbal_control.gimbal_6020_motor.give_current,gimbal_control .horizontal_scroll_motor[6].give_current);
    }
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
	
	for (i=0;i<7;++i)
	{
		if(i!=2) //2号为6020电机
		{
			//初始化云台水平移动电机数据指针
			gimbal_init->horizontal_scroll_motor[i] .gimbal_motor_measure  = get_gimbal_motor_measure_point(i);
			//初始化云台水平电机最大最低速度
			gimbal_init->horizontal_scroll_motor[i] .speed_max =NORMAL_MAX_GIMBAL_SPEED;
			gimbal_init->horizontal_scroll_motor[i] .speed_min =-NORMAL_MAX_GIMBAL_SPEED;
		}
		else
		{
			gimbal_init->gimbal_6020_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(2);
			gimbal_init->gimbal_6020_motor.speed_max =NORMAL_MAX_GIMBAL_SPEED;
			gimbal_init->gimbal_6020_motor.speed_min =-NORMAL_MAX_GIMBAL_SPEED;
			
		}
			
		
	}
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
//			gimbal_shu(&gimbal_control);
//			gimbal_xuan(&gimbal_control); 
//			gimbal_fan(&gimbal_control);
			gimbal_jiaodubihuan_mechanical_arm(&gimbal_control);
			 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
			Pump_Flag = 0;
    }
		//左中右上操作抬升与舵机
  else if (SWITCH_LEFT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{   //存矿写在抬升中
			gimbal_tai(&gimbal_control);
		  gimbal_duoji(&gimbal_control);
			Pump_Flag = 0;
	}
	//左下右上控制舵机
	else if (SWITCH_LEFT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{
		 fsm_run();
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
		Pump_Flag = 0;
	}
	//左上右中控制气泵
	else if(SWITCH_LEFT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{
		if(Pump_Flag == 0)
		{
			if(Pump_Control_Flag == 0)
			{
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,Pump_Value[Pump_Control_Flag]);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,Pump_Value[Pump_Control_Flag]);
				Pump_Control_Flag = 1;
			}
			else if(Pump_Control_Flag == 1)
			{
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,Pump_Value[Pump_Control_Flag]);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,Pump_Value[Pump_Control_Flag]);
				Pump_Control_Flag = 0;
			}
		}
		Pump_Flag = 1;
	}
	//左中右中底盘运动
	else if (SWITCH_LEFT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
	{
		gimbal_keep(&gimbal_control);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
		Pump_Flag = 0;
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
  for (i=0;i<7;++i)
	{
		gimbal_move_update->horizontal_scroll_motor[i] .motor_speed_current  = GIMBAL_RPM_TO_SPEED_3508 * gimbal_move_update ->horizontal_scroll_motor[i] .gimbal_motor_measure->speed_rpm;
		//gimbal_move_update->horizontal_scroll_motor[i] .motor_speed_current  = MOTOR_RPM_TO_SPEED * gimbal_move_update ->horizontal_scroll_motor[i] .gimbal_motor_measure->speed_rpm;
	}
	gimbal_move_update->gimbal_6020_motor.motor_speed_current = GIMBAL_RPM_TO_SPEED_6020 * gimbal_move_update->gimbal_6020_motor.gimbal_motor_measure->speed_rpm;
	gimbal_mouse_keyboard_identify();
	relative_angle_to_mechanical_arm = gimbal_move_update->gimbal_6020_motor.gimbal_motor_measure->angle - 118;
	for(int i = 0;i < 4; i++)
	{
		if((int)fabs(relative_angle_to_mechanical_arm) == i*360)
		{
			relative_angle_to_mechanical_arm = 0;
		}
	}
}
/**
*@brief         控制循环，根据速度控制设定值，计算电机电流值，进行控制
*@param[in]			*gimbal_move_control_loop: 云台电机数据指针

*@param[in]			i:电机编号
*@param[in]			current_target_transfer：云台电机目标速度值
**/
static void gimbal_3508_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,float current_target_transfer)
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
	//	pid_test_1=gimbal_move_control_loop->horizontal_scroll_motor[6].motor_speed_current;

  //功率控制
  // chassis_power_control(chassis_move_control_loop);

  //赋值电流值
    gimbal_move_control_loop->horizontal_scroll_motor[i] .give_current  = (int16_t)(gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_speed_pid .out );   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
}

/**
*@brief         控制循环，根据速度控制设定值，计算电机电流值，进行控制
*@param[in]			*gimbal_move_control_loop: 云台电机数据指针
*@param[in]			i:电机编号
*@param[in]			current_target_transfer：云台电机目标速度值
**/
static void gimbal_6020_angle_control_loop(gimbal_control_t *gimbal_6020_angle_control_loop,int16_t i,float current_target_transfer)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
	float current_target=0,angle_out=0;
    temp = fabs(gimbal_6020_angle_control_loop->gimbal_6020_motor.speed_set);
	current_target=current_target_transfer;//遥控器模式使用

  // calculate pid
  //计算pid
	angle_out = PIID_calc(&gimbal_6020_angle_control_loop->gimbal_6020_motor.gimbal_motor_angle_pid,gimbal_6020_angle_control_loop->gimbal_6020_motor.gimbal_motor_measure->angle,current_target);
  PID_calc(&gimbal_6020_angle_control_loop->gimbal_6020_motor.gimbal_motor_speed_pid , gimbal_6020_angle_control_loop->gimbal_6020_motor.motor_speed_current, angle_out );
	//PID_calc(&gimbal_6020_angle_control_loop->gimbal_6020_motor.gimbal_motor_speed_pid , gimbal_6020_angle_control_loop->gimbal_6020_motor.motor_speed_current, current_target );
	pid_test_1=gimbal_6020_angle_control_loop->gimbal_6020_motor.gimbal_motor_measure->angle;
	//工程不需要功率限制
  //赋值电流值
    gimbal_6020_angle_control_loop->gimbal_6020_motor.give_current  = (int16_t)(gimbal_6020_angle_control_loop->gimbal_6020_motor.gimbal_motor_speed_pid .out );   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
}
/**
*@brief         控制循环，根据角度控制设定值，计算电机电流值，进行控制
*@param[in]			*gimbal_move_control_loop: 云台电机数据指针
*@param[in]			i:电机编号
*@param[in]			angle_target_transfer：云台电机目标角度值
**/
static void gimbal_3508_angle_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,int angle_target_transfer)
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
static void gimbal_jiaodubihuan_mechanical_arm(gimbal_control_t *gimbal_jiaodubihuan_mechanical_arm)
{
	target_can2_201_angle+=-gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_RIGHT_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-600);
	target_can2_202_angle+=-gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-500);
	target_can2_207_angle_6020+=gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [YAW_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-400);
	target_can2_204_angle+=gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-500);
	//target_can2_207_angle_6020 += (gimbal_jiaodubihuan_mechanical_arm->gimbal_6020_motor.gimbal_motor_measure->angle + gimbal_jiaodubihuan_mechanical_arm->gimbal_rc_ctrl ->rc .ch [YAW_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-500));
	pid_test_2 = target_can2_207_angle_6020;
	
	//电子限位 防止位置过于溢出
	electric_limit(target_can2_201_angle,target_can2_201_angle,TARGET_CAN2_201_MAX,TARGET_CAN2_201_MIN);
	electric_limit(target_can2_202_angle,target_can2_202_angle,TARGET_CAN2_202_MAX,TARGET_CAN2_202_MIN);
	electric_limit(target_can2_204_angle,target_can2_204_angle,TARGET_CAN2_204_MAX,TARGET_CAN2_204_MIN);
	electric_limit(target_can2_207_angle_6020,target_can2_207_angle_6020,TARGET_CAN2_207_6020_MAX,TARGET_CAN2_207_6020_MIN);
	
	gimbal_3508_angle_control_loop(&gimbal_control,0,target_can2_201_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,1,target_can2_202_angle);
	gimbal_6020_angle_control_loop(&gimbal_control,2,target_can2_207_angle_6020);
	gimbal_3508_angle_control_loop(&gimbal_control,3,target_can2_204_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,4,target_can2_205_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,5,target_can2_206_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,6,target_can2_208_angle);

}

//电机抬升角度闭环控制
static void gimbal_jiaoduhuan(gimbal_control_t *gimbal_jiaoduhuan)
{
	target_can2_205_angle+= gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]/(REMOTE_CONTROL_CHANNEL_LONG-600);
	target_can2_206_angle+=-gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]/(REMOTE_CONTROL_CHANNEL_LONG-600);//+gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]/660
	//存矿控制写在此处
	target_can2_208_angle+= gimbal_jiaoduhuan->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]/(REMOTE_CONTROL_CHANNEL_LONG-600);
	//pid_test_2 = target_can2_208_angle;
	
	//电子限位防止位置溢出
	electric_limit(target_can2_205_angle,target_can2_205_angle,TARGET_CAN2_205_MAX,TARGET_CAN2_205_MIN);
	electric_limit(target_can2_206_angle,target_can2_206_angle,TARGET_CAN2_206_MAX,TARGET_CAN2_206_MIN);
	electric_limit(target_can2_208_angle,target_can2_208_angle,TARGET_CAN2_208_MAX,TARGET_CAN2_208_MIN);
	
	gimbal_3508_angle_control_loop(&gimbal_control,0,target_can2_201_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,1,target_can2_202_angle);
	gimbal_6020_angle_control_loop(&gimbal_control,2,target_can2_207_angle_6020);
	gimbal_3508_angle_control_loop(&gimbal_control,3,target_can2_204_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,4,target_can2_205_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,5,target_can2_206_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,6,target_can2_208_angle);
}

//横向移动控制
static void gimbal_heng(gimbal_control_t *gimbal_heng)
{
	gimbal_heng_set=-gimbal_heng->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_RIGHT_CHANNEL]*GIMBAL_VEL;
	gimbal_3508_control_loop(&gimbal_control,0,gimbal_heng_set);
	gimbal_heng_set=0;
	
}

//竖向移动控制
static void gimbal_shu(gimbal_control_t *gimbal_shu)
{
	gimbal_shu_set=-gimbal_shu->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]*GIMBAL_VEL;
	gimbal_3508_control_loop(&gimbal_control,1,gimbal_shu_set);
	gimbal_shu_set=0;
}

//小臂旋转控制
static void gimbal_xuan(gimbal_control_t *gimbal_xuan)
{
	gimbal_xuan_set=-gimbal_xuan->gimbal_rc_ctrl ->rc .ch [YAW_CHANNEL]*GIMBAL_VEL*0.8;
	pid_test_2 = gimbal_xuan_set;
	gimbal_6020_angle_control_loop(&gimbal_control,2,gimbal_xuan_set);
	gimbal_xuan_set=0;
}

//小臂翻转控制
static void gimbal_fan(gimbal_control_t *gimbal_fan)
{
	gimbal_fan_set=gimbal_fan->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]*GIMBAL_VEL*0.8;
	gimbal_3508_control_loop(&gimbal_control,3,gimbal_fan_set);
	gimbal_fan_set=0;
}

//云台抬升控制
static void gimbal_tai(gimbal_control_t *gimbal_tai)
{
//  //使用双环时注释
//	if (gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]>100||gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]<-100)
//	gimbal_tai_set=gimbal_tai->gimbal_rc_ctrl ->rc .ch [GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE]*GIMBAL_VEL*0.8*(-1);
//	else
//	gimbal_tai_set=gimbal_tai_keep;	
//	gimbal_3508_control_loop(&gimbal_control,4,gimbal_tai_set);
//	gimbal_3508_control_loop(&gimbal_control,5,-gimbal_tai_set);
		gimbal_jiaoduhuan(&gimbal_control);
}

//舵机控制
static void gimbal_duoji(gimbal_control_t *gimbal_duoji)
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
	   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duoji);
}
//底盘运动时的云台保持
static void gimbal_keep(gimbal_control_t *gimbal_keep)
{
	gimbal_3508_angle_control_loop(&gimbal_control,0,target_can2_201_angle);
  gimbal_3508_angle_control_loop(&gimbal_control,1,target_can2_202_angle);
	gimbal_6020_angle_control_loop(&gimbal_control,2,target_can2_207_angle_6020);
	gimbal_3508_angle_control_loop(&gimbal_control,3,target_can2_204_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,4,target_can2_205_angle);
	gimbal_3508_angle_control_loop(&gimbal_control,5,target_can2_206_angle);
}
static void gimbal_pid_init(gimbal_control_t *gimbal_pid_init)
{
	//云台电机速度环PID初始化
  PID_init(&gimbal_pid_init->horizontal_scroll_motor[0] .gimbal_motor_speed_pid ,3000.0		, 0			,130.0	, 8000.0	, 3000.0);
  PID_init(&gimbal_pid_init->horizontal_scroll_motor[1] .gimbal_motor_speed_pid ,3000.0		, 200.0	,135.0	, 8000.0	, 3000.0);
	PID_init(&gimbal_pid_init->gimbal_6020_motor.gimbal_motor_speed_pid 					,15000.0	, 0			,0.0		, 20000.0	, 3000.0);
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[3] .gimbal_motor_speed_pid ,3200.0		, 0			,14.0		, 8000.0	, 3000.0);
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[4] .gimbal_motor_speed_pid ,2700.0		, 0			,11.3		, 8000.0	, 3000.0);
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[5] .gimbal_motor_speed_pid ,2700.0		, 0			,11.3		, 8000.0	, 3000.0);
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[6] .gimbal_motor_speed_pid ,3000.0		, 100.0	,50.0		, 3000.0	, 1000.0);
	//云台电机角度环PID初始化
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[0] .gimbal_motor_angle_pid ,0.08		, 0			,0.012	, 6.0		, 0.2 );
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[1] .gimbal_motor_angle_pid ,0.1 		, 0			,0.012	, 6.0		, 0.2 );
	PID_init(&gimbal_pid_init->gimbal_6020_motor.gimbal_motor_angle_pid 					,0.15 	, 0.0		, 0.14	, 10.0	, 0.3	);
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[3] .gimbal_motor_angle_pid ,0.1 		, 0			,0.01		, 4.0		, 0.2 );
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[4] .gimbal_motor_angle_pid ,0.035	, 0			,0.0035	, 6.0		, 0.2 );
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[5] .gimbal_motor_angle_pid ,0.035	, 0			,0.0035	, 6.0		, 0.2 );
	PID_init(&gimbal_pid_init->horizontal_scroll_motor[6] .gimbal_motor_angle_pid ,0.2		, 0			,0.0		, 8.0		, 0.0 );
	
	target_can2_207_angle_6020 = gimbal_pid_init->gimbal_6020_motor.gimbal_motor_measure->angle;
}
void gimbal_mouse_keyboard_identify(void)
{
	
	mouse_keyboard_state_flag.key_C.former_state = mouse_keyboard_state_flag.key_C.current_state;
	mouse_keyboard_state_flag.key_B.former_state = mouse_keyboard_state_flag.key_B.current_state;
	mouse_keyboard_state_flag.former_mouse_state	= mouse_keyboard_state_flag.current_mouse_state;
	
	mouse_keyboard_state_flag.key_C.current_state 	= ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13);	//位移为了使数据变成1变为标识符
	mouse_keyboard_state_flag.key_B.current_state  = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);
	//读取鼠标状态 用于判断当且仅当按下鼠标时 改变舵机状态
	mouse_keyboard_state_flag.current_mouse_state	=	(rc_ctrl.mouse.press_l	<< 1);
	mouse_keyboard_state_flag.current_mouse_state = (rc_ctrl.mouse.press_r | mouse_keyboard_state_flag.current_mouse_state);
	
			//切换键盘控制方式标识符更新 B用来控制云台控制模式
	if(mouse_keyboard_state_flag.key_B.current_state == 1 && mouse_keyboard_state_flag.key_B.former_state == 0) //当按下时
	{
		coordinates_control_flag++;
		if(coordinates_control_flag > 1)
		{
			coordinates_control_flag = 0;
		}
	}
			
			//切换键盘控制方式标识符更新 C用来控制底盘控制模式
	if(mouse_keyboard_state_flag.key_C.current_state == 1 && mouse_keyboard_state_flag.key_C.former_state == 0)
	{
		low_speed++;
		if(low_speed > 2)
		{
			low_speed = 0;
		}
	}
}