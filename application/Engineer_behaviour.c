//名称:2023工程连招总函数
//Author:Allen.Chopin
//描述:此文件包含工程车所有电机、气缸的控制函数，并在gimbal_task.c线程中引用。

 /**************************************/
 /******      ID使用说明：        ******/
 /******   0: 横轴平移电机        ******/
 /******   1：竖轴平移电机        ******/
 /******   2：机械臂水平旋转电机  ******/   //已修改为6020且发送ID为3，标识符为0x1ff,接收ID为0x207
 /******   3：机械臂竖直俯仰电机  ******/
 /******   4：侧方抬升电机        ******/
 /******   5：侧方抬升电机        ******/
 /**************************************/
 
 
 /******     2:向上角度增加     ********/
 /******    3:逆时针角度增加    ********/
 /******       4:电机正装       *******/
 /******       5:电机反装       *******/
 
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "Engineer_behaviour.h"
#include "tim.h"
#include "INS_task.h"
#include "referee_usart_task.h"
#include "gimbal_task.h"
#include "fsm.h"
//*****************************************************************结构体取回区************************************************//
extern gimbal_control_t gimbal_control;
extern RC_ctrl_t rc_ctrl;
//*****************************************************************函数声明区**************************************************//
void PID_All_Cal(void);
void act_Formatting(void);
//大资源岛空接动作函数声明如下:
static void act_AG_Formatting(void);  //强制初始化
static void act_AG_BigResourceIsland_Init(void);
static void act_AG_BigResourceIsland_Rising(void);
static void act_AG_BigResourceIsland_Center(void);
static void act_AG_BigResourceIsland_ProTract(void);
static void act_AG_BigResourceIsland_OverTurn(void);
static void act_AG_BigResourceIsland_Cylinder_On(void);
//大资源岛槽内取矿函数声明如下:
static void act_BTM_Formatting(void);  //强制初始化
static void act_BTM_BigResourceIsland_Rising(void);
static void act_BTM_BigResourceIsland_ArmReturn(void);
static void act_BTM_BigResourceIsland_Retract(void);
static void act_BTM_BigResourceIsland_OverTurn(void);
static void act_BTM_BigResourceIsland_Servo_Center(void);
static void act_BTM_BigResourceIsland_Storage_Down(void);
//键盘控制云台函数声明如下:
void engineer_gimbal_behaviour_keyboard_control(void);
//************************************************************PID计算参数定义/声明区*******************************************//
float target_can2_201_angle = 0;
float target_can2_202_angle = 0;
float target_can2_204_angle = 0;
float target_can2_205_angle;
float target_can2_206_angle;
float target_can2_207_angle_6020;
float target_can2_208_angle;
int servo_angle[8] = {0};

float pid_test_can2_201;
float pid_test_can2_202;
float pid_test_can2_207_angle_6020;
float pid_test_can2_204;
float pid_test_can2_205;
float pid_test_can2_206;
float pid_test_can2_207;
float pid_test_can2_208;

float give_angle_test = 0;
float give_angle_error_test = 40;
//************************************************************其它.C文件变量取回区及便捷变量区*********************************************//
extern float angle_can2_201;
extern float angle_can2_202;
extern float angle_can2_204;
extern float angle_can2_205;
extern float angle_can2_206;
extern float angle_can2_207_6020;
extern float angle_can2_208;


//********************************************************标志位定义区********************************************************//
uint8_t 	pump_flag_left 		= 0;
uint8_t 	pump_flag_right		= 0;
uint8_t 	laser_flag_left 	= 0;
uint8_t 	laser_flag_right	= 0;
int pump5_flag=1;
int pump6_flag=1;
int pump7_flag=1;
int pump8_flag=1;
int pump9_flag=1;
int pump10_flag=1;//下舵机标志位
int pump11_flag=1;
int pump12_flag=1;
int low_speed=0;//连招时置低速模式标志位
int put_mine=1;//此标志位为了解决前进方块按键冲突问题
int Speed_flag=1; //此标志位为了解决升快降慢的问题
int time_flag1=0;//此标志位为0.93s的延迟标志位
int time_flag2=0;//此标志位为0.4s的延迟标志位
int time_flag3=0;//此标志位为1.5s的延迟标志位
int gfo2_flag=1;//此标志位为仅定义一次延迟标志位
int aim_flag=0;
//***************************************************状态机状态、事件定义区*******************************************************//
typedef enum{
	//大资源岛空接事件
	AG_INIT,   //初始化事件
	AG_RISING,  //抬升事件
	AG_CENTER,   //中置事件
	AG_PROTRACT,  //爪子前伸事件
	AG_OVERTURN, //爪子翻转事件
	AG_CYLINDER_ON, //气缸打开事件
	AG_BIGISLAND_FINISH, //大资源岛空接事件完成
	
	//大资源岛槽内取矿事件
	BTM_START_RISING, //抬升事件
	BTM_ARM_RETURN,   //机械臂回正
	BTM_RETRACT,      //收回事件
	BTM_OVERTURN,     //翻转事件
	BTM_SERVO_INIT,   //舵机中置
	BTM_STORAGE_ORE_DOWN, //存矿下降
	BTM_FISISH,
	
	
	
	//强制初始化事件
	FORMATTING
}Event_ID;

typedef enum{
	//大资源岛空接状态
	AG_INIT_STATE,
	AG_RISING_STATE,
	AG_CENTER_STATE,   //中置状态
	AG_PROTRACR_STATE, //前伸状态
	AG_OVERTURN_STATE,  //前爪翻转状态
	AG_CYLINDER_ON_STATE, //气缸打开状态
	
	//大资源岛槽内取矿状态
	BTM_START_RISING_STATE, //抬升状态
	BTM_ARM_RETURN_STATE,   //回正状态
	BTM_RETRACT_STATE,      //收回状态
	BTM_SERVO_INIT_STATE,   //舵机中置
	BTM_CYLINDER_OFF_STATE, //关闭气缸
	BTM_STORAGE_ORE_DOWN_STATE, //存矿下降
	
}State;
//------------------------------------------------------连招状态机定义区--------------------------------------------------------------------------//
FSM Air_Get;//大资源岛空接事件定义

FsmTable ag_table[]={
	{AG_INIT_STATE				,AG_INIT							,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_RISING_STATE			,AG_RISING						,AG_CENTER_STATE						  ,act_AG_BigResourceIsland_Rising},
	{AG_CENTER_STATE			,AG_CENTER					  ,AG_PROTRACR_STATE						,act_AG_BigResourceIsland_Center},
	{AG_PROTRACR_STATE		,AG_PROTRACT					,AG_OVERTURN_STATE						,act_AG_BigResourceIsland_ProTract},
	{AG_OVERTURN_STATE		,AG_OVERTURN					,AG_CYLINDER_ON_STATE					,act_AG_BigResourceIsland_OverTurn},
	{AG_CYLINDER_ON_STATE	,AG_BIGISLAND_FINISH	,AG_INIT_STATE								,act_AG_BigResourceIsland_Cylinder_On},

  //暂时留有强制初始化位置
	{AG_INIT_STATE				,FORMATTING						,AG_RISING_STATE							,act_AG_Formatting},
	{AG_RISING_STATE			,FORMATTING					  ,AG_CENTER_STATE						  ,act_AG_Formatting},
	{AG_CENTER_STATE			,FORMATTING						,AG_PROTRACR_STATE						,act_AG_Formatting},
	{AG_PROTRACR_STATE		,FORMATTING						,AG_OVERTURN_STATE						,act_AG_Formatting},
	{AG_OVERTURN_STATE		,FORMATTING						,AG_CYLINDER_ON_STATE					,act_AG_Formatting},
	{AG_CYLINDER_ON_STATE	,FORMATTING						,AG_INIT_STATE								,act_AG_Formatting},
};
int Ag_Event; //大资源岛事件变量
//_______________________________________________________________________________
FSM Big_Take_Mine; //大资源岛槽内取矿事件

FsmTable btm_table[]=
{
	{BTM_START_RISING_STATE				,BTM_START_RISING				,BTM_ARM_RETURN_STATE				,act_BTM_BigResourceIsland_Rising},
	{BTM_ARM_RETURN_STATE					,BTM_ARM_RETURN					,BTM_RETRACT_STATE					,act_BTM_BigResourceIsland_ArmReturn},
	{BTM_RETRACT_STATE						,BTM_RETRACT						,BTM_SERVO_INIT_STATE				,act_BTM_BigResourceIsland_Retract},
	{BTM_SERVO_INIT_STATE					,BTM_SERVO_INIT					,BTM_STORAGE_ORE_DOWN_STATE	,act_BTM_BigResourceIsland_Servo_Center},
	{BTM_STORAGE_ORE_DOWN_STATE   ,BTM_STORAGE_ORE_DOWN		,BTM_START_RISING_STATE 		,act_BTM_BigResourceIsland_Storage_Down},
	
	{BTM_START_RISING_STATE				,FORMATTING							,BTM_ARM_RETURN_STATE				,act_BTM_Formatting},
	{BTM_ARM_RETURN_STATE					,FORMATTING							,BTM_RETRACT_STATE					,act_BTM_Formatting},
	{BTM_RETRACT_STATE						,FORMATTING							,BTM_SERVO_INIT_STATE				,act_BTM_Formatting},
	{BTM_SERVO_INIT_STATE					,FORMATTING							,BTM_STORAGE_ORE_DOWN_STATE	,act_BTM_Formatting},
	{BTM_STORAGE_ORE_DOWN_STATE   ,FORMATTING							,BTM_START_RISING_STATE 		,act_BTM_Formatting},
};
int Btm_Event;
//--------------------------------------------------------状态机初始化函数----------------------------------------------------------//
void fsm_init(void)
{
	//大资源岛空接初始化
	FSM_Regist(&Air_Get, ag_table);
	Air_Get.size = 12;
	FSM_StateTransfer(&Air_Get,AG_INIT_STATE);
	Ag_Event = AG_INIT;
	
	//大资源岛槽内取矿初始化
//	FSM_Regist(&Big_Take_Mine, btm_table);
//	Air_Get.size = 10;
//	FSM_StateTransfer(&Big_Take_Mine,AG_INIT_STATE);
//	Btm_Event = BTM_START_RISING_STATE;
}
//--------------------------------------------------------状态机运行函数-------------------------------------------------//
void fsm_run(void)
{
	//传感器标志更新
	pump_flag_left 		= 	gimbal_control.ore_flag.air_pump_flag_left;
	pump_flag_right 	= 	gimbal_control.ore_flag.air_pump_flag_right;
	laser_flag_left		=		gimbal_control.ore_flag.laser_flag_left;
	laser_flag_right  =   gimbal_control.ore_flag.laser_flag_right;
	FSM_EventHandle(&Air_Get,Ag_Event); //大资源岛空接状态机处理函数
	
	
	engineer_gimbal_behaviour_keyboard_control();//键盘控制处理函数
	PID_All_Cal();
}
void PID_All_Cal(void)
{
	//此处为PID双环
	gimbal_control.horizontal_scroll_motor[0].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[0].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[0].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[0].gimbal_motor_angle_pid		,angle_can2_201					,target_can2_201_angle));
	gimbal_control.horizontal_scroll_motor[1].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[1].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[1].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[1].gimbal_motor_angle_pid		,angle_can2_202					,target_can2_202_angle));
	gimbal_control.horizontal_scroll_motor[3].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[3].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[3].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[3].gimbal_motor_angle_pid		,angle_can2_204					,target_can2_204_angle));
	gimbal_control.gimbal_6020_motor.give_current 				 = PID_calc(&gimbal_control.gimbal_6020_motor.gimbal_motor_speed_pid						,gimbal_control.gimbal_6020_motor.motor_speed_current							,PID_calc(&gimbal_control.gimbal_6020_motor.gimbal_motor_angle_pid						,angle_can2_207_6020		,target_can2_207_angle_6020));
	gimbal_control.horizontal_scroll_motor[4].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[4].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[4].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[4].gimbal_motor_angle_pid		,angle_can2_205					,target_can2_205_angle));
	gimbal_control.horizontal_scroll_motor[5].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[5].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[5].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[5].gimbal_motor_angle_pid		,angle_can2_206					,target_can2_206_angle));

	pid_test_can2_201 = gimbal_control.horizontal_scroll_motor[0].give_current;
	pid_test_can2_202 = gimbal_control.horizontal_scroll_motor[1].give_current;
	pid_test_can2_207_angle_6020 = gimbal_control.gimbal_6020_motor.give_current;
	pid_test_can2_204 = gimbal_control.horizontal_scroll_motor[3].give_current;
	pid_test_can2_205 = gimbal_control.horizontal_scroll_motor[4].give_current;
	pid_test_can2_206 = gimbal_control.horizontal_scroll_motor[5].give_current;
}

void engineer_gimbal_behaviour_keyboard_control(void)
{
	//云台机械臂水平电机键盘控制
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q) 		 								//键盘按下Q时机械臂水平电机向左移动
	{
		target_can2_207_angle_6020 += KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_207_angle_6020,target_can2_207_angle_6020,TARGET_CAN2_207_6020_MAX,TARGET_CAN2_207_6020_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) 								//键盘按下E时机械臂水平向右移动
	{
		target_can2_207_angle_6020 -= KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_207_angle_6020,target_can2_207_angle_6020,TARGET_CAN2_207_6020_MAX,TARGET_CAN2_207_6020_MIN);
	}
	
	//云台机械臂竖直键盘控制
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R) 		 								//鼠标按下R时前伸电机向前移动
	{
		target_can2_204_angle += KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_204_angle,target_can2_204_angle,TARGET_CAN2_204_MAX,TARGET_CAN2_204_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F) 		 						//键盘按下R时前伸电机向后移动
	{
		target_can2_204_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_204_angle,target_can2_204_angle,TARGET_CAN2_204_MAX,TARGET_CAN2_204_MIN);
	}
	
	//云台抬升键盘控制
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C) 		 								//鼠标按下C时抬升电机向上移动
	{
		target_can2_205_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_205_angle,target_can2_205_angle,TARGET_CAN2_205_MAX,TARGET_CAN2_205_MIN);
		electric_limit(target_can2_206_angle,target_can2_206_angle,TARGET_CAN2_206_MAX,TARGET_CAN2_205_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V) 		 						//鼠标按下V时抬升电机向下移动
	{
		target_can2_205_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_205_angle,target_can2_205_angle,TARGET_CAN2_205_MAX,TARGET_CAN2_205_MIN);
		electric_limit(target_can2_206_angle,target_can2_206_angle,TARGET_CAN2_206_MAX,TARGET_CAN2_206_MIN);
	}
	
	//云台横移电机键盘控制
	if(gimbal_control.gimbal_rc_ctrl->mouse.press_l & 0x01) 		 						 						//鼠标按下左键时横移电机向左移动
	{
		target_can2_201_angle += MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_201_angle,target_can2_201_angle,TARGET_CAN2_201_MAX,TARGET_CAN2_201_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->mouse.press_r & 0x01) 		 										//鼠标按下右键时横移电机向右移动
	{
		target_can2_201_angle -= MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_201_angle,target_can2_201_angle,TARGET_CAN2_201_MAX,TARGET_CAN2_201_MIN);
	}
	
	//云台竖直电机键盘控制
	if(gimbal_control.gimbal_rc_ctrl->mouse.y | 0x00)                                   //鼠标前后移动时前伸前后移动
	{
		target_can2_202_angle += gimbal_control.gimbal_rc_ctrl->mouse.y * MOUSE_CONTROL_ANGLE_CAN2_202_CHANGE;
		//增加电子限位功能防止设定值超出物理限位无法及时响应
		electric_limit(target_can2_202_angle,target_can2_202_angle,TARGET_CAN2_202_MAX,TARGET_CAN2_202_MIN);
	}
}
//暂空
void act_Formatting(void)
{
	
}
//大资源岛空接动作函数与变量定义
int ag_flag=1;
int start_flag=0;

//大资源岛空接连招按键shift+f即可运行
static void act_AG_Formatting(void)
{
	if(rc_ctrl.key.v == 272)//shift+r强制初始化
	{
		act_Formatting();
		Ag_Event = FORMATTING;
		Air_Get.transfer_flag = 1;
		ag_flag = 0;
	}
}
static void act_AG_BigResourceIsland_Init(void)
{
	ag_flag=1;
	if(1/*rc_ctrl.key.v==528*/)//按键shift+f即可运行 调试时暂时使用恒成立
	{	
		low_speed=1;//进入低速模式  [待修改]
		target_can2_202_angle = TARGET_AG_CAN2_202_ANGLE_INIT; //收回竖直伸出
		target_can2_204_angle = TARGET_AG_CAN2_204_ANGLE_INIT;  //机械臂爪收回
		start_flag=1;
	}
	if(start_flag) 
	{
		if(angle_can2_202>20) //此10待修改 用于判断是否达到初始化位置
				return;
		else //当达到预设值时
		{
			target_can2_205_angle = TARGET_AG_CAN2_205_ANGLE_INIT; //抬升下降
			target_can2_206_angle = -TARGET_AG_CAN2_206_ANGLE_INIT; //抬升下降
			if(angle_can2_205 < 10) //待修改 用于判断是否下降充分
			{
				target_can2_207_angle_6020 = TARGET_AG_CAN2_207_ANGLE_6020_INIT;
				servo_angle[0] = 1500; //舵机中心值
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0); //继电器高电平，使能
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0); //同上
			}
			Ag_Event = AG_RISING;
			Air_Get.transfer_flag = 1;
		}
	}
	
}
static void act_AG_BigResourceIsland_Rising(void)
{
	if(ag_flag)//ag_flag为没有进行初始化的标志，此处暂写但不使用
	{
		start_flag = 0;
		target_can2_205_angle = TARGET_AG_CAN2_205_ANGLE_RISING; //抬升上升
		target_can2_206_angle = -TARGET_AG_CAN2_206_ANGLE_RISING;
	}
	if(angle_can2_205 < TARGET_AG_CAN2_205_ANGLE_RISING-give_angle_error_test) //待修改
	{
		return;
	}
	else
	{
		Ag_Event = AG_CENTER; //下一个事件为中置
		Air_Get.transfer_flag = 1;
	}
}
static void act_AG_BigResourceIsland_Center(void)
{
	if(ag_flag)//ag_flag为没有进行初始化的标志，此处暂写但不使用
	{
		target_can2_207_angle_6020 = TARGET_AG_CAN2_207_ANGLE_6020_CENTER; //机械臂中置
	}
	if(angle_can2_207_6020 < TARGET_AG_CAN2_207_ANGLE_6020_CENTER - 20) //待修改
	{
		return;
	}
	else
	{
		Ag_Event = AG_PROTRACT; //下一个事件为前伸
		Air_Get.transfer_flag = 1;
	}
}
static void act_AG_BigResourceIsland_ProTract(void)
{
	if(ag_flag)
	{
		target_can2_202_angle = -TARGET_AG_CAN2_202_ANGLE_PROTRACTING;  //前伸
		if(angle_can2_202 > (50-TARGET_AG_CAN2_202_ANGLE_PROTRACTING )) //待修改
		{
			return;
		}
		else
		{
			Ag_Event = AG_OVERTURN;
			Air_Get.transfer_flag = 1;
		}
	}
	
}
static void act_AG_BigResourceIsland_OverTurn(void)
{
	if(ag_flag)
	{
		target_can2_204_angle = TARGET_AG_CAN2_204_ANGLE_OVERTURN;  //翻转
		if(angle_can2_204 < TARGET_AG_CAN2_204_ANGLE_OVERTURN - 20) //待修改
		{
			return;
		}
		else
		{
			Ag_Event = AG_BIGISLAND_FINISH;
			Air_Get.transfer_flag = 1;
		}
	}
}
static void act_AG_BigResourceIsland_Cylinder_On(void)
{
	if(ag_flag)
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999); 
		Ag_Event = AG_INIT;
		Air_Get.transfer_flag = 1;
		//已结束故不执行
	}
}
//--------------------------------------------------------大资源岛槽内取矿连招函数-------------------------------------------------//
int BTM_Flag;
static void act_BTM_Formatting(void)
{
	if(rc_ctrl.key.v == 272)
	{
		act_Formatting();
		Btm_Event = FORMATTING;
		Big_Take_Mine.transfer_flag = 1;
		BTM_Flag = 0;
	}
}
//由于取矿已处于一个状态，故不予初始化
static void act_BTM_BigResourceIsland_Rising(void)
{
	BTM_Flag = 1;
	if( rc_ctrl.key.v==2064)	//按键shift+z即可运行
	{
		target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_RISING; //抬升上升
		target_can2_206_angle = -TARGET_BTM_CAN2_206_ANGLE_RISING;
	}
	if(angle_can2_205 < TARGET_BTM_CAN2_205_ANGLE_RISING - 10) //需调整
	{
		return;
	}
	else
	{
		Btm_Event = BTM_ARM_RETURN_STATE;
		Big_Take_Mine.transfer_flag = 1;
	}
}
//此函数用于机械臂中心回到存矿
static void act_BTM_BigResourceIsland_ArmReturn(void)
{
	if(BTM_Flag)
	{
		target_can2_207_angle_6020 = TARGET_BTM_CAN2_207_ANGLE_6020_RETURN; //回正
	}
	if(fabs(angle_can2_207_6020) >  TARGET_BTM_CAN2_207_ANGLE_6020_RETURN - 1) //待修改
	{
		return;
	}
	else
	{
		Btm_Event = BTM_RETRACT_STATE;
		Big_Take_Mine.transfer_flag = 1;
	}
}
static void act_BTM_BigResourceIsland_Retract(void)
{
	if(BTM_Flag)
	{
		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_RETRACR;  //前伸收回
	}
	if(angle_can2_202 > TARGET_BTM_CAN2_202_ANGLE_RETRACR - 1) //待修改
	{
		return;
	}
	else
	{
		Btm_Event = BTM_SERVO_INIT;
		Big_Take_Mine.transfer_flag = 1;
	}
}
static void act_BTM_BigResourceIsland_Servo_Center(void)
{
	if(BTM_Flag)
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1500);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999);  //关闭气缸
		Btm_Event = BTM_STORAGE_ORE_DOWN;
		Big_Take_Mine.transfer_flag = 1;
	}
}
static void act_BTM_BigResourceIsland_Storage_Down(void)
{
	if(BTM_Flag)
	{
		target_can2_208_angle = TARGET_BTM_CAN2_208_ANGLE_DOWN;  //存矿下降
	}
	if(angle_can2_208 > TARGET_BTM_CAN2_208_ANGLE_DOWN - 1) //待修改
	{
		return;
	}
	else
	{
		Btm_Event = BTM_START_RISING;
		Big_Take_Mine.transfer_flag = 1;
	}
}