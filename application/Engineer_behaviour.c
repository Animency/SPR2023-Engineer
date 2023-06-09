//名称:2023工程控制总函数
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
#include "math.h"
#include "cmsis_armcc.h"  //此头文件包含软件复位操作函数
#include "custom_ui_draw.h"
#include "chassis_behaviour.h"
#define rm_deadband_judge(input,dealine)             \
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
static void act_STM_Formatting(void);  //强制初始化
static void act_STM_BigResourceIsland_Init(void);
static void act_STM_BigResourceIsland_Rising(void);
static void act_STM_BigResourceIsland_Protract(void);
static void act_STM_BigResourceIsland_ArmReturn(void);
static void act_STM_BigResourceIsland_Rising_Step_2(void);
static void act_STM_BigResourceIsland_Retract(void);
static void act_STM_BigResourceIsland_Falling(void);
static void act_STM_BigResourceIsland_OverTurn(void);
static void act_STM_BigResourceIsland_Servo_Center(void);
static void act_STM_BigResourceIsland_Storage_Down(void);
static void act_STM_BigResourceIsland_Storage_up(void);
static void act_STM_BigResourceIsland_Retract_Ore(void);
static void act_STM_BigResourceIsland_Protract_Ore(void);
static void act_STM_BigResourceIsland_Arm_Return_Ore(void);
//存矿连招函数声明如下:
static void act_AOS_rising(void);
static void act_AOS_arm_overturn(void);
static void act_AOS_arm_turnaround(void);
static void act_AOS_servo_init(void);
static void act_AOS_storage_ore_up(void);
static void act_AOS_retract(void);
static void act_AOS_cylinder_off(void);
static void act_AOS_arm_vertical_dowm(void);
static void act_AOS_arm_vertical_down_horizon(void);
static void act_AOS_retract_ore_in(void);
static void act_AOS_protract(void);
static void act_AOS_storage_ore_down(void);
static void act_AOS_arm_return(void);
//全部机械结构复位声明如下:
static void act_AMI_Formatting(void);
static void act_AMI_init(void);
static void act_AMI_row_middle(void);
static void act_AMI_arm_vertical_horizon(void);
static void act_AMI_arm_return(void);
static void act_AMI_protratc(void);
static void act_AMI_sidesway_middle(void);
static void act_AMI_down(void);
static void act_AMI_finish(void);
//键盘鼠标控制云台函数声明如下:
void engineer_gimbal_behaviour_keyboard_control(void);
void engineer_keyboard_control_ordinal_coordinates(void);
void engineer_keyboard_control_except_coordinates(void);
void engineer_keyboard_control_mechanical_servo_coordinates(void);
void engineer_keyboard_control_mechanical_arm_coordinates(void);
void engineer_mouse_control(void);

//软件复位函数声明如下
__STATIC_INLINE void __set_FAULTMASK(uint32_t faultMask);
void SoftReset(void);

//判断是否在死区范围内
int engineer_deadband_judge(float input,float deadline,float deadband);
//************************************************************PID计算参数定义/声明区*******************************************//
float target_can2_201_angle = 0;
float target_can2_202_angle = 0;
float target_can2_203_angle = 0;
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
//************************************************************变量定义区*******************************************//
float give_angle_test = 0;
float give_angle_error_test = 40;
float relative_angle_to_mechanical_arm = 0;
float relative_angle_to_servo = 0;
//************************************************************其它.C文件变量取回区及便捷变量区*********************************************//
extern float angle_can2_201;
extern float angle_can2_202;
extern float angle_can2_203;
extern float angle_can2_204;
extern float angle_can2_205;
extern float angle_can2_206;
extern float angle_can2_207_6020;
extern float angle_can2_208;

extern int duoji;
extern uint16_t receive_id;
extern uint16_t send_id;
//********************************************************标志位定义区********************************************************//
uint8_t 	pump_flag_left 		= 0;
uint8_t 	pump_flag_right		= 0;
uint8_t 	laser_flag_left 	= 0;
uint8_t 	laser_flag_right	= 0;

uint8_t 	coordinates_control_flag = 0;
int pump10_flag=1;//下舵机标志位
int low_speed=0;//连招时置低速模式标志位
int put_mine=1;//此标志位为了解决前进方块按键冲突问题
int Speed_flag=1; //此标志位为了解决升快降慢的问题
int time_flag1=0;//此标志位为0.93s的延迟标志位
int time_flag2=0;//此标志位为0.4s的延迟标志位
int time_flag3=0;//此标志位为1.5s的延迟标志位
int gfo2_flag=1;//此标志位为仅定义一次延迟标志位
int aim_flag=0;
uint8_t STM_6020_flag = 1;
//定义读取鼠标键盘状态
mouse_keyboard_state_flag_t mouse_keyboard_state_flag;
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
	
	//小资源岛槽内取矿事件
	STM_INIT,
	STM_START_RISING, //抬升事件
	STM_ARM_PROTRACT, //前伸事件
	STM_ARM_RETURN,   //机械臂回正
	STM_RISING_STEP_2, //再次抬升
	STM_RETRACT,      //收回事件
	STM_FALLING,      //下降事件
	STM_SERVO_INIT,   //舵机中置
	STM_STORAGE_ORE_DOWN, //存矿下降
	STM_STORAGE_ORE_UP,   //存矿上升
	STM_RETRACT_ORE,      //存矿处前伸回收
	STM_PROTRACT_OVERTURN_ORE,     //存矿处前伸前伸与翻转
	STM_ARM_RETURN_ORE,						 //机械臂回正
	
	//自动存取矿石事件 此状态机作用为解决比赛规则限制抬升高度问题导致兑矿困难
	AOS_INIT,          		         //抬升下降初始化
	AOS_HORIZON_CENTER,            //横移回正
	AOS_ARM_OVERTURN,   	 	       //机械臂翻转
	AOS_ARM_TURNAROUND,  		       //机械臂回到中心
	AOS_SERVO_INIT,                //舵机中置
	AOS_ARM_OVERTURN_HORIZON,      //机械臂翻转到水平
	AOS_RETRACT_FIRST,        	 	 //前伸第一次收回
	AOS_CYLINDER_OFF,       		   //气泵关闭
	AOS_PROTRACT_FIRST,            //第一次微伸 
	AOS_STORAGE_ORE_UP,  		       //存矿上升
	AOS_CYLINDER_ON,               //气泵打开
	AOS_RETRACT_SECOND,            //前伸第二次收回
	AOS_PROTRACT_SECOND,					 //前伸第二次伸出
	AOS_STORAGE_ORE_DOWN,          //存矿下降
	AOS_ARM_RETURN,                //机械臂回正
	AOS_RETRACT_THIRD,						 //前伸第三次收回
	AOS_FINISH,										 //自动存矿结束
	
	//自动机械臂初始化事件
	AMI_INIT,
	AMI_ROW_MIDDLE,           //2006中置
	AMI_ARM_VERTICAL_HORIZON, //机械臂翻转水平
	AMI_ARM_RETURN,           //机械臂回正
	AMI_PROTRACT,             //前伸回收
	AMI_SIDESWAY_MIDDLE,      //横移中置
	AMI_DOWN,                 //抬升下降

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
	
	//小资源岛槽内取矿状态
	STM_INIT_STATE,
	STM_START_RISING_STATE, //抬升状态
	STM_ARM_PROTRACT_STATE, //前伸状态
	STM_ARM_RETURN_STATE,   //回正状态
	STM_RISING_STEP_2_STATE, //再次上升
	STM_RETRACT_STATE,      //收回状态
	STM_FALLING_STATE,      //下降状态
	STM_SERVO_INIT_STATE,   //舵机中置
	STM_CYLINDER_OFF_STATE, //关闭气缸
	STM_STORAGE_ORE_DOWN_STATE, //存矿下降
	STM_STORAGE_ORE_UP_STATE,   //存矿上升
	STM_RETRACT_ORE_STATE,      //存矿处前伸回收
	STM_PROTRACT_OVERTURN_ORE_STATE,     //存矿处前伸前伸与翻转
	STM_ARM_RETURN_ORE_STATE,						 //机械臂回正
	
	
	
	//自动存取矿石事件 此状态机作用为解决比赛规则限制抬升高度问题导致兑矿困难
	AOS_INIT_STATE,          		         //抬升下降初始化
	AOS_HORIZON_CENTER_STATE,            //横移回正
	AOS_ARM_OVERTURN_STATE,   	 	       //机械臂翻转
	AOS_ARM_TURNAROUND_STATE,  		       //机械臂回到中心
	AOS_SERVO_INIT_STATE,                //舵机中置
	AOS_ARM_OVERTURN_HORIZON_STATE,      //机械臂翻转到水平
	AOS_RETRACT_FIRST_STATE,        	 	 //前伸第一次收回
	AOS_CYLINDER_OFF_STATE,       		   //气泵关闭
	AOS_PROTRACT_FIRST_STATE,            //第一次微伸 
	AOS_STORAGE_ORE_UP_STATE,  		       //存矿上升
	AOS_CYLINDER_ON_SATET,               //气泵打开
	AOS_RETRACT_SECOND_STATE,            //前伸第二次收回
	AOS_PROTRACT_SECOND_STATE,					 //前伸第二次伸出
	AOS_STORAGE_ORE_DOWN_STATE,          //存矿下降
	AOS_ARM_RETURN_STATE,                //机械臂回正
	AOS_RETRACT_THIRD_STATE,						 //前伸第三次收回
	AOS_FINISH_STATE,										 //自动存矿结束
	
		//自动机械臂初始化事件
	AMI_INIT_STATE,
	AMI_ROW_MIDDLE_STATE,           //2006中置状态
	AMI_ARM_VERTICAL_HORIZON_STATE, //机械臂翻转水平状态
	AMI_ARM_RETURN_STATE,           //机械臂回正状态
	AMI_PROTRACT_STATE,             //前伸回收状态
	AMI_SIDESWAY_MIDDLE_STATE,      //横移中置状态
	AMI_DOWN_STATE,                 //抬升下降状态
	
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
	{AG_INIT_STATE				,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_RISING_STATE			,FORMATTING					  ,AG_RISING_STATE						  ,act_AG_BigResourceIsland_Init},
	{AG_CENTER_STATE			,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_PROTRACR_STATE		,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_OVERTURN_STATE		,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_CYLINDER_ON_STATE	,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
};
int Ag_Event; //大资源岛事件变量
//_________________________________________________________大资源岛槽内取矿___________________________________________________________________________//
FSM Small_Take_Mine; //小资源岛槽内取矿事件 

FsmTable stm_table[]=
{
	{STM_INIT_STATE										,STM_INIT										,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_START_RISING_STATE						,STM_START_RISING						,STM_ARM_PROTRACT_STATE							,act_STM_BigResourceIsland_Rising},
	{STM_ARM_PROTRACT_STATE						,STM_ARM_PROTRACT						,STM_ARM_RETURN_STATE								,act_STM_BigResourceIsland_Protract},
	{STM_ARM_RETURN_STATE							,STM_ARM_RETURN							,STM_RISING_STEP_2_STATE						,act_STM_BigResourceIsland_ArmReturn},
	{STM_RISING_STEP_2_STATE					,STM_RISING_STEP_2     			,STM_RETRACT_STATE									,act_STM_BigResourceIsland_Rising_Step_2},
	{STM_RETRACT_STATE								,STM_RETRACT								,STM_FALLING_STATE									,act_STM_BigResourceIsland_Retract},
	{STM_FALLING_STATE								,STM_FALLING								,STM_SERVO_INIT_STATE								,act_STM_BigResourceIsland_Falling},
	{STM_SERVO_INIT_STATE							,STM_SERVO_INIT							,STM_STORAGE_ORE_DOWN_STATE					,act_STM_BigResourceIsland_Servo_Center},
	{STM_STORAGE_ORE_DOWN_STATE   		,STM_STORAGE_ORE_DOWN				,STM_STORAGE_ORE_UP_STATE 					,act_STM_BigResourceIsland_Storage_Down},
	{STM_STORAGE_ORE_UP_STATE					,STM_STORAGE_ORE_UP					,STM_RETRACT_ORE_STATE							,act_STM_BigResourceIsland_Storage_up},
	{STM_RETRACT_ORE_STATE						,STM_RETRACT_ORE						,STM_PROTRACT_OVERTURN_ORE_STATE		,act_STM_BigResourceIsland_Retract_Ore},
	{STM_PROTRACT_OVERTURN_ORE_STATE	,STM_PROTRACT_OVERTURN_ORE	,STM_ARM_RETURN_ORE_STATE						,act_STM_BigResourceIsland_Protract_Ore},
	{STM_ARM_RETURN_ORE_STATE					,STM_ARM_RETURN_ORE					,STM_INIT_STATE											,act_STM_BigResourceIsland_Arm_Return_Ore},
	
	{STM_INIT_STATE										,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_START_RISING_STATE						,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_ARM_PROTRACT_STATE						,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_ARM_RETURN_STATE							,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_RISING_STEP_2_STATE					,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_RETRACT_STATE								,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_FALLING_STATE								,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_SERVO_INIT_STATE							,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_STORAGE_ORE_DOWN_STATE   		,FORMATTING							,STM_START_RISING_STATE 						,act_STM_BigResourceIsland_Init},
	{STM_STORAGE_ORE_UP_STATE					,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_RETRACT_ORE_STATE						,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_PROTRACT_OVERTURN_ORE_STATE	,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
	{STM_ARM_RETURN_ORE_STATE					,FORMATTING							,STM_START_RISING_STATE							,act_STM_BigResourceIsland_Init},
};
int Stm_Event;
//____________________________________________________________自动存矿___________________________________________________________________________//
FSM Auto_Ore_Storage;

//FsmTable aos_table[] = 
//{
//	{AOS_RISING_STATE							,AOS_RISING								,AOS_ARM_OVERTURN_STATE				,act_AOS_rising},
//	{AOS_ARM_OVERTURN_STATE				,AOS_ARM_OVERTURN					,AOS_ARM_TURNAROUND_STATE			,act_AOS_arm_overturn},
//	{AOS_ARM_TURNAROUND_STATE			,AOS_ARM_TURNAROUND				,AOS_SERVO_INIT_STATE					,act_AOS_arm_turnaround},
//	{AOS_SERVO_INIT_STATE					,AOS_SERVO_INIT						,AOS_STORAGE_ORE_UP_STATE			,act_AOS_servo_init},
//	{AOS_STORAGE_ORE_UP_STATE			,AOS_STORAGE_ORE_UP				,AOS_RETRACT_STATE						,act_AOS_storage_ore_up},
//};
int Aos_Event;

//____________________________________________________________所有机械臂软件归位____________________________________________________________________//
FSM Auto_Machine_Init;

FsmTable ami_table[] = 
{
	{AMI_INIT_STATE										,AMI_INIT									,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_ROW_MIDDLE_STATE							,AMI_ROW_MIDDLE						,AMI_ARM_VERTICAL_HORIZON_STATE			,act_AMI_row_middle},
	{AMI_ARM_VERTICAL_HORIZON_STATE		,AMI_ARM_VERTICAL_HORIZON	,AMI_ARM_RETURN_STATE								,act_AMI_arm_vertical_horizon},
	{AMI_ARM_RETURN_STATE							,AMI_ARM_RETURN						,AMI_PROTRACT_STATE									,act_AMI_arm_return},
	{AMI_PROTRACT_STATE								,AMI_PROTRACT							,AMI_SIDESWAY_MIDDLE_STATE					,act_AMI_protratc},
	{AMI_SIDESWAY_MIDDLE_STATE				,AMI_SIDESWAY_MIDDLE			,AMI_DOWN_STATE											,act_AMI_sidesway_middle},
	{AMI_DOWN_STATE										,AMI_DOWN									,AMI_INIT_STATE											,act_AMI_down},
	
	{AMI_INIT_STATE										,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_ROW_MIDDLE_STATE							,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_ARM_VERTICAL_HORIZON_STATE		,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_ARM_RETURN_STATE							,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_PROTRACT_STATE								,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_SIDESWAY_MIDDLE_STATE				,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
	{AMI_DOWN_STATE										,FORMATTING								,AMI_ROW_MIDDLE_STATE								,act_AMI_init},
};
int Ami_Event;
//--------------------------------------------------------状态机初始化函数----------------------------------------------------------//
void fsm_init(void)
{
	//大资源岛空接初始化
	FSM_Regist(&Air_Get, ag_table);
	Air_Get.size = 12;
	FSM_StateTransfer(&Air_Get,AG_INIT_STATE);
	Ag_Event = AG_INIT;
	
	//大资源岛槽内取矿初始化
	FSM_Regist(&Small_Take_Mine, stm_table);
	Small_Take_Mine.size = 26;
	FSM_StateTransfer(&Small_Take_Mine,STM_INIT_STATE);
	Stm_Event = STM_INIT;
	
	//自动置位初始化
	FSM_Regist(&Auto_Machine_Init, ami_table);
	Auto_Machine_Init.size = 14;
	FSM_StateTransfer(&Auto_Machine_Init,AMI_INIT_STATE);
	Ami_Event = AMI_INIT;
}
int engineer_deadband_judge(float input,float deadline,float deadband)
{
	if((input >= (deadline - deadband)) &&(input < (deadline + deadband)))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//--------------------------------------------------------状态机运行函数-------------------------------------------------//

/**
 * @brief          状态机运行函数，本文件的运行总函数，在gimbal_task.c里面调用
 *
 * @param[out]     none
 * @retval         none
 */
void fsm_run(void)
{
	//传感器标志更新  此处仅限于双板通信使用 处于非双板通信状态由键盘控制四个变量的更换
//	pump_flag_left 		= 	gimbal_control.ore_flag.air_pump_flag_left;
//	pump_flag_right 	= 	gimbal_control.ore_flag.air_pump_flag_right;
//	laser_flag_left		=		gimbal_control.ore_flag.laser_flag_left;
//	laser_flag_right  =   gimbal_control.ore_flag.laser_flag_right;
	
	//大资源岛空接状态机处理函数
	FSM_EventHandle(&Air_Get						,Ag_Event); 
	FSM_EventHandle(&Small_Take_Mine		,Stm_Event);
	FSM_EventHandle(&Auto_Machine_Init	,Ami_Event);
	
	//键盘控制处理函数
	engineer_gimbal_behaviour_keyboard_control();
	
	//增加电子限位功能防止设定值超出物理限位无法及时响应
	electric_limit(target_can2_201_angle			,target_can2_201_angle			,TARGET_CAN2_201_MAX			,TARGET_CAN2_201_MIN);
	electric_limit(target_can2_202_angle			,target_can2_202_angle			,TARGET_CAN2_202_MAX			,TARGET_CAN2_202_MIN);
	electric_limit(target_can2_203_angle			,target_can2_203_angle			,TARGET_CAN2_203_MAX			,TARGET_CAN2_203_MIN);
	electric_limit(target_can2_204_angle			,target_can2_204_angle			,TARGET_CAN2_204_MAX			,TARGET_CAN2_204_MIN);
	electric_limit(target_can2_205_angle			,target_can2_205_angle			,TARGET_CAN2_205_MAX			,TARGET_CAN2_205_MIN);
	electric_limit(target_can2_206_angle			,target_can2_206_angle			,TARGET_CAN2_206_MAX			,TARGET_CAN2_206_MIN);
	electric_limit(target_can2_207_angle_6020	,target_can2_207_angle_6020	,TARGET_CAN2_207_6020_MAX	,TARGET_CAN2_207_6020_MIN);
	electric_limit(target_can2_208_angle			,target_can2_208_angle			,TARGET_CAN2_208_MAX			,TARGET_CAN2_208_MIN);

	//当抬升到一定高度时切换为低速模式
	if(angle_can2_205 > CHASSIS_LOW_SPEED_ANGLE && low_speed == 0) 
	{
		low_speed = 2;
	}
	
	PID_All_Cal();
	
	SoftReset();
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
	gimbal_control.horizontal_scroll_motor[6].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[6].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[6].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[6].gimbal_motor_angle_pid		,angle_can2_208					,target_can2_208_angle));
	gimbal_control.horizontal_scroll_motor[7].give_current = PID_calc(&gimbal_control.horizontal_scroll_motor[7].gimbal_motor_speed_pid		,gimbal_control.horizontal_scroll_motor[7].motor_speed_current		,PID_calc(&gimbal_control.horizontal_scroll_motor[7].gimbal_motor_angle_pid		,angle_can2_203					,target_can2_203_angle));
	
	pid_test_can2_201 = gimbal_control.horizontal_scroll_motor[0].give_current;
	pid_test_can2_202 = gimbal_control.horizontal_scroll_motor[1].give_current;
	pid_test_can2_207_angle_6020 = gimbal_control.gimbal_6020_motor.give_current;
	pid_test_can2_204 = gimbal_control.horizontal_scroll_motor[3].give_current;
	pid_test_can2_205 = gimbal_control.horizontal_scroll_motor[4].give_current;
	pid_test_can2_206 = gimbal_control.horizontal_scroll_motor[5].give_current;
}

void engineer_gimbal_behaviour_keyboard_control(void)
{
	engineer_mouse_control();
	
	switch(coordinates_control_flag)
	{
		case 0: //第一人称控制
			engineer_keyboard_control_ordinal_coordinates();
			break;
		case 1: //以机械臂为坐标控制
			engineer_keyboard_control_mechanical_arm_coordinates();
			break;
		case 2: //以舵机所在位置为坐标控制
		  engineer_keyboard_control_mechanical_servo_coordinates();
		default:
			break;
	}
	engineer_keyboard_control_except_coordinates();
}
void engineer_keyboard_control_except_coordinates(void) //除坐标系转换外的电机控制
{
	//云台机械臂水平电机键盘控制
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_Q) 		 							//键盘按下[SHIFT+Q]时机械臂水平电机向左移动
	{
		target_can2_207_angle_6020 += KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_E) 							//键盘按下[SHIFT+E]时机械臂水平向右移动
	{
		target_can2_207_angle_6020 -= KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
	}
	
	//云台机械臂竖直电机键盘控制
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_Z) 		 								//键盘按下[SHIFT+Z]时机械臂竖直电机向下移动
	{
		target_can2_204_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_X) 							//键盘按下[SHIFT+X]时机械臂竖直电机向上移动
	{
		target_can2_204_angle += KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
	}
	
	//云台抬升键盘控制
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_R) 		 								//键盘按下[SHIFT+R]时抬升电机向上移动
	{
		target_can2_205_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_F) 		 						//键盘按下[SHIFT+F]时抬升电机向下移动
	{
		target_can2_205_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
	}
	
	//键盘控制行走已写在chassis_task中
	
	if(!((gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) || (gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)))
	{
		//云台舵机气泵控制
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) 		 								//键盘按下G时开泵
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,19999);
			pump_flag_left = 1;
			pump_flag_right = 1;
			draw_card_position(send_id, receive_id, 2);
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)               //键盘按下V时关泵
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
			pump_flag_left = 0;
			pump_flag_right = 0;
			draw_card_position(send_id, receive_id, 2);
		}
		
		//存矿电机控制
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)
		{
			target_can2_208_angle += KEYBOARD_CONTROL_ANGLE_CAN2_208_CHANGE;
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)
		{
			target_can2_208_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_208_CHANGE;
		}
	}
	
}
void engineer_keyboard_control_mechanical_servo_coordinates(void)
{
	//W+S 前后
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_W)               //键盘按下[SHIFT+W]时竖轴电机向前移动
	{
		target_can2_202_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm 	* ANGLE_TO_ECD) * cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_201_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	* ANGLE_TO_ECD)	* cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_205_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
		target_can2_206_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_S)					//键盘按下[SHIFT+S]时竖轴电机向后移动
	{
		target_can2_202_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm 	* ANGLE_TO_ECD) * cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_201_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	* ANGLE_TO_ECD)	* cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_205_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
		target_can2_206_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
	}
	//A+D 左右
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_A)               //键盘按下[SHIFT+A]时横轴电机向左移动
	{
		target_can2_202_angle += (-KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle += ( KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_D)					//键盘按下[SHIFT+S]时横轴电机向右移动
	{
		target_can2_202_angle -= (-KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle -= ( KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
}
void engineer_keyboard_control_mechanical_arm_coordinates(void)
{
	//W+S 前后
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_W)               //键盘按下[SHIFT+W]时竖轴电机向前移动
	{
		target_can2_202_angle -= KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE	*	sin(	relative_angle_to_mechanical_arm 	* ANGLE_TO_ECD) ;
		target_can2_201_angle -= KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	* ANGLE_TO_ECD)	;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_S)					//键盘按下[SHIFT+S]时竖轴电机向后移动
	{
		target_can2_202_angle += KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE *	sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD) ;
		target_can2_201_angle += KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD) ;
	}
	//A+D 左右
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_A)               //键盘按下[SHIFT+A]时横轴电机向左移动
	{
		target_can2_202_angle += (-KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle += ( KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_D)					//键盘按下[SHIFT+S]时横轴电机向右移动
	{
		target_can2_202_angle -= (-KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle -= ( KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
}
void engineer_keyboard_control_ordinal_coordinates(void)   //以图传为坐标的控制 第一人称
{
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)
	{
			//竖轴电机键盘控制
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W)               //键盘按下[SHIFT+W]时竖轴电机向前移动
		{
			target_can2_202_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_202_CHANGE ;
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S)					//键盘按下[SHIFT+S]时竖轴电机向后移动
		{
			target_can2_202_angle += KEYBOARD_CONTROL_ANGLE_CAN2_202_CHANGE ;
		}
		//横轴电机键盘控制
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A)               //键盘按下[SHIFT+A]时横轴电机向左移动
		{
			target_can2_201_angle += KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE ;
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)					//键盘按下[SHIFT+D]时横轴电机向右移动
		{
			target_can2_201_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE ;
		}
	}
}
void engineer_mouse_control(void)
{
	//当鼠标左键按下时暂设定为控制舵机
	if(mouse_keyboard_state_flag.current_mouse_state & MOUSE_PRESSED_OFFSET_LEFT)
	{
		target_can2_203_angle -= 4;
//		//上一次状态为松开时
//		if(!(mouse_keyboard_state_flag.former_mouse_state & MOUSE_PRESSED_OFFSET_LEFT))
//		{
//			target_can2_203_angle += 500;
//		}
	}
	//当鼠标右键按下时
	if(mouse_keyboard_state_flag.current_mouse_state & MOUSE_PRESSED_OFFSET_RIGHT)
	{
		target_can2_203_angle += 4;
		//上一次状态为松开时
//		if(!(mouse_keyboard_state_flag.former_mouse_state & MOUSE_PRESSED_OFFSET_RIGHT))
//		{
//			target_can2_203_angle -= 500;
//		}
	}
}

//软件复位单片机
void SoftReset(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_Q) //Ctrl+Q开始软件置位
	{
		__set_FAULTMASK(1); //关闭所有中断
		NVIC_SystemReset(); //复位
	}
}


void act_Formatting(void)
{
	//yaw轴回正 此函数本就是一个让状态机重新加载的函数，故没有很复杂的操作，如果要做到全部归位的话，就应该重新写一个状态机用于回复
	target_can2_203_angle = 90;
	target_can2_207_angle_6020 = TARGET_CAN2_207_6020_ANGLE_FORMATTING;
}
//大资源岛空接动作函数与变量定义
int ag_flag=1;
int start_flag=0;

//大资源岛空接连招按键shift+f即可运行
static void act_AG_Formatting(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_R)//shift+r强制初始化
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
	if(rc_ctrl.key.v==KEY_PRESSED_CTRL_B)//按键ctrl+B即可运行 调试时暂时使用恒成立
	{	
		low_speed=2;//离开低速模式  [待修改] 本应进入
		target_can2_202_angle = TARGET_AG_CAN2_202_ANGLE_INIT; //收回竖直伸出
		target_can2_204_angle = TARGET_AG_CAN2_204_ANGLE_INIT;  //机械臂爪收回
		start_flag=1;
	}
	if(start_flag) 
	{
		if(angle_can2_202>20) // 用于判断是否达到初始化位置
				return;
		else //当达到预设值时
		{
			target_can2_205_angle = TARGET_AG_CAN2_205_ANGLE_INIT; //抬升下降
			target_can2_206_angle = -TARGET_AG_CAN2_206_ANGLE_INIT; //抬升下降
			if(angle_can2_205 < 10) // 用于判断是否下降充分
			{
				target_can2_207_angle_6020 = TARGET_AG_CAN2_207_ANGLE_6020_INIT;
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0); //继电器低电平，失能
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
	if(angle_can2_205 < TARGET_AG_CAN2_205_ANGLE_RISING-100) //待修改
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
	act_AG_Formatting();
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
	act_AG_Formatting();
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
	act_AG_Formatting();
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
	act_AG_Formatting();
	if(ag_flag)
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999); 
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,19999); 
		pump_flag_left = 1;
		pump_flag_right = 1;
		Ag_Event = AG_INIT;
		Air_Get.transfer_flag = 1;
		//已结束故不执行
	}
}
//--------------------------------------------------------大资源岛槽内取矿连招函数-------------------------------------------------//
int STM_Flag;
static void act_STM_Formatting(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_X) //Ctrl+X强制初始化
	{
		act_Formatting();
		Stm_Event = FORMATTING;
		Small_Take_Mine.transfer_flag = 1;
		STM_Flag = 0;
	}
}
static void act_STM_BigResourceIsland_Init(void)
{
	if( rc_ctrl.key.v == KEY_PRESSED_CTRL_F)  //按键Ctrl+f即可运行
	{
		low_speed=2;//进入低速模式 
		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_INIT; //收回竖直伸出
		target_can2_204_angle = TARGET_BTM_CAN2_204_ANGLE_INIT;  //机械臂爪收回
		STM_Flag = 1;
	}
	if(STM_Flag) 
	{
		if(angle_can2_202>20) //此10待修改 用于判断是否达到初始化位置
				return;
		else //当达到预设值时
		{
			Stm_Event = STM_START_RISING;
			Small_Take_Mine.transfer_flag = 1;
		}
	}
}

static void act_STM_BigResourceIsland_Rising(void)
{

	if(STM_Flag)	
	{
		target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_RISING; //抬升上升
		target_can2_206_angle = -TARGET_BTM_CAN2_206_ANGLE_RISING;
	}
	if(engineer_deadband_judge(angle_can2_205,TARGET_BTM_CAN2_205_ANGLE_RISING,70))
	{
		Stm_Event = STM_ARM_PROTRACT;
		Small_Take_Mine.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_STM_BigResourceIsland_Protract(void)
{
	act_STM_Formatting();
	if(STM_Flag)	
	{
		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_Protract; //前伸
	}
	if(engineer_deadband_judge(angle_can2_202,TARGET_BTM_CAN2_202_ANGLE_Protract,40))
	{
		Stm_Event = STM_ARM_RETURN;
		Small_Take_Mine.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
//此函数用于机械臂中心来到正前方
static void act_STM_BigResourceIsland_ArmReturn(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		target_can2_207_angle_6020 = TARGET_BTM_CAN2_207_ANGLE_6020_RETURN; //回正
	}
	if(fabs(angle_can2_207_6020) >  TARGET_BTM_CAN2_207_ANGLE_6020_RETURN + 3 || fabs(angle_can2_207_6020) < TARGET_BTM_CAN2_207_ANGLE_6020_RETURN - 3) //待修改
	{
		return;
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999);//打开气泵
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,19999);
		pump_flag_left = 1;
		pump_flag_right = 1;
		Stm_Event = STM_RISING_STEP_2;    
		Small_Take_Mine.transfer_flag = 1;
		STM_Flag = 0;
	}
}
static void act_STM_BigResourceIsland_Rising_Step_2(void)
{
	act_STM_Formatting(); //2080
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_Z)
	{
		STM_Flag = 1;
	}
	if(STM_Flag)
	{
		target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_RISING_STEP_2; //抬升上升 取出小资源岛
		target_can2_206_angle = -TARGET_BTM_CAN2_206_ANGLE_RISING_STEP_2;
	}

	if(engineer_deadband_judge(angle_can2_205,TARGET_BTM_CAN2_205_ANGLE_RISING_STEP_2,100) && STM_Flag)
	{
		Stm_Event = STM_RETRACT;
		Small_Take_Mine.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_STM_BigResourceIsland_Retract(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		target_can2_202_angle = -200.0;  //前伸收回
	}
	if(engineer_deadband_judge(angle_can2_202,-TARGET_BTM_CAN2_202_ANGLE_RETRACR,40))
	{
		target_can2_204_angle = TARGET_BTM_CAN2_204_ANGLE_OVERTURN; //在前伸收回的过程中机械臂翻转向上
		if(engineer_deadband_judge(angle_can2_202,-TARGET_BTM_CAN2_202_ANGLE_RETRACR,30) && engineer_deadband_judge(target_can2_204_angle,TARGET_BTM_CAN2_204_ANGLE_OVERTURN,20.0))
		{
			Stm_Event = STM_FALLING;
			Small_Take_Mine.transfer_flag = 1;
		}
		else
		{
			return ;
		}
	}
}
static void act_STM_BigResourceIsland_Falling(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		if(STM_6020_flag)
		{
			target_can2_207_angle_6020 -= 1.0f;
		}
		if(target_can2_207_angle_6020 == TARGET_STM_CAN2_207_6020_ANGLE_ORE) //在前伸收回的过程中6020回到存矿中
		{
			STM_6020_flag = 0;
			target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_FALLING; //抬升下降
			target_can2_206_angle = -TARGET_BTM_CAN2_205_ANGLE_FALLING;
			if(engineer_deadband_judge(angle_can2_205,TARGET_BTM_CAN2_205_ANGLE_FALLING,50) && engineer_deadband_judge(angle_can2_207_6020,TARGET_STM_CAN2_207_6020_ANGLE_ORE,10.0))
			{
				if(engineer_deadband_judge(angle_can2_205,TARGET_BTM_CAN2_205_ANGLE_FALLING,100))
				{
					Stm_Event = STM_SERVO_INIT;
					Small_Take_Mine.transfer_flag = 1;
					STM_6020_flag = 1;
				}
				else
				{
					return;
				}
			}
			else
			{
				return;
			}
		}
		else
		{
			return;
		}
	}
	
}
static void act_STM_BigResourceIsland_Servo_Center(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		target_can2_203_angle = 0; //舵机值待修改
		target_can2_204_angle = TARGET_BTM_CAN2_204_ANGLE_OVERTURN_SECOND;
		pump_flag_left = 0;
		pump_flag_right = 0;
		if(engineer_deadband_judge(angle_can2_204,TARGET_BTM_CAN2_204_ANGLE_OVERTURN_SECOND,20))
		{
			Stm_Event = STM_STORAGE_ORE_DOWN;
			Small_Take_Mine.transfer_flag = 1;
		}
		else
		{
			return ;
		}
	}
}
static void act_STM_BigResourceIsland_Storage_Down(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		target_can2_208_angle = TARGET_BTM_CAN2_208_ANGLE_DOWN;  //存矿下降
		if(engineer_deadband_judge(angle_can2_208,TARGET_BTM_CAN2_208_ANGLE_DOWN,80))
		{
			Stm_Event = STM_STORAGE_ORE_UP_STATE;
		  Small_Take_Mine.transfer_flag = 1;
			STM_Flag = 0;
		}
		else
		{
			return;
		}
	}
}
//在存矿下降后可手动触发取出
static void act_STM_BigResourceIsland_Storage_up(void)
{
	act_STM_Formatting();
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_R)
	{
		STM_Flag = 1; //可继续后面的函数
//		target_can2_208_angle = TARGET_BTM_CAN2_208_ANGLE_UP;
	}
	if(STM_Flag)
	{
//		if(engineer_deadband_judge(angle_can2_208,TARGET_BTM_CAN2_208_ANGLE_UP,40))
//		{
//			Stm_Event = STM_RETRACT_ORE;
//		  Small_Take_Mine.transfer_flag = 1;
//		}
//		else
//		{
//			return;
//		}
		Stm_Event = STM_RETRACT_ORE;
		Small_Take_Mine.transfer_flag = 1;
	}
}
static void act_STM_BigResourceIsland_Retract_Ore(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
//		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_RETRACT_ORE;
//		if(engineer_deadband_judge(target_can2_202_angle,TARGET_BTM_CAN2_202_ANGLE_RETRACT_ORE,10))
//		{
//			Stm_Event = STM_PROTRACT_OVERTURN_ORE;
//			Small_Take_Mine.transfer_flag = 1;
//		}
//		else
//		{
//			return;
//		}
		Stm_Event = STM_PROTRACT_OVERTURN_ORE;
		Small_Take_Mine.transfer_flag = 1;
	}
}
static void act_STM_BigResourceIsland_Protract_Ore(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		target_can2_202_angle = -TARGET_BTM_CAN2_202_ANGLE_PROTRACT_ORE;
		if(engineer_deadband_judge(target_can2_202_angle,-TARGET_BTM_CAN2_202_ANGLE_PROTRACT_ORE,50))
		{
			target_can2_204_angle = TARGET_BTM_CAN2_204_ANGLE_OVERTURN_ORE;
			if(engineer_deadband_judge(target_can2_202_angle,-TARGET_BTM_CAN2_202_ANGLE_PROTRACT_ORE,30) && engineer_deadband_judge(target_can2_204_angle,TARGET_BTM_CAN2_204_ANGLE_OVERTURN_ORE,30))
			{
					Stm_Event = STM_ARM_RETURN_ORE;
					Small_Take_Mine.transfer_flag = 1;
			}
			else
			{
				return;
			}
		}
		else
		{
			return;
		}
	}
}
static void act_STM_BigResourceIsland_Arm_Return_Ore(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		if(STM_6020_flag)
		{
			target_can2_207_angle_6020 += 1.0f;
		}
		if(engineer_deadband_judge(target_can2_207_angle_6020,TARGET_BTM_CAN2_207_6020_ANGLE_ARM_RETURN_ORE,4.0f))
		{
			STM_6020_flag = 0;
			if(engineer_deadband_judge(target_can2_207_angle_6020,TARGET_BTM_CAN2_207_6020_ANGLE_ARM_RETURN_ORE,20))
			{
				Stm_Event = STM_INIT;
				Small_Take_Mine.transfer_flag = 1;
				STM_Flag = 0;
				STM_6020_flag = 1;
			}
			else
			{
				return;
			}
		}
		
	}
}

//--------------------------------------------------------自动存矿连招函数-------------------------------------------------//
int AOS_flag;
static void act_AOS_formatting(void)
{
	if(rc_ctrl.key.v == 1000) //待修改
	{
		act_Formatting();
		Aos_Event = FORMATTING;
		Auto_Ore_Storage.transfer_flag = 1;
	}
}
static void act_AOS_init(void)
{
	AOS_flag = 1;
	if(rc_ctrl.key.v == 6666) //ctrl+I控制本连招
	{
		target_can2_205_angle = TARGET_AOS_CAN2_205_ANGLE_DOWN; //抬升下降
		target_can2_206_angle = -TARGET_AOS_CAN2_206_ANGLE_DOWN;
	}
	if(engineer_deadband_judge(angle_can2_205,TARGET_AOS_CAN2_205_ANGLE_DOWN,40.0)) //待修改
	{
		return;
	}
	else
	{
		Aos_Event = AOS_HORIZON_CENTER;
		Auto_Ore_Storage.transfer_flag = 1;
	}
}
static void act_AOS_horizon_center(void)
{
	act_AOS_formatting();
	if(AOS_flag)
	{
		target_can2_202_angle = TARGET_AOS_CAN2_202_HORIZON_CENTER; //横移水平回正
	}
	if(engineer_deadband_judge(angle_can2_202,TARGET_AOS_CAN2_202_HORIZON_CENTER,20.0)) //待修改
	{
		return;
	}
	else
	{
		Aos_Event = AOS_ARM_OVERTURN;
		Auto_Ore_Storage.transfer_flag = 1;
	}
}
//static void act_AOS_arm_overturn(void)
//{
//	act_AOS_formatting();
//	if(AOS_flag)
//	{
//		target_can2_204_angle = TARGET_AOS_CAN2_204_ARM_OVERTURN; //小臂翻转
//	}
//	if(engineer_deadband_judge(angle_can2_204,TARGET_AOS_CAN2_204_ARM_OVERTURN,40.0)) //待修改
//	{
//		return;
//	}
//	else
//	{
//		Aos_Event = AOS_ARM_TURNAROUND;
//		Auto_Ore_Storage.transfer_flag = 1;
//	}
//}
static void act_AOS_arm_turnaround(void)
{
	act_AOS_formatting();
	if(AOS_flag)
	{
		target_can2_207_angle_6020 = TARGET_AOS_CAN2_207_6020_ARM_TURNAROUND; //小臂翻转
	}
	if(engineer_deadband_judge(angle_can2_207_6020,TARGET_AOS_CAN2_207_6020_ARM_TURNAROUND,40.0)) //待修改
	{
		return;
	}
	else
	{
		Aos_Event = AOS_SERVO_INIT;
		Auto_Ore_Storage.transfer_flag = 1;
	}
}


static void act_AOS_servo_init(void)
{
	act_AOS_formatting();
	if(AOS_flag)
	{
		target_can2_203_angle = TARGET_CAN2_203_SERVO_INIT;
	}
	Aos_Event = AOS_ARM_OVERTURN_HORIZON;
	Auto_Ore_Storage.transfer_flag = 1;
}


static void act_AOS_arm_overturn_horizon(void)
{
	act_AOS_formatting();
	if(AOS_flag)
	{
		target_can2_204_angle = TARGET_AOS_CAN2_204_OVERTURN_HORIZON; //小臂翻转到水平
	}
	if(engineer_deadband_judge(target_can2_204_angle,TARGET_AOS_CAN2_204_OVERTURN_HORIZON,30.0f)) //待修改
	{
		return;
	}
	else
	{
		Aos_Event = AOS_RETRACT_FIRST;
		Auto_Ore_Storage.transfer_flag = 1;
	}
}

//static void act_AOS_retract_first(void)
//{
//	act_AOS_formatting();
//	if(AOS_flag)
//	{
//		target_can2_202_angle = TARGET_AOS_CAN2_202_RETRACT_FIRST; //小臂翻转到水平
//	}
//	if(engineer_deadband_judge(target_can2_202_angle,TARGET_AOS_CAN2_202_RETRACT_FIRST,30.0f)) //待修改
//	{
//		return;
//	}
//	else
//	{
//		Aos_Event = AOS_CYLINDER_OFF;
//		Auto_Ore_Storage.transfer_flag = 1;
//	}
//}
//static void act_AOS_cylinder_off(void)
//{
//	act_AOS_formatting();
//	if(AOS_flag)
//	{
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
//	}
//		Aos_Event = AOS_PROTRACT_FIRST;
//		Auto_Ore_Storage.transfer_flag = 1;
//}
//static void act_AOS_retract_first(void)
//{
//	act_AOS_formatting();
//	if(AOS_flag)
//	{
//		target_can2_202_angle = TARGET_AOS_CAN2_202_RETRACT_FIRST; //小臂翻转到水平
//	}
//	if(engineer_deadband_judge(target_can2_202_angle,TARGET_AOS_CAN2_202_RETRACT_FIRST,30.0f)) //待修改
//	{
//		return;
//	}
//	else
//	{
//		Aos_Event = AOS_CYLINDER_OFF;
//		Auto_Ore_Storage.transfer_flag = 1;
//	}
//}
//static void act_AOS_storage_ore_up(void)
//{
//	act_AOS_formatting();
//	if(AOS_flag)
//	{
//		target_can2_208_angle = TARGET_AOS_CAN2_208_STORAGE_UP; //小臂翻转
//	}
//	if(angle_can2_207_6020 > TARGET_AOS_CAN2_208_STORAGE_UP - 10) //待修改
//	{
//		return;
//	}
//	else
//	{
//		Aos_Event = AOS_RETRACT;
//		Auto_Ore_Storage.transfer_flag = 1;
//	}
//}
//static void act_AOS_retract(void)
//{
//	act_AOS_formatting();
//	if(AOS_flag)
//	{
//		target_can2_202_angle = TARGET_AOS_CAN2_202_RETRACT; //前伸收回
//	}
//	if(angle_can2_202 > TARGET_AOS_CAN2_202_RETRACT - 10) //待修改
//	{
//		return;
//	}
//	else
//	{
//		Aos_Event = AOS_RETRACT;
//		Auto_Ore_Storage.transfer_flag = 1;
//	}
//}
//--------------------------------------------------------自动置位连招函数-------------------------------------------------//

int AMI_flag;
static void act_AMI_Formatting(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_V) //Ctrl+V 为云台复位初始化
	{
		act_Formatting();
		Ami_Event = FORMATTING;
		Auto_Machine_Init.transfer_flag = 1;
	}
}
static void act_AMI_init(void)
{
	AMI_flag = 1;
	if( rc_ctrl.key.v == KEY_PRESSED_CTRL_G)  //按键Ctrl+g即可运行
	{
		Ami_Event = AMI_ROW_MIDDLE;
		Auto_Machine_Init.transfer_flag = 1;
	}
}
static void act_AMI_row_middle(void)
{
	act_AMI_Formatting();
	if(AMI_flag)
	{
		target_can2_203_angle = TARGET_AMI_CAN2_203_MIDDLE; //row轴中置
	}
	if(engineer_deadband_judge(angle_can2_203,TARGET_AMI_CAN2_203_MIDDLE,20))
	{
		Ami_Event = AMI_ARM_VERTICAL_HORIZON;
		Auto_Machine_Init.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_AMI_arm_vertical_horizon(void)
{
	act_AMI_Formatting();
	if(AMI_flag)
	{
		target_can2_204_angle = TARGET_AMI_CAN2_204_HORIZON; 
	}
	if(engineer_deadband_judge(angle_can2_204,TARGET_AMI_CAN2_204_HORIZON,20))
	{
		Ami_Event = AMI_ARM_RETURN;
		Auto_Machine_Init.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_AMI_arm_return(void)
{
	act_AMI_Formatting();
	if(AMI_flag)
	{
		target_can2_207_angle_6020 = -TARGET_AMI_CAN2_207_6020_HORIZON; 
	}
	if(engineer_deadband_judge(target_can2_207_angle_6020,-TARGET_AMI_CAN2_207_6020_HORIZON,20))
	{
		Ami_Event = AMI_PROTRACT;
		Auto_Machine_Init.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_AMI_protratc(void)
{
	act_AMI_Formatting();
	if(AMI_flag)
	{
		target_can2_202_angle = TARGET_AMI_CAN2_202_PROTRACT; 
	}
	if(engineer_deadband_judge(angle_can2_202,TARGET_AMI_CAN2_202_PROTRACT,40))
	{
		Ami_Event = AMI_SIDESWAY_MIDDLE;
		Auto_Machine_Init.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_AMI_sidesway_middle(void)  //横移中置
{
	act_AMI_Formatting();
	if(AMI_flag)
	{
		target_can2_201_angle = TARGET_AMI_CAN2_201_MIDDLE; 
	}
	if(engineer_deadband_judge(target_can2_201_angle,TARGET_AMI_CAN2_201_MIDDLE,30))
	{
		Ami_Event = AMI_DOWN;
		Auto_Machine_Init.transfer_flag = 1;
	}
	else
	{
		return;
	}
}
static void act_AMI_down(void)
{
	act_AMI_Formatting();
	if(AMI_flag)
	{
		target_can2_205_angle = TARGET_AMI_CAN2_205_ANGLE_DOWN; //抬升上升
		target_can2_206_angle = -20;
	}
	if(engineer_deadband_judge(angle_can2_205,TARGET_AMI_CAN2_205_ANGLE_DOWN,40))
	{
		Ami_Event = AMI_INIT;
		Auto_Machine_Init.transfer_flag = 1;
		AMI_flag = 0;
	}
	else
	{
		return;
	}
}
