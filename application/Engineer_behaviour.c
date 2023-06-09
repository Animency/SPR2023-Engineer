//����:2023���̿����ܺ���
//Author:Allen.Chopin
//����:���ļ��������̳����е�������׵Ŀ��ƺ���������gimbal_task.c�߳������á�

 /**************************************/
 /******      IDʹ��˵����        ******/
 /******   0: ����ƽ�Ƶ��        ******/
 /******   1������ƽ�Ƶ��        ******/
 /******   2����е��ˮƽ��ת���  ******/   //���޸�Ϊ6020�ҷ���IDΪ3����ʶ��Ϊ0x1ff,����IDΪ0x207
 /******   3����е����ֱ�������  ******/
 /******   4���෽̧�����        ******/
 /******   5���෽̧�����        ******/
 /**************************************/
 
 
 /******     2:���ϽǶ�����     ********/
 /******    3:��ʱ��Ƕ�����    ********/
 /******       4:�����װ       *******/
 /******       5:�����װ       *******/
 
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
#include "cmsis_armcc.h"  //��ͷ�ļ����������λ��������
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
//*****************************************************************�ṹ��ȡ����************************************************//
extern gimbal_control_t gimbal_control;
extern RC_ctrl_t rc_ctrl;
//*****************************************************************����������**************************************************//
void PID_All_Cal(void);
void act_Formatting(void);
//����Դ���սӶ���������������:
static void act_AG_Formatting(void);  //ǿ�Ƴ�ʼ��
static void act_AG_BigResourceIsland_Init(void);
static void act_AG_BigResourceIsland_Rising(void);
static void act_AG_BigResourceIsland_Center(void);
static void act_AG_BigResourceIsland_ProTract(void);
static void act_AG_BigResourceIsland_OverTurn(void);
static void act_AG_BigResourceIsland_Cylinder_On(void);
//����Դ������ȡ������������:
static void act_STM_Formatting(void);  //ǿ�Ƴ�ʼ��
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
//������к�����������:
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
//ȫ����е�ṹ��λ��������:
static void act_AMI_Formatting(void);
static void act_AMI_init(void);
static void act_AMI_row_middle(void);
static void act_AMI_arm_vertical_horizon(void);
static void act_AMI_arm_return(void);
static void act_AMI_protratc(void);
static void act_AMI_sidesway_middle(void);
static void act_AMI_down(void);
static void act_AMI_finish(void);
//������������̨������������:
void engineer_gimbal_behaviour_keyboard_control(void);
void engineer_keyboard_control_ordinal_coordinates(void);
void engineer_keyboard_control_except_coordinates(void);
void engineer_keyboard_control_mechanical_servo_coordinates(void);
void engineer_keyboard_control_mechanical_arm_coordinates(void);
void engineer_mouse_control(void);

//�����λ������������
__STATIC_INLINE void __set_FAULTMASK(uint32_t faultMask);
void SoftReset(void);

//�ж��Ƿ���������Χ��
int engineer_deadband_judge(float input,float deadline,float deadband);
//************************************************************PID�����������/������*******************************************//
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
//************************************************************����������*******************************************//
float give_angle_test = 0;
float give_angle_error_test = 40;
float relative_angle_to_mechanical_arm = 0;
float relative_angle_to_servo = 0;
//************************************************************����.C�ļ�����ȡ��������ݱ�����*********************************************//
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
//********************************************************��־λ������********************************************************//
uint8_t 	pump_flag_left 		= 0;
uint8_t 	pump_flag_right		= 0;
uint8_t 	laser_flag_left 	= 0;
uint8_t 	laser_flag_right	= 0;

uint8_t 	coordinates_control_flag = 0;
int pump10_flag=1;//�¶����־λ
int low_speed=0;//����ʱ�õ���ģʽ��־λ
int put_mine=1;//�˱�־λΪ�˽��ǰ�����鰴����ͻ����
int Speed_flag=1; //�˱�־λΪ�˽�����콵��������
int time_flag1=0;//�˱�־λΪ0.93s���ӳٱ�־λ
int time_flag2=0;//�˱�־λΪ0.4s���ӳٱ�־λ
int time_flag3=0;//�˱�־λΪ1.5s���ӳٱ�־λ
int gfo2_flag=1;//�˱�־λΪ������һ���ӳٱ�־λ
int aim_flag=0;
uint8_t STM_6020_flag = 1;
//�����ȡ������״̬
mouse_keyboard_state_flag_t mouse_keyboard_state_flag;
//***************************************************״̬��״̬���¼�������*******************************************************//
typedef enum{
	//����Դ���ս��¼�
	AG_INIT,   //��ʼ���¼�
	AG_RISING,  //̧���¼�
	AG_CENTER,   //�����¼�
	AG_PROTRACT,  //צ��ǰ���¼�
	AG_OVERTURN, //צ�ӷ�ת�¼�
	AG_CYLINDER_ON, //���״��¼�
	AG_BIGISLAND_FINISH, //����Դ���ս��¼����
	
	//С��Դ������ȡ���¼�
	STM_INIT,
	STM_START_RISING, //̧���¼�
	STM_ARM_PROTRACT, //ǰ���¼�
	STM_ARM_RETURN,   //��е�ۻ���
	STM_RISING_STEP_2, //�ٴ�̧��
	STM_RETRACT,      //�ջ��¼�
	STM_FALLING,      //�½��¼�
	STM_SERVO_INIT,   //�������
	STM_STORAGE_ORE_DOWN, //����½�
	STM_STORAGE_ORE_UP,   //�������
	STM_RETRACT_ORE,      //���ǰ�����
	STM_PROTRACT_OVERTURN_ORE,     //���ǰ��ǰ���뷭ת
	STM_ARM_RETURN_ORE,						 //��е�ۻ���
	
	//�Զ���ȡ��ʯ�¼� ��״̬������Ϊ���������������̧���߶����⵼�¶ҿ�����
	AOS_INIT,          		         //̧���½���ʼ��
	AOS_HORIZON_CENTER,            //���ƻ���
	AOS_ARM_OVERTURN,   	 	       //��е�۷�ת
	AOS_ARM_TURNAROUND,  		       //��е�ۻص�����
	AOS_SERVO_INIT,                //�������
	AOS_ARM_OVERTURN_HORIZON,      //��е�۷�ת��ˮƽ
	AOS_RETRACT_FIRST,        	 	 //ǰ���һ���ջ�
	AOS_CYLINDER_OFF,       		   //���ùر�
	AOS_PROTRACT_FIRST,            //��һ��΢�� 
	AOS_STORAGE_ORE_UP,  		       //�������
	AOS_CYLINDER_ON,               //���ô�
	AOS_RETRACT_SECOND,            //ǰ��ڶ����ջ�
	AOS_PROTRACT_SECOND,					 //ǰ��ڶ������
	AOS_STORAGE_ORE_DOWN,          //����½�
	AOS_ARM_RETURN,                //��е�ۻ���
	AOS_RETRACT_THIRD,						 //ǰ��������ջ�
	AOS_FINISH,										 //�Զ�������
	
	//�Զ���е�۳�ʼ���¼�
	AMI_INIT,
	AMI_ROW_MIDDLE,           //2006����
	AMI_ARM_VERTICAL_HORIZON, //��е�۷�תˮƽ
	AMI_ARM_RETURN,           //��е�ۻ���
	AMI_PROTRACT,             //ǰ�����
	AMI_SIDESWAY_MIDDLE,      //��������
	AMI_DOWN,                 //̧���½�

	//ǿ�Ƴ�ʼ���¼�
	FORMATTING
}Event_ID;

typedef enum{
	//����Դ���ս�״̬
	AG_INIT_STATE,
	AG_RISING_STATE,
	AG_CENTER_STATE,   //����״̬
	AG_PROTRACR_STATE, //ǰ��״̬
	AG_OVERTURN_STATE,  //ǰצ��ת״̬
	AG_CYLINDER_ON_STATE, //���״�״̬
	
	//С��Դ������ȡ��״̬
	STM_INIT_STATE,
	STM_START_RISING_STATE, //̧��״̬
	STM_ARM_PROTRACT_STATE, //ǰ��״̬
	STM_ARM_RETURN_STATE,   //����״̬
	STM_RISING_STEP_2_STATE, //�ٴ�����
	STM_RETRACT_STATE,      //�ջ�״̬
	STM_FALLING_STATE,      //�½�״̬
	STM_SERVO_INIT_STATE,   //�������
	STM_CYLINDER_OFF_STATE, //�ر�����
	STM_STORAGE_ORE_DOWN_STATE, //����½�
	STM_STORAGE_ORE_UP_STATE,   //�������
	STM_RETRACT_ORE_STATE,      //���ǰ�����
	STM_PROTRACT_OVERTURN_ORE_STATE,     //���ǰ��ǰ���뷭ת
	STM_ARM_RETURN_ORE_STATE,						 //��е�ۻ���
	
	
	
	//�Զ���ȡ��ʯ�¼� ��״̬������Ϊ���������������̧���߶����⵼�¶ҿ�����
	AOS_INIT_STATE,          		         //̧���½���ʼ��
	AOS_HORIZON_CENTER_STATE,            //���ƻ���
	AOS_ARM_OVERTURN_STATE,   	 	       //��е�۷�ת
	AOS_ARM_TURNAROUND_STATE,  		       //��е�ۻص�����
	AOS_SERVO_INIT_STATE,                //�������
	AOS_ARM_OVERTURN_HORIZON_STATE,      //��е�۷�ת��ˮƽ
	AOS_RETRACT_FIRST_STATE,        	 	 //ǰ���һ���ջ�
	AOS_CYLINDER_OFF_STATE,       		   //���ùر�
	AOS_PROTRACT_FIRST_STATE,            //��һ��΢�� 
	AOS_STORAGE_ORE_UP_STATE,  		       //�������
	AOS_CYLINDER_ON_SATET,               //���ô�
	AOS_RETRACT_SECOND_STATE,            //ǰ��ڶ����ջ�
	AOS_PROTRACT_SECOND_STATE,					 //ǰ��ڶ������
	AOS_STORAGE_ORE_DOWN_STATE,          //����½�
	AOS_ARM_RETURN_STATE,                //��е�ۻ���
	AOS_RETRACT_THIRD_STATE,						 //ǰ��������ջ�
	AOS_FINISH_STATE,										 //�Զ�������
	
		//�Զ���е�۳�ʼ���¼�
	AMI_INIT_STATE,
	AMI_ROW_MIDDLE_STATE,           //2006����״̬
	AMI_ARM_VERTICAL_HORIZON_STATE, //��е�۷�תˮƽ״̬
	AMI_ARM_RETURN_STATE,           //��е�ۻ���״̬
	AMI_PROTRACT_STATE,             //ǰ�����״̬
	AMI_SIDESWAY_MIDDLE_STATE,      //��������״̬
	AMI_DOWN_STATE,                 //̧���½�״̬
	
}State;
//------------------------------------------------------����״̬��������--------------------------------------------------------------------------//
FSM Air_Get;//����Դ���ս��¼�����

FsmTable ag_table[]={
	{AG_INIT_STATE				,AG_INIT							,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_RISING_STATE			,AG_RISING						,AG_CENTER_STATE						  ,act_AG_BigResourceIsland_Rising},
	{AG_CENTER_STATE			,AG_CENTER					  ,AG_PROTRACR_STATE						,act_AG_BigResourceIsland_Center},
	{AG_PROTRACR_STATE		,AG_PROTRACT					,AG_OVERTURN_STATE						,act_AG_BigResourceIsland_ProTract},
	{AG_OVERTURN_STATE		,AG_OVERTURN					,AG_CYLINDER_ON_STATE					,act_AG_BigResourceIsland_OverTurn},
	{AG_CYLINDER_ON_STATE	,AG_BIGISLAND_FINISH	,AG_INIT_STATE								,act_AG_BigResourceIsland_Cylinder_On},

  //��ʱ����ǿ�Ƴ�ʼ��λ��
	{AG_INIT_STATE				,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_RISING_STATE			,FORMATTING					  ,AG_RISING_STATE						  ,act_AG_BigResourceIsland_Init},
	{AG_CENTER_STATE			,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_PROTRACR_STATE		,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_OVERTURN_STATE		,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
	{AG_CYLINDER_ON_STATE	,FORMATTING						,AG_RISING_STATE							,act_AG_BigResourceIsland_Init},
};
int Ag_Event; //����Դ���¼�����
//_________________________________________________________����Դ������ȡ��___________________________________________________________________________//
FSM Small_Take_Mine; //С��Դ������ȡ���¼� 

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
//____________________________________________________________�Զ����___________________________________________________________________________//
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

//____________________________________________________________���л�е�������λ____________________________________________________________________//
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
//--------------------------------------------------------״̬����ʼ������----------------------------------------------------------//
void fsm_init(void)
{
	//����Դ���սӳ�ʼ��
	FSM_Regist(&Air_Get, ag_table);
	Air_Get.size = 12;
	FSM_StateTransfer(&Air_Get,AG_INIT_STATE);
	Ag_Event = AG_INIT;
	
	//����Դ������ȡ���ʼ��
	FSM_Regist(&Small_Take_Mine, stm_table);
	Small_Take_Mine.size = 26;
	FSM_StateTransfer(&Small_Take_Mine,STM_INIT_STATE);
	Stm_Event = STM_INIT;
	
	//�Զ���λ��ʼ��
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
//--------------------------------------------------------״̬�����к���-------------------------------------------------//

/**
 * @brief          ״̬�����к��������ļ��������ܺ�������gimbal_task.c�������
 *
 * @param[out]     none
 * @retval         none
 */
void fsm_run(void)
{
	//��������־����  �˴�������˫��ͨ��ʹ�� ���ڷ�˫��ͨ��״̬�ɼ��̿����ĸ������ĸ���
//	pump_flag_left 		= 	gimbal_control.ore_flag.air_pump_flag_left;
//	pump_flag_right 	= 	gimbal_control.ore_flag.air_pump_flag_right;
//	laser_flag_left		=		gimbal_control.ore_flag.laser_flag_left;
//	laser_flag_right  =   gimbal_control.ore_flag.laser_flag_right;
	
	//����Դ���ս�״̬��������
	FSM_EventHandle(&Air_Get						,Ag_Event); 
	FSM_EventHandle(&Small_Take_Mine		,Stm_Event);
	FSM_EventHandle(&Auto_Machine_Init	,Ami_Event);
	
	//���̿��ƴ�����
	engineer_gimbal_behaviour_keyboard_control();
	
	//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
	electric_limit(target_can2_201_angle			,target_can2_201_angle			,TARGET_CAN2_201_MAX			,TARGET_CAN2_201_MIN);
	electric_limit(target_can2_202_angle			,target_can2_202_angle			,TARGET_CAN2_202_MAX			,TARGET_CAN2_202_MIN);
	electric_limit(target_can2_203_angle			,target_can2_203_angle			,TARGET_CAN2_203_MAX			,TARGET_CAN2_203_MIN);
	electric_limit(target_can2_204_angle			,target_can2_204_angle			,TARGET_CAN2_204_MAX			,TARGET_CAN2_204_MIN);
	electric_limit(target_can2_205_angle			,target_can2_205_angle			,TARGET_CAN2_205_MAX			,TARGET_CAN2_205_MIN);
	electric_limit(target_can2_206_angle			,target_can2_206_angle			,TARGET_CAN2_206_MAX			,TARGET_CAN2_206_MIN);
	electric_limit(target_can2_207_angle_6020	,target_can2_207_angle_6020	,TARGET_CAN2_207_6020_MAX	,TARGET_CAN2_207_6020_MIN);
	electric_limit(target_can2_208_angle			,target_can2_208_angle			,TARGET_CAN2_208_MAX			,TARGET_CAN2_208_MIN);

	//��̧����һ���߶�ʱ�л�Ϊ����ģʽ
	if(angle_can2_205 > CHASSIS_LOW_SPEED_ANGLE && low_speed == 0) 
	{
		low_speed = 2;
	}
	
	PID_All_Cal();
	
	SoftReset();
}
void PID_All_Cal(void)
{
	//�˴�ΪPID˫��
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
		case 0: //��һ�˳ƿ���
			engineer_keyboard_control_ordinal_coordinates();
			break;
		case 1: //�Ի�е��Ϊ�������
			engineer_keyboard_control_mechanical_arm_coordinates();
			break;
		case 2: //�Զ������λ��Ϊ�������
		  engineer_keyboard_control_mechanical_servo_coordinates();
		default:
			break;
	}
	engineer_keyboard_control_except_coordinates();
}
void engineer_keyboard_control_except_coordinates(void) //������ϵת����ĵ������
{
	//��̨��е��ˮƽ������̿���
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_Q) 		 							//���̰���[SHIFT+Q]ʱ��е��ˮƽ��������ƶ�
	{
		target_can2_207_angle_6020 += KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_E) 							//���̰���[SHIFT+E]ʱ��е��ˮƽ�����ƶ�
	{
		target_can2_207_angle_6020 -= KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
	}
	
	//��̨��е����ֱ������̿���
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_Z) 		 								//���̰���[SHIFT+Z]ʱ��е����ֱ��������ƶ�
	{
		target_can2_204_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_X) 							//���̰���[SHIFT+X]ʱ��е����ֱ��������ƶ�
	{
		target_can2_204_angle += KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
	}
	
	//��̨̧�����̿���
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_R) 		 								//���̰���[SHIFT+R]ʱ̧����������ƶ�
	{
		target_can2_205_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_F) 		 						//���̰���[SHIFT+F]ʱ̧����������ƶ�
	{
		target_can2_205_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
	}
	
	//���̿���������д��chassis_task��
	
	if(!((gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) || (gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)))
	{
		//��̨������ÿ���
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) 		 								//���̰���Gʱ����
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,19999);
			pump_flag_left = 1;
			pump_flag_right = 1;
			draw_card_position(send_id, receive_id, 2);
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)               //���̰���Vʱ�ر�
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
			pump_flag_left = 0;
			pump_flag_right = 0;
			draw_card_position(send_id, receive_id, 2);
		}
		
		//���������
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
	//W+S ǰ��
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_W)               //���̰���[SHIFT+W]ʱ��������ǰ�ƶ�
	{
		target_can2_202_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm 	* ANGLE_TO_ECD) * cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_201_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	* ANGLE_TO_ECD)	* cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_205_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
		target_can2_206_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_S)					//���̰���[SHIFT+S]ʱ����������ƶ�
	{
		target_can2_202_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm 	* ANGLE_TO_ECD) * cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_201_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	* ANGLE_TO_ECD)	* cos( relative_angle_to_servo * ANGLE_TO_ECD);
		target_can2_205_angle -= KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
		target_can2_206_angle += KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin( relative_angle_to_servo * ANGLE_TO_ECD) * RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE;
	}
	//A+D ����
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_A)               //���̰���[SHIFT+A]ʱ�����������ƶ�
	{
		target_can2_202_angle += (-KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle += ( KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_D)					//���̰���[SHIFT+S]ʱ�����������ƶ�
	{
		target_can2_202_angle -= (-KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle -= ( KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
}
void engineer_keyboard_control_mechanical_arm_coordinates(void)
{
	//W+S ǰ��
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_W)               //���̰���[SHIFT+W]ʱ��������ǰ�ƶ�
	{
		target_can2_202_angle -= KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE	*	sin(	relative_angle_to_mechanical_arm 	* ANGLE_TO_ECD) ;
		target_can2_201_angle -= KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	* ANGLE_TO_ECD)	;
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_S)					//���̰���[SHIFT+S]ʱ����������ƶ�
	{
		target_can2_202_angle += KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE *	sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD) ;
		target_can2_201_angle += KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD) ;
	}
	//A+D ����
	if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_A)               //���̰���[SHIFT+A]ʱ�����������ƶ�
	{
		target_can2_202_angle += (-KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle += ( KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v == KEY_PRESSED_SHIFT_D)					//���̰���[SHIFT+S]ʱ�����������ƶ�
	{
		target_can2_202_angle -= (-KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * cos(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
		target_can2_201_angle -= ( KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE * sin(	relative_angle_to_mechanical_arm	*ANGLE_TO_ECD));
	}
}
void engineer_keyboard_control_ordinal_coordinates(void)   //��ͼ��Ϊ����Ŀ��� ��һ�˳�
{
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)
	{
			//���������̿���
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W)               //���̰���[SHIFT+W]ʱ��������ǰ�ƶ�
		{
			target_can2_202_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_202_CHANGE ;
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S)					//���̰���[SHIFT+S]ʱ����������ƶ�
		{
			target_can2_202_angle += KEYBOARD_CONTROL_ANGLE_CAN2_202_CHANGE ;
		}
		//���������̿���
		if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A)               //���̰���[SHIFT+A]ʱ�����������ƶ�
		{
			target_can2_201_angle += KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE ;
		}
		else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)					//���̰���[SHIFT+D]ʱ�����������ƶ�
		{
			target_can2_201_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE ;
		}
	}
}
void engineer_mouse_control(void)
{
	//������������ʱ���趨Ϊ���ƶ��
	if(mouse_keyboard_state_flag.current_mouse_state & MOUSE_PRESSED_OFFSET_LEFT)
	{
		target_can2_203_angle -= 4;
//		//��һ��״̬Ϊ�ɿ�ʱ
//		if(!(mouse_keyboard_state_flag.former_mouse_state & MOUSE_PRESSED_OFFSET_LEFT))
//		{
//			target_can2_203_angle += 500;
//		}
	}
	//������Ҽ�����ʱ
	if(mouse_keyboard_state_flag.current_mouse_state & MOUSE_PRESSED_OFFSET_RIGHT)
	{
		target_can2_203_angle += 4;
		//��һ��״̬Ϊ�ɿ�ʱ
//		if(!(mouse_keyboard_state_flag.former_mouse_state & MOUSE_PRESSED_OFFSET_RIGHT))
//		{
//			target_can2_203_angle -= 500;
//		}
	}
}

//�����λ��Ƭ��
void SoftReset(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_Q) //Ctrl+Q��ʼ�����λ
	{
		__set_FAULTMASK(1); //�ر������ж�
		NVIC_SystemReset(); //��λ
	}
}


void act_Formatting(void)
{
	//yaw����� �˺���������һ����״̬�����¼��صĺ�������û�кܸ��ӵĲ��������Ҫ����ȫ����λ�Ļ�����Ӧ������дһ��״̬�����ڻظ�
	target_can2_203_angle = 90;
	target_can2_207_angle_6020 = TARGET_CAN2_207_6020_ANGLE_FORMATTING;
}
//����Դ���սӶ����������������
int ag_flag=1;
int start_flag=0;

//����Դ���ս����а���shift+f��������
static void act_AG_Formatting(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_R)//shift+rǿ�Ƴ�ʼ��
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
	if(rc_ctrl.key.v==KEY_PRESSED_CTRL_B)//����ctrl+B�������� ����ʱ��ʱʹ�ú����
	{	
		low_speed=2;//�뿪����ģʽ  [���޸�] ��Ӧ����
		target_can2_202_angle = TARGET_AG_CAN2_202_ANGLE_INIT; //�ջ���ֱ���
		target_can2_204_angle = TARGET_AG_CAN2_204_ANGLE_INIT;  //��е��צ�ջ�
		start_flag=1;
	}
	if(start_flag) 
	{
		if(angle_can2_202>20) // �����ж��Ƿ�ﵽ��ʼ��λ��
				return;
		else //���ﵽԤ��ֵʱ
		{
			target_can2_205_angle = TARGET_AG_CAN2_205_ANGLE_INIT; //̧���½�
			target_can2_206_angle = -TARGET_AG_CAN2_206_ANGLE_INIT; //̧���½�
			if(angle_can2_205 < 10) // �����ж��Ƿ��½����
			{
				target_can2_207_angle_6020 = TARGET_AG_CAN2_207_ANGLE_6020_INIT;
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0); //�̵����͵�ƽ��ʧ��
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0); //ͬ��
			}
			Ag_Event = AG_RISING;
			Air_Get.transfer_flag = 1;
		}
	}
	
}
static void act_AG_BigResourceIsland_Rising(void)
{
	if(ag_flag)//ag_flagΪû�н��г�ʼ���ı�־���˴���д����ʹ��
	{
		start_flag = 0;
		target_can2_205_angle = TARGET_AG_CAN2_205_ANGLE_RISING; //̧������
		target_can2_206_angle = -TARGET_AG_CAN2_206_ANGLE_RISING;
	}
	if(angle_can2_205 < TARGET_AG_CAN2_205_ANGLE_RISING-100) //���޸�
	{
		return;
	}
	else
	{
		Ag_Event = AG_CENTER; //��һ���¼�Ϊ����
		Air_Get.transfer_flag = 1;
	}
}
static void act_AG_BigResourceIsland_Center(void)
{
	act_AG_Formatting();
	if(ag_flag)//ag_flagΪû�н��г�ʼ���ı�־���˴���д����ʹ��
	{
		target_can2_207_angle_6020 = TARGET_AG_CAN2_207_ANGLE_6020_CENTER; //��е������
	}
	if(angle_can2_207_6020 < TARGET_AG_CAN2_207_ANGLE_6020_CENTER - 20) //���޸�
	{
		return;
	}
	else
	{
		Ag_Event = AG_PROTRACT; //��һ���¼�Ϊǰ��
		Air_Get.transfer_flag = 1;
	}
}
static void act_AG_BigResourceIsland_ProTract(void)
{
	act_AG_Formatting();
	if(ag_flag)
	{
		target_can2_202_angle = -TARGET_AG_CAN2_202_ANGLE_PROTRACTING;  //ǰ��
		if(angle_can2_202 > (50-TARGET_AG_CAN2_202_ANGLE_PROTRACTING )) //���޸�
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
		target_can2_204_angle = TARGET_AG_CAN2_204_ANGLE_OVERTURN;  //��ת
		if(angle_can2_204 < TARGET_AG_CAN2_204_ANGLE_OVERTURN - 20) //���޸�
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
		//�ѽ����ʲ�ִ��
	}
}
//--------------------------------------------------------����Դ������ȡ�����к���-------------------------------------------------//
int STM_Flag;
static void act_STM_Formatting(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_X) //Ctrl+Xǿ�Ƴ�ʼ��
	{
		act_Formatting();
		Stm_Event = FORMATTING;
		Small_Take_Mine.transfer_flag = 1;
		STM_Flag = 0;
	}
}
static void act_STM_BigResourceIsland_Init(void)
{
	if( rc_ctrl.key.v == KEY_PRESSED_CTRL_F)  //����Ctrl+f��������
	{
		low_speed=2;//�������ģʽ 
		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_INIT; //�ջ���ֱ���
		target_can2_204_angle = TARGET_BTM_CAN2_204_ANGLE_INIT;  //��е��צ�ջ�
		STM_Flag = 1;
	}
	if(STM_Flag) 
	{
		if(angle_can2_202>20) //��10���޸� �����ж��Ƿ�ﵽ��ʼ��λ��
				return;
		else //���ﵽԤ��ֵʱ
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
		target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_RISING; //̧������
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
		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_Protract; //ǰ��
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
//�˺������ڻ�е������������ǰ��
static void act_STM_BigResourceIsland_ArmReturn(void)
{
	act_STM_Formatting();
	if(STM_Flag)
	{
		target_can2_207_angle_6020 = TARGET_BTM_CAN2_207_ANGLE_6020_RETURN; //����
	}
	if(fabs(angle_can2_207_6020) >  TARGET_BTM_CAN2_207_ANGLE_6020_RETURN + 3 || fabs(angle_can2_207_6020) < TARGET_BTM_CAN2_207_ANGLE_6020_RETURN - 3) //���޸�
	{
		return;
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999);//������
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
		target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_RISING_STEP_2; //̧������ ȡ��С��Դ��
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
		target_can2_202_angle = -200.0;  //ǰ���ջ�
	}
	if(engineer_deadband_judge(angle_can2_202,-TARGET_BTM_CAN2_202_ANGLE_RETRACR,40))
	{
		target_can2_204_angle = TARGET_BTM_CAN2_204_ANGLE_OVERTURN; //��ǰ���ջصĹ����л�е�۷�ת����
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
		if(target_can2_207_angle_6020 == TARGET_STM_CAN2_207_6020_ANGLE_ORE) //��ǰ���ջصĹ�����6020�ص������
		{
			STM_6020_flag = 0;
			target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_FALLING; //̧���½�
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
		target_can2_203_angle = 0; //���ֵ���޸�
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
		target_can2_208_angle = TARGET_BTM_CAN2_208_ANGLE_DOWN;  //����½�
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
//�ڴ���½�����ֶ�����ȡ��
static void act_STM_BigResourceIsland_Storage_up(void)
{
	act_STM_Formatting();
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_R)
	{
		STM_Flag = 1; //�ɼ�������ĺ���
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

//--------------------------------------------------------�Զ�������к���-------------------------------------------------//
int AOS_flag;
static void act_AOS_formatting(void)
{
	if(rc_ctrl.key.v == 1000) //���޸�
	{
		act_Formatting();
		Aos_Event = FORMATTING;
		Auto_Ore_Storage.transfer_flag = 1;
	}
}
static void act_AOS_init(void)
{
	AOS_flag = 1;
	if(rc_ctrl.key.v == 6666) //ctrl+I���Ʊ�����
	{
		target_can2_205_angle = TARGET_AOS_CAN2_205_ANGLE_DOWN; //̧���½�
		target_can2_206_angle = -TARGET_AOS_CAN2_206_ANGLE_DOWN;
	}
	if(engineer_deadband_judge(angle_can2_205,TARGET_AOS_CAN2_205_ANGLE_DOWN,40.0)) //���޸�
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
		target_can2_202_angle = TARGET_AOS_CAN2_202_HORIZON_CENTER; //����ˮƽ����
	}
	if(engineer_deadband_judge(angle_can2_202,TARGET_AOS_CAN2_202_HORIZON_CENTER,20.0)) //���޸�
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
//		target_can2_204_angle = TARGET_AOS_CAN2_204_ARM_OVERTURN; //С�۷�ת
//	}
//	if(engineer_deadband_judge(angle_can2_204,TARGET_AOS_CAN2_204_ARM_OVERTURN,40.0)) //���޸�
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
		target_can2_207_angle_6020 = TARGET_AOS_CAN2_207_6020_ARM_TURNAROUND; //С�۷�ת
	}
	if(engineer_deadband_judge(angle_can2_207_6020,TARGET_AOS_CAN2_207_6020_ARM_TURNAROUND,40.0)) //���޸�
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
		target_can2_204_angle = TARGET_AOS_CAN2_204_OVERTURN_HORIZON; //С�۷�ת��ˮƽ
	}
	if(engineer_deadband_judge(target_can2_204_angle,TARGET_AOS_CAN2_204_OVERTURN_HORIZON,30.0f)) //���޸�
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
//		target_can2_202_angle = TARGET_AOS_CAN2_202_RETRACT_FIRST; //С�۷�ת��ˮƽ
//	}
//	if(engineer_deadband_judge(target_can2_202_angle,TARGET_AOS_CAN2_202_RETRACT_FIRST,30.0f)) //���޸�
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
//		target_can2_202_angle = TARGET_AOS_CAN2_202_RETRACT_FIRST; //С�۷�ת��ˮƽ
//	}
//	if(engineer_deadband_judge(target_can2_202_angle,TARGET_AOS_CAN2_202_RETRACT_FIRST,30.0f)) //���޸�
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
//		target_can2_208_angle = TARGET_AOS_CAN2_208_STORAGE_UP; //С�۷�ת
//	}
//	if(angle_can2_207_6020 > TARGET_AOS_CAN2_208_STORAGE_UP - 10) //���޸�
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
//		target_can2_202_angle = TARGET_AOS_CAN2_202_RETRACT; //ǰ���ջ�
//	}
//	if(angle_can2_202 > TARGET_AOS_CAN2_202_RETRACT - 10) //���޸�
//	{
//		return;
//	}
//	else
//	{
//		Aos_Event = AOS_RETRACT;
//		Auto_Ore_Storage.transfer_flag = 1;
//	}
//}
//--------------------------------------------------------�Զ���λ���к���-------------------------------------------------//

int AMI_flag;
static void act_AMI_Formatting(void)
{
	if(rc_ctrl.key.v == KEY_PRESSED_CTRL_V) //Ctrl+V Ϊ��̨��λ��ʼ��
	{
		act_Formatting();
		Ami_Event = FORMATTING;
		Auto_Machine_Init.transfer_flag = 1;
	}
}
static void act_AMI_init(void)
{
	AMI_flag = 1;
	if( rc_ctrl.key.v == KEY_PRESSED_CTRL_G)  //����Ctrl+g��������
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
		target_can2_203_angle = TARGET_AMI_CAN2_203_MIDDLE; //row������
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
static void act_AMI_sidesway_middle(void)  //��������
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
		target_can2_205_angle = TARGET_AMI_CAN2_205_ANGLE_DOWN; //̧������
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
