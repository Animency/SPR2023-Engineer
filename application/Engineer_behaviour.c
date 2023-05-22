//����:2023���������ܺ���
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
static void act_BTM_Formatting(void);  //ǿ�Ƴ�ʼ��
static void act_BTM_BigResourceIsland_Rising(void);
static void act_BTM_BigResourceIsland_ArmReturn(void);
static void act_BTM_BigResourceIsland_Retract(void);
static void act_BTM_BigResourceIsland_OverTurn(void);
static void act_BTM_BigResourceIsland_Servo_Center(void);
static void act_BTM_BigResourceIsland_Storage_Down(void);
//���̿�����̨������������:
void engineer_gimbal_behaviour_keyboard_control(void);
//************************************************************PID�����������/������*******************************************//
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
//************************************************************����.C�ļ�����ȡ��������ݱ�����*********************************************//
extern float angle_can2_201;
extern float angle_can2_202;
extern float angle_can2_204;
extern float angle_can2_205;
extern float angle_can2_206;
extern float angle_can2_207_6020;
extern float angle_can2_208;


//********************************************************��־λ������********************************************************//
uint8_t 	pump_flag_left 		= 0;
uint8_t 	pump_flag_right		= 0;
uint8_t 	laser_flag_left 	= 0;
uint8_t 	laser_flag_right	= 0;
int pump5_flag=1;
int pump6_flag=1;
int pump7_flag=1;
int pump8_flag=1;
int pump9_flag=1;
int pump10_flag=1;//�¶����־λ
int pump11_flag=1;
int pump12_flag=1;
int low_speed=0;//����ʱ�õ���ģʽ��־λ
int put_mine=1;//�˱�־λΪ�˽��ǰ�����鰴����ͻ����
int Speed_flag=1; //�˱�־λΪ�˽�����콵��������
int time_flag1=0;//�˱�־λΪ0.93s���ӳٱ�־λ
int time_flag2=0;//�˱�־λΪ0.4s���ӳٱ�־λ
int time_flag3=0;//�˱�־λΪ1.5s���ӳٱ�־λ
int gfo2_flag=1;//�˱�־λΪ������һ���ӳٱ�־λ
int aim_flag=0;
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
	
	//����Դ������ȡ���¼�
	BTM_START_RISING, //̧���¼�
	BTM_ARM_RETURN,   //��е�ۻ���
	BTM_RETRACT,      //�ջ��¼�
	BTM_OVERTURN,     //��ת�¼�
	BTM_SERVO_INIT,   //�������
	BTM_STORAGE_ORE_DOWN, //����½�
	BTM_FISISH,
	
	
	
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
	
	//����Դ������ȡ��״̬
	BTM_START_RISING_STATE, //̧��״̬
	BTM_ARM_RETURN_STATE,   //����״̬
	BTM_RETRACT_STATE,      //�ջ�״̬
	BTM_SERVO_INIT_STATE,   //�������
	BTM_CYLINDER_OFF_STATE, //�ر�����
	BTM_STORAGE_ORE_DOWN_STATE, //����½�
	
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
	{AG_INIT_STATE				,FORMATTING						,AG_RISING_STATE							,act_AG_Formatting},
	{AG_RISING_STATE			,FORMATTING					  ,AG_CENTER_STATE						  ,act_AG_Formatting},
	{AG_CENTER_STATE			,FORMATTING						,AG_PROTRACR_STATE						,act_AG_Formatting},
	{AG_PROTRACR_STATE		,FORMATTING						,AG_OVERTURN_STATE						,act_AG_Formatting},
	{AG_OVERTURN_STATE		,FORMATTING						,AG_CYLINDER_ON_STATE					,act_AG_Formatting},
	{AG_CYLINDER_ON_STATE	,FORMATTING						,AG_INIT_STATE								,act_AG_Formatting},
};
int Ag_Event; //����Դ���¼�����
//_______________________________________________________________________________
FSM Big_Take_Mine; //����Դ������ȡ���¼�

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
//--------------------------------------------------------״̬����ʼ������----------------------------------------------------------//
void fsm_init(void)
{
	//����Դ���սӳ�ʼ��
	FSM_Regist(&Air_Get, ag_table);
	Air_Get.size = 12;
	FSM_StateTransfer(&Air_Get,AG_INIT_STATE);
	Ag_Event = AG_INIT;
	
	//����Դ������ȡ���ʼ��
//	FSM_Regist(&Big_Take_Mine, btm_table);
//	Air_Get.size = 10;
//	FSM_StateTransfer(&Big_Take_Mine,AG_INIT_STATE);
//	Btm_Event = BTM_START_RISING_STATE;
}
//--------------------------------------------------------״̬�����к���-------------------------------------------------//
void fsm_run(void)
{
	//��������־����
	pump_flag_left 		= 	gimbal_control.ore_flag.air_pump_flag_left;
	pump_flag_right 	= 	gimbal_control.ore_flag.air_pump_flag_right;
	laser_flag_left		=		gimbal_control.ore_flag.laser_flag_left;
	laser_flag_right  =   gimbal_control.ore_flag.laser_flag_right;
	FSM_EventHandle(&Air_Get,Ag_Event); //����Դ���ս�״̬��������
	
	
	engineer_gimbal_behaviour_keyboard_control();//���̿��ƴ�����
	PID_All_Cal();
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

	pid_test_can2_201 = gimbal_control.horizontal_scroll_motor[0].give_current;
	pid_test_can2_202 = gimbal_control.horizontal_scroll_motor[1].give_current;
	pid_test_can2_207_angle_6020 = gimbal_control.gimbal_6020_motor.give_current;
	pid_test_can2_204 = gimbal_control.horizontal_scroll_motor[3].give_current;
	pid_test_can2_205 = gimbal_control.horizontal_scroll_motor[4].give_current;
	pid_test_can2_206 = gimbal_control.horizontal_scroll_motor[5].give_current;
}

void engineer_gimbal_behaviour_keyboard_control(void)
{
	//��̨��е��ˮƽ������̿���
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q) 		 								//���̰���Qʱ��е��ˮƽ��������ƶ�
	{
		target_can2_207_angle_6020 += KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_207_angle_6020,target_can2_207_angle_6020,TARGET_CAN2_207_6020_MAX,TARGET_CAN2_207_6020_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) 								//���̰���Eʱ��е��ˮƽ�����ƶ�
	{
		target_can2_207_angle_6020 -= KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_207_angle_6020,target_can2_207_angle_6020,TARGET_CAN2_207_6020_MAX,TARGET_CAN2_207_6020_MIN);
	}
	
	//��̨��е����ֱ���̿���
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R) 		 								//��갴��Rʱǰ������ǰ�ƶ�
	{
		target_can2_204_angle += KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_204_angle,target_can2_204_angle,TARGET_CAN2_204_MAX,TARGET_CAN2_204_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F) 		 						//���̰���Rʱǰ��������ƶ�
	{
		target_can2_204_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_204_angle,target_can2_204_angle,TARGET_CAN2_204_MAX,TARGET_CAN2_204_MIN);
	}
	
	//��̨̧�����̿���
	if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C) 		 								//��갴��Cʱ̧����������ƶ�
	{
		target_can2_205_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_205_angle,target_can2_205_angle,TARGET_CAN2_205_MAX,TARGET_CAN2_205_MIN);
		electric_limit(target_can2_206_angle,target_can2_206_angle,TARGET_CAN2_206_MAX,TARGET_CAN2_205_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V) 		 						//��갴��Vʱ̧����������ƶ�
	{
		target_can2_205_angle -= KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		target_can2_206_angle += KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_205_angle,target_can2_205_angle,TARGET_CAN2_205_MAX,TARGET_CAN2_205_MIN);
		electric_limit(target_can2_206_angle,target_can2_206_angle,TARGET_CAN2_206_MAX,TARGET_CAN2_206_MIN);
	}
	
	//��̨���Ƶ�����̿���
	if(gimbal_control.gimbal_rc_ctrl->mouse.press_l & 0x01) 		 						 						//��갴�����ʱ���Ƶ�������ƶ�
	{
		target_can2_201_angle += MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_201_angle,target_can2_201_angle,TARGET_CAN2_201_MAX,TARGET_CAN2_201_MIN);
	}
	else if(gimbal_control.gimbal_rc_ctrl->mouse.press_r & 0x01) 		 										//��갴���Ҽ�ʱ���Ƶ�������ƶ�
	{
		target_can2_201_angle -= MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_201_angle,target_can2_201_angle,TARGET_CAN2_201_MAX,TARGET_CAN2_201_MIN);
	}
	
	//��̨��ֱ������̿���
	if(gimbal_control.gimbal_rc_ctrl->mouse.y | 0x00)                                   //���ǰ���ƶ�ʱǰ��ǰ���ƶ�
	{
		target_can2_202_angle += gimbal_control.gimbal_rc_ctrl->mouse.y * MOUSE_CONTROL_ANGLE_CAN2_202_CHANGE;
		//���ӵ�����λ���ܷ�ֹ�趨ֵ����������λ�޷���ʱ��Ӧ
		electric_limit(target_can2_202_angle,target_can2_202_angle,TARGET_CAN2_202_MAX,TARGET_CAN2_202_MIN);
	}
}
//�ݿ�
void act_Formatting(void)
{
	
}
//����Դ���սӶ����������������
int ag_flag=1;
int start_flag=0;

//����Դ���ս����а���shift+f��������
static void act_AG_Formatting(void)
{
	if(rc_ctrl.key.v == 272)//shift+rǿ�Ƴ�ʼ��
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
	if(1/*rc_ctrl.key.v==528*/)//����shift+f�������� ����ʱ��ʱʹ�ú����
	{	
		low_speed=1;//�������ģʽ  [���޸�]
		target_can2_202_angle = TARGET_AG_CAN2_202_ANGLE_INIT; //�ջ���ֱ���
		target_can2_204_angle = TARGET_AG_CAN2_204_ANGLE_INIT;  //��е��צ�ջ�
		start_flag=1;
	}
	if(start_flag) 
	{
		if(angle_can2_202>20) //��10���޸� �����ж��Ƿ�ﵽ��ʼ��λ��
				return;
		else //���ﵽԤ��ֵʱ
		{
			target_can2_205_angle = TARGET_AG_CAN2_205_ANGLE_INIT; //̧���½�
			target_can2_206_angle = -TARGET_AG_CAN2_206_ANGLE_INIT; //̧���½�
			if(angle_can2_205 < 10) //���޸� �����ж��Ƿ��½����
			{
				target_can2_207_angle_6020 = TARGET_AG_CAN2_207_ANGLE_6020_INIT;
				servo_angle[0] = 1500; //�������ֵ
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0); //�̵����ߵ�ƽ��ʹ��
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
	if(angle_can2_205 < TARGET_AG_CAN2_205_ANGLE_RISING-give_angle_error_test) //���޸�
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
	if(ag_flag)
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999); 
		Ag_Event = AG_INIT;
		Air_Get.transfer_flag = 1;
		//�ѽ����ʲ�ִ��
	}
}
//--------------------------------------------------------����Դ������ȡ�����к���-------------------------------------------------//
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
//����ȡ���Ѵ���һ��״̬���ʲ����ʼ��
static void act_BTM_BigResourceIsland_Rising(void)
{
	BTM_Flag = 1;
	if( rc_ctrl.key.v==2064)	//����shift+z��������
	{
		target_can2_205_angle = TARGET_BTM_CAN2_205_ANGLE_RISING; //̧������
		target_can2_206_angle = -TARGET_BTM_CAN2_206_ANGLE_RISING;
	}
	if(angle_can2_205 < TARGET_BTM_CAN2_205_ANGLE_RISING - 10) //�����
	{
		return;
	}
	else
	{
		Btm_Event = BTM_ARM_RETURN_STATE;
		Big_Take_Mine.transfer_flag = 1;
	}
}
//�˺������ڻ�е�����Ļص����
static void act_BTM_BigResourceIsland_ArmReturn(void)
{
	if(BTM_Flag)
	{
		target_can2_207_angle_6020 = TARGET_BTM_CAN2_207_ANGLE_6020_RETURN; //����
	}
	if(fabs(angle_can2_207_6020) >  TARGET_BTM_CAN2_207_ANGLE_6020_RETURN - 1) //���޸�
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
		target_can2_202_angle = TARGET_BTM_CAN2_202_ANGLE_RETRACR;  //ǰ���ջ�
	}
	if(angle_can2_202 > TARGET_BTM_CAN2_202_ANGLE_RETRACR - 1) //���޸�
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
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,19999);  //�ر�����
		Btm_Event = BTM_STORAGE_ORE_DOWN;
		Big_Take_Mine.transfer_flag = 1;
	}
}
static void act_BTM_BigResourceIsland_Storage_Down(void)
{
	if(BTM_Flag)
	{
		target_can2_208_angle = TARGET_BTM_CAN2_208_ANGLE_DOWN;  //����½�
	}
	if(angle_can2_208 > TARGET_BTM_CAN2_208_ANGLE_DOWN - 1) //���޸�
	{
		return;
	}
	else
	{
		Btm_Event = BTM_START_RISING;
		Big_Take_Mine.transfer_flag = 1;
	}
}