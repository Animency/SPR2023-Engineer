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
static void S_type_acc_and_dec(gimbal_control_t *gimbal_move_S_type_acc_and_dec);

static void gimbal_init(gimbal_control_t *gimbal_init);

static void gimbal_feedback_update(gimbal_control_t *gimbal_move_update);

static void gimbal_2006_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,int current_target_transfer);

static void gimbal_set_mode(gimbal_control_t *gimbal_move_mode);

static void gimbal_sigang(gimbal_control_t *gimbal_sigang);

static void gimbal_tongbulun(gimbal_control_t *gimbal_mocachuandonglun);

static void gimbal_mocachuandonglun(gimbal_control_t *gimbal_mocachuandonglun);

static void gimbal_mocalun(gimbal_control_t *gimbal_mocalun);

static void gimbal_diancifa(gimbal_control_t *gimbal_diancifa);

static void gimbal_sigang_fashe(gimbal_control_t *gimbal_sigang_fashe);

static void gimbal_taisheng(gimbal_control_t *gimbal_taisheng);
	
static void gimbal_diancifa_auto(gimbal_control_t *gimbal_diancifa_auto);

static void gimbal_tongbulun_auto(gimbal_control_t *gimbal_tongbulun_auto);

static void gimbal_sigang_auto(gimbal_control_t *gimbal_sigang_auto);

static void gimbal_tongbulun_keepwork(gimbal_control_t *gimbal_tongbulun_keepwork);
	
extern	chassis_move_t chassis_move;
	
int mocalun_target=0,tongbulun_flag=0;

float S_type_acc_and_dec_gimbal_i=0,gimbal_pid_test=0,gimbal_variate=9,gimbal_cs=0,gimbal_pid_cs=10000,speed_rpm_view=0;
//云台控制所有相关数据
gimbal_control_t gimbal_control;

//发送的电机电流
static int16_t yaw_can_set_current = 0,pitch_can_set_current = 0, shoot_can_set_current = 0;
extern float flag_follow;
extern int achieve_flag;

void gimbal_task(void const *pvParameters)
{
  vTaskDelay(20);
  gimbal_init(&gimbal_control);

  while (1)
  {
		usart1_tx_dma_enable(uart1_send_data,CMD_PWM_COUNT);	
		gimbal_set_mode(&gimbal_control);
		S_type_acc_and_dec(&gimbal_control );
		gimbal_mocalun(&gimbal_control);
		gimbal_tongbulun_keepwork(&gimbal_control);
		tongbulun_flag=0;
		gimbal_feedback_update(&gimbal_control);
		CAN_cmd_gimbal(gimbal_control .horizontal_scroll_motor[0].give_current ,gimbal_control .horizontal_scroll_motor[1].give_current,gimbal_control .horizontal_scroll_motor[2].give_current,0); //gimbal_control .horizontal_scroll_motor .give_current
		CAN2_cmd_gimbal(gimbal_control .horizontal_scroll_motor[3].give_current ,gimbal_control .horizontal_scroll_motor[4].give_current,0,0); 
		gimbal_pid_test =10*sin(gimbal_variate*GIMBAL_DEC);
//		gimbal_sigang(&gimbal_control);
//		gimbal_tongbulun(&gimbal_control);//调试模式下使用这句话，遥控器模式不得使用这句
//		gimbal_taisheng(&gimbal_control);
//		HAL_UART_Transmit_DMA(&huart1,uart1_send_data,8);

    vTaskDelay(20);
  }
}	

static void S_type_acc_and_dec(gimbal_control_t *gimbal_move_S_type_acc_and_dec)   
{
	int speed_min=0,speed_max=3,speed_acc_len=500;
	float flexible=2,melo,deno,median,coefficient=1;
	melo = flexible * (S_type_acc_and_dec_gimbal_i-speed_acc_len/2) / (speed_acc_len/2);
	deno = 1.0f / (1 + expf(-melo));
	median=speed_min+(speed_max-speed_min)*deno;
//	gimbal_control.horizontal_scroll_motor.speed_set =8*sin(gimbal_variate*GIMBAL_DEC); //median*coefficient实际使用
	S_type_acc_and_dec_gimbal_i+=0.1;
	gimbal_variate +=0.1;
}

static void gimbal_init(gimbal_control_t *gimbal_init)
{
	unsigned int i=0;
	if (gimbal_init == NULL)
  {
    return;
  }
	gimbal_init->gimbal_rc_ctrl= get_remote_control_point();
	for (i=0;i<5;++i)
	{
	gimbal_init->horizontal_scroll_motor[i] .gimbal_motor_measure  = get_gimbal_motor_measure_point(i);
//  PID_init(&gimbal_init->horizontal_scroll_motor[i] .gimbal_motor_speed_pid ,0, 0, 0, 13000, 3000); //每个2006的pid初始化都不一样，需要单独初始化
	gimbal_init->horizontal_scroll_motor[i] .speed_max =NORMAL_MAX_GIMBAL_SPEED;
	gimbal_init->horizontal_scroll_motor[i] .speed_min =-NORMAL_MAX_GIMBAL_SPEED;
	}
		PID_init(&gimbal_init->horizontal_scroll_motor[0] .gimbal_motor_speed_pid ,1, 0, 0, 13000, 3000); 
		PID_init(&gimbal_init->horizontal_scroll_motor[1] .gimbal_motor_speed_pid ,1.4, 0.02, 1.4, 13000, 3000); 
	  PID_init(&gimbal_init->horizontal_scroll_motor[2] .gimbal_motor_speed_pid ,1.7, 0.12, 0.25, 13000, 3000);
	  PID_init(&gimbal_init->horizontal_scroll_motor[3] .gimbal_motor_speed_pid ,0, 0, 0, 13000, 3000); 
	  PID_init(&gimbal_init->horizontal_scroll_motor[4] .gimbal_motor_speed_pid ,0, 0, 0, 13000, 3000); 
	gimbal_feedback_update(&gimbal_control);
}

static void gimbal_set_mode(gimbal_control_t *gimbal_move_mode)
{
	if (gimbal_move_mode == NULL)
  {
    return;
  }
	
//	//摩擦轮    右摇杆—同步轮
//	 if (SWITCH_LEFT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl   ->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl  ->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
//    {
//       mocalun_target=1;
//			gimbal_tongbulun(&gimbal_control);
//			gimbal_diancifa(&gimbal_control);
//			gimbal_sigang_fashe(&gimbal_control);
//			flag_follow=1;
//    }
//		
//	//摩擦传动    右摇杆-同步轮
//    else if (SWITCH_LEFT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl ->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
//    {
//     gimbal_cs=2; 
//		 gimbal_tongbulun(&gimbal_control);		
//     gimbal_mocachuandonglun (&gimbal_control );
//     flag_follow=1;			
//    }
//		
//	//左前摇杆-电磁阀    右摇杆-同步轮
//    else if (SWITCH_LEFT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl ->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
//    {
//			gimbal_tongbulun(&gimbal_control);	
//			flag_follow=1;
//    }
//		
//	//左前滑轮—丝杠	
//		else if (SWITCH_LEFT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
//		{
//			gimbal_sigang(&gimbal_control);
//			flag_follow=1;
//		}
//		//抬升	
//		else if (SWITCH_LEFT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
//		{
//			gimbal_taisheng(&gimbal_control);
//			flag_follow=1;
//		}
//		//自动化
//		else if (SWITCH_LEFT_IS_DOWN(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_UP(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
//		{
//			/*发射预装球击蓝塔*/
//		//电磁阀控制气缸，摩擦轮在通电时就启动
//			if(chassis_move.sick_data[1].voltage==0&&achieve_flag==2)
//			{
//				vTaskDelay(20);
//			gimbal_diancifa_auto(&gimbal_control);
//				achieve_flag=3;
//			}
//			
//			/*就地取一个红球*/
//			if(achieve_flag ==3&&htim8.Instance->CCR1 ==0)
//				gimbal_tongbulun_auto(&gimbal_control);
//			
//			//取球结束标志
//			/*if()
//			    achieve_flag=4;*/
//			
//			if(chassis_move.sick_data [0].voltage<0&&achieve_flag==4)
//			{
//			//	if ()   判断炮筒是否到达指定角度，击塔角度
//				gimbal_sigang_auto(&gimbal_control);
//			}
//			
//				/*发射红球击红塔*/
//		//电磁阀控制气缸，摩擦轮在通电时就启动
//			if(chassis_move.sick_data[0].voltage==0&&achieve_flag==5)
//			{
//				vTaskDelay(20);
//			gimbal_diancifa_auto(&gimbal_control);
//				achieve_flag=6;
//			}
//			
//			/*就地取一个蓝球*/
//			if(achieve_flag ==6&&htim8.Instance->CCR1 ==0)
//				gimbal_tongbulun_auto(&gimbal_control);
//			
//			//取球结束标志
//			/*if()
//			    achieve_flag=7;*/
//			
//			/*调整俯仰角，击头顶球*/
//			if(achieve_flag==7)
//			{
//			/*	if ()   判断炮筒是否到达指定角度，击塔角度
//				achieve_flag=8;
//				*/
//				gimbal_sigang_auto(&gimbal_control);
//			}
//			
//			/*发射蓝球击头顶球*/
//			if(achieve_flag==8)
//			{
//				vTaskDelay(20);
//			gimbal_diancifa_auto(&gimbal_control);
//				achieve_flag=9;
//			}
//		}
	//云台手操
	//摩擦轮和同步轮自动工作，左前滑轮控制发射，左摇杆控制摩擦传动轮，右摇杆控制丝杠
	 if (SWITCH_LEFT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_LEFT_CHANNEL])&&SWITCH_RIGHT_IS_MID(gimbal_move_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_RIGHT_CHANNEL]))
    {
       mocalun_target=1;
			tongbulun_flag=1;
			gimbal_diancifa(&gimbal_control);
			gimbal_sigang(&gimbal_control);
			gimbal_mocachuandonglun (&gimbal_control );
			flag_follow=1;
    }
}
static void gimbal_feedback_update(gimbal_control_t *gimbal_move_update)
{
	int i=0;
	if (gimbal_move_update == NULL)
  {
    return;
  }
  for (i=0;i<3;++i)
	gimbal_move_update->horizontal_scroll_motor[i] .motor_speed_current  = MOTOR_RPM_TO_SPEED * gimbal_move_update ->horizontal_scroll_motor[i] .gimbal_motor_measure->speed_rpm  ;
}
static void gimbal_2006_control_loop(gimbal_control_t *gimbal_move_control_loop,int16_t i,int current_target_transfer)
{
	 fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
	float current_target=0;
    temp = fabs(gimbal_move_control_loop->horizontal_scroll_motor[i] .speed_set);
	current_target=current_target_transfer;//遥控器模式使用
//	current_target=gimbal_pid_cs;
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
    PID_calc(&gimbal_move_control_loop->horizontal_scroll_motor[i].gimbal_motor_speed_pid , gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_measure ->speed_rpm    , current_target );
    speed_rpm_view=gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_measure ->speed_rpm;
  //功率控制
  // chassis_power_control(chassis_move_control_loop);

  //赋值电流值
    gimbal_move_control_loop->horizontal_scroll_motor[i] .give_current  = (int16_t)(gimbal_move_control_loop->horizontal_scroll_motor[i] .gimbal_motor_speed_pid .out );   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
}

//丝杠控制
static void gimbal_sigang(gimbal_control_t *gimbal_sigang)
{
	float sigang_target=0;
	if (gimbal_sigang->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]>300)
		sigang_target=-10000;
	else if (gimbal_sigang->gimbal_rc_ctrl->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]<-300)
		sigang_target=10000;
	gimbal_2006_control_loop(&gimbal_control,0,sigang_target);
}
//自动化的丝杠控制
static void gimbal_sigang_auto(gimbal_control_t *gimbal_sigang_auto)
{
	float sigang_target=0;
		sigang_target=-10000;
	gimbal_2006_control_loop(&gimbal_control,0,sigang_target);
}
//丝杠在发射时
static void gimbal_sigang_fashe(gimbal_control_t *gimbal_sigang_fashe)
{
	float sigang_target=0;
	if (gimbal_sigang_fashe->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]>300)
		sigang_target=-10000;
	else if (gimbal_sigang_fashe->gimbal_rc_ctrl->rc .ch [PITCH_CHANNEL]<-300)
		sigang_target=10000;
	gimbal_2006_control_loop(&gimbal_control,0,sigang_target);
}

//同步轮控制
static void gimbal_tongbulun(gimbal_control_t *gimbal_tongbulun)
{
	float tongbulun_target=0;
	if (gimbal_tongbulun->gimbal_rc_ctrl ->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]>300)
		tongbulun_target=-10000;
	else if (gimbal_tongbulun->gimbal_rc_ctrl->rc .ch [GIMBAL_MODE_LEFT_CHANNEL]<-300)
		tongbulun_target=10000;
	gimbal_2006_control_loop(&gimbal_control,1,tongbulun_target);
}

//同步轮自动控制
static void gimbal_tongbulun_keepwork(gimbal_control_t *gimbal_tongbulun_keepwork)
{
	float tongbulun_target=0;
	if (tongbulun_flag==1)
		tongbulun_target=-10000;
	else
		tongbulun_target=0;
	gimbal_2006_control_loop(&gimbal_control,1,tongbulun_target);
}

//自动化的同步轮控制
static void gimbal_tongbulun_auto(gimbal_control_t *gimbal_tongbulun_auto)
{
	float tongbulun_target=0;
		tongbulun_target=-10000;
	gimbal_2006_control_loop(&gimbal_control,1,tongbulun_target);
}

//摩擦传动轮控制
static void gimbal_mocachuandonglun(gimbal_control_t *gimbal_mocachuandonglun)
{
	float mocachuandonglun_target=0;
	if (gimbal_mocachuandonglun->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]>300)
		mocachuandonglun_target=1500;
	else if (gimbal_mocachuandonglun->gimbal_rc_ctrl->rc .ch [PITCH_CHANNEL]<-300)
		mocachuandonglun_target=-1500;
	gimbal_2006_control_loop(&gimbal_control,2,mocachuandonglun_target);
}

//3508抬升控制
	static void gimbal_taisheng(gimbal_control_t *gimbal_taisheng)
{
	float taisheng_target=0;
	if (gimbal_taisheng->gimbal_rc_ctrl ->rc .ch [PITCH_CHANNEL]>300)
		taisheng_target=500;
	else if (gimbal_taisheng->gimbal_rc_ctrl->rc .ch [PITCH_CHANNEL]<-300)
		taisheng_target=-500;
	gimbal_2006_control_loop(&gimbal_control,3,taisheng_target);
	gimbal_2006_control_loop(&gimbal_control,4,taisheng_target);
}

//摩擦轮控制
static void gimbal_mocalun(gimbal_control_t *gimbal_mocalun)
{
	int mocalun_off=1000;
	int mocalun_on=1350;
	if(mocalun_target   ==0)
	{
		htim1.Instance->CCR1 = mocalun_off;
		htim1.Instance->CCR2 = mocalun_off;
//		htim1.Instance->CCR3 = mocalun_off;
	}
	if(mocalun_target   ==1)
	{
		htim1.Instance->CCR1 = mocalun_on;
		htim1.Instance->CCR2 = mocalun_on;
//		htim1.Instance->CCR3 = mocalun_on;
	}
	mocalun_target=0;
}

//电磁阀控制
static void gimbal_diancifa(gimbal_control_t *gimbal_diancifa)
{
	int diancifa_target=0;
	int diancifa_on=20000;
	int diancifa_off=0;
	if (gimbal_diancifa->gimbal_rc_ctrl ->rc .ch [GIMBAL_SIGANG_CHANNEL]>300)
		diancifa_target=1;
	else
		diancifa_target=0;
	if(diancifa_target  ==0)
	{
		htim1.Instance->CCR3 = diancifa_off;
	}
	if(diancifa_target  ==1)
	{
		htim1.Instance->CCR3 = diancifa_on;
	}
}
//自动化的电磁阀控制
static void gimbal_diancifa_auto(gimbal_control_t *gimbal_diancifa_auto)
{
	int diancifa_target=0;
	int diancifa_on=20000;
	int diancifa_off=0;
		diancifa_target=1;
	if(diancifa_target  ==0)
	{
		htim1.Instance->CCR3 = diancifa_off;
	}
	if(diancifa_target  ==1)
	{
		htim1.Instance->CCR3 = diancifa_on;
	}
}
 //电磁阀控制，伸出
static void gimbal_diancifa_shenchu(gimbal_control_t *gimbal_diancifa)
{
	int diancifa_target=0;
	int diancifa_on=20000;
	int diancifa_off=0;
	if (1)
		diancifa_target=1;
	else
		diancifa_target=0;
	if(diancifa_target  ==0)
	{
		htim1.Instance->CCR4 = diancifa_off;
	}
	if(diancifa_target  ==1)
	{
		htim1.Instance->CCR4 = diancifa_on;
	}
}

//电磁阀控制，夹取
static void gimbal_diancifa_jiaqu(gimbal_control_t *gimbal_diancifa)
{
	int diancifa_target=0;
	int diancifa_on=20000;
	int diancifa_off=0;
	if (1)
		diancifa_target=1;
	else
		diancifa_target=0;
	if(diancifa_target  ==0)
	{
		htim8.Instance->CCR1 = diancifa_off;
	}
	if(diancifa_target  ==1)
	{
		htim8.Instance->CCR1 = diancifa_on;
	}
}

//舵机，水平角维持+旋转
static void gimbal_duoji(gimbal_control_t *gimbal_duoji)
{
	int duoji_lf=1300;
	int duoji_ri=1300;
	int duoji_flag=0;//0水平维持 1反转
	if(duoji_flag==0)
	{
		htim8.Instance->CCR2 = duoji_lf;
		htim8.Instance->CCR3 = duoji_ri;
	}
	if(duoji_flag==1)
	{
		
	}
}
