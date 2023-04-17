/**
 * @file chassis_task.c/h
 * @author 何清华，周鸣阳
 * @brief 底盘控制任务线程
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
 /*
 mode_flag=0，则表示击塔阶段
 mode_flag=1，则表示击头顶球阶段
 mode_flag=2，则表示建塔阶段
  左挡位  右挡位         模式         左前滑轮         左摇杆         右摇杆
   UP     UP    
   MID    MID           底盘控制        无             自旋          全向移动
   DOWN   UP            自动            无              无             无
   DOWN   MID           底盘控制        无             自旋          全向移动
	 DOWN   DOWN          在击塔、击头顶球、建塔三个阶段切换
*/
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "chassis_behaviour.h"
#include "detect_task.h"
#include "INS_task.h"
#include "math.h"
#include "tim.h"
#include "gimbal_task.h"
#include "bsp_adc.h"
#include "board_talk.h"
float variate=9,vy_set_pid_test,view=0,view_melo=0,view_deno=0,S_type_acc_and_dec_i=-8,pid_current=400,pid_v=0.4,pid_angle=90,distance_target_view=0;
float INS_angle_init=0,flag_follow=1,CHASSIS_X_CHANNEL_last=0,CHASSIS_Y_CHANNEL_last=0,INS_angle_chassis=0,INS_angle_init_view=0,INS_angle_view=0,distance_view=0,distance_target=0.935;
int achieve_flag=0,mode_flag=0;
extern float pid_current_actual;
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

	
	float vx_set_zhuanghuan=0,vy_set_zhuanghuan=0,wz_set_zhuanghuan=0;
/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
 * @brief
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

static void chassis_follow_control_loop(chassis_move_t *chassis_follow_move_control_loop);

static void S_type_acc_and_dec_chassis(chassis_move_t  *chassis_move_test_S_type_acc_and_dec_chassis,float start_flag,float end_flag);   

static void chassis_auto_control_loop(chassis_move_t *chassis_auto_control_loop,float end_flag,float sick_num);
float get_average_1(void);

//底盘运动数据
chassis_move_t chassis_move;

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //底盘初始化
  chassis_init(&chassis_move);
  //判断底盘电机是否都在线
//  while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
//  {
//    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//  }

  while (1)
  {
		flag_follow=1;
    //设置底盘控制模式
    chassis_set_mode(&chassis_move);
		
    //底盘数据更新
    chassis_feedback_update(&chassis_move);
		
    //底盘控制量设置
    chassis_set_contorl(&chassis_move);
		distance_view=100*board_talk_recive_data[11];
		distance_target_view=100*distance_target ;
//	  S_type_acc_and_dec_chassis(&chassis_move );
		//底盘跟随时PID计算
		if (flag_follow ==0)
		 chassis_follow_control_loop(&chassis_move);

CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current); 
    vy_set_pid_test = 10.0*sin(variate*DEC);
		//系统延时
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
		
  }
}

/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
  if (chassis_move_init == NULL)
  {
    return;
  }

  // const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  // const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;

  //底盘开机状态
  chassis_move_init->chassis_mode = CHASSIS_VECTOR_DEBUG;
	
  //获取遥控器指针
  chassis_move_init->chassis_RC = get_remote_control_point();
  //获取陀螺仪姿态角指针
  chassis_move_init->chassis_follow_angle  = get_INS_angle_point();
	chassis_move_init->chassis_follow_gyro  = get_gyro_data_point();
	INS_angle_init=	chassis_move_init->chassis_INS_angle[0] ;
	INS_angle_init_view =100*INS_angle_init ;
	
//  //获取云台电机数据指针
//  chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
//  chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
  //获取底盘电机数据指针，初始化PID
  for (i = 0; i < 4; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move_init->motor_speed_pid[i], 27300, 1900, 7000, 6000, 2000);//13000  3000
		PID_init(&chassis_move_init->motor_speed_auto_pid[i], 27300, 1900, 7000, 2000, 1000);
  }
  //初始化角度PID(旋转跟随云台)
	PID_init(&chassis_move_init->chassis_angle_pid, 0.02, 0, 0, 10000, 3000);
	
  PID_init(&chassis_move_init->chassis_auto_right_pid, 45, 0.005, 0, 2000, 1000);
	PID_init(&chassis_move_init->chassis_auto_left_pid, 45, 0.012, 0, 2000, 1000);
	//p=40,i=0,d=0在目标点右侧5到30厘米效果好
	//p=45,i=0.005,d=0在目标点右侧0到15厘米效果好
	//p=45,i=0.012,d=0在目标点左侧0到15厘米效果好

  // //用一阶滤波代替斜波函数生成
  // first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  // first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

  //最大 最小速度
  chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	
  //更新一下数据
  chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	float vx_set = 0.0f, vy_set = 0.0f , angle_set = 0.0f;
	float redball_1_m1=0;
  if (chassis_move_mode == NULL)
  {
    return;
  }
	chassis_move_mode->chassis_mode = CHASSIS_VECTOR_DEBUG;
	if (SWITCH_LEFT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
	chassis_move_mode->chassis_mode = CHASSIS_RC_CONTROL;
//	else if (SWITCH_LEFT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
//	{
//		chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_mode);
//		chassis_move_mode->vx_set = vx_set ;
//		chassis_move_mode->vy_set = vy_set ;
//		chassis_move_mode->wz_set = angle_set ;
//		chassis_control_loop(&chassis_move);
//		flag_follow =1;
//		CHASSIS_X_CHANNEL_last=chassis_move_mode->chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
//		CHASSIS_Y_CHANNEL_last=chassis_move_mode->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
//	}
		else if (SWITCH_LEFT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
             chassis_move_mode->chassis_mode = CHASSIS_FOLLOW;
		else if (SWITCH_LEFT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
		{
//			if (abs(board_talk_recive_data[11]-distance_target)>0.001)
			chassis_auto_control_loop(chassis_move_mode, distance_target , 11);
			
		}
//	else if (SWITCH_LEFT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
//	{
//		if(flag_follow ==1)
//		{
//		chassis_move_mode->vx_set = 0 ;
//		chassis_move_mode->vy_set = 0 ;
//		chassis_move_mode->wz_set = 0 ;
//		INS_angle_init=	chassis_move_mode->chassis_INS_angle[0] ;
//		}
//    flag_follow =0;		
//	}
//	else if (SWITCH_LEFT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
//	{
//		int i=0;
//		//先来一个矫正姿态
//		
//		/*自动矫正函数*/
//		
//		
////		chassis_move_mode->vx_set = 0 ;
////		chassis_move_mode->vy_set = 0 ;
////		chassis_move_mode->wz_set = 0 ;
////		//水平移动
////		if (chassis_move_mode->sick_data[0].voltage<redball_1_m1&&chassis_move_mode->sick_data[0].voltage>redball_1_m2 )
////		{
////			chassis_move_mode->vx_set = CHASSIS_AUTO_VX ;
////		chassis_move_mode->vy_set = 0 ;
////		chassis_move_mode->wz_set = 0 ;
////		}
////		//水平移动缓停
////		if(chassis_move_mode->sick_data[0].voltage<redball_1_m2&&chassis_move_mode->sick_data[0].voltage>redball_1_m3)
////		{
////			chassis_move_mode->vx_set = CHASSIS_AUTO_VX ;
////		chassis_move_mode->vy_set = 0 ;
////		chassis_move_mode->wz_set = 0 ;
//////			S_type_acc_and_dec_chassis(chassis_move_mode);
////			achieve_flag=1;
////		}
////		//竖直移动
////		if (chassis_move_mode->sick_data[1].voltage>redball_1_m4&&achieve_flag==1)
////		{
////			chassis_move_mode->vx_set = 0 ;
////		chassis_move_mode->vy_set = CHASSIS_AUTO_VY ;
////		chassis_move_mode->wz_set = 0 ;
////		}
////		//竖直移动缓停
////		if (chassis_move_mode->sick_data[1].voltage>redball_1_m5&&chassis_move_mode->sick_data[1].voltage<redball_1_m4&&achieve_flag==1)
////		{
////			chassis_move_mode->vx_set = 0 ;
////		chassis_move_mode->vy_set = CHASSIS_AUTO_VY ;
////		chassis_move_mode->wz_set = 0 ;
//////			S_type_acc_and_dec_chassis(chassis_move_mode);
////			achieve_flag=2;
////		}
////		//自动矫正
////		if (achieve_flag==2)
////		{
////			/*自动矫正函数*/
////			
////			achieve_flag=0;
////		}		
//    
//		/*移动至击蓝塔发射点*/
//		//水平移动
//		if(chassis_move_mode->sick_data[0].voltage>redball_1_m1&&achieve_flag==0)
//		chassis_auto_control_loop(chassis_move_mode, redball_1_m1, 0);
//		//水平移动结束标志
//		if(chassis_move_mode->sick_data[0].voltage==redball_1_m1&&achieve_flag==0)
//		{
//			achieve_flag=1;
//			/*自动矫正函数*/
//		}
//		//竖直移动
//		if(chassis_move_mode->sick_data[1].voltage>redball_1_m1&&achieve_flag==1)
//		chassis_auto_control_loop(chassis_move_mode, redball_1_m1, 1);
//		//竖直移动结束标志
//		if(chassis_move_mode->sick_data[1].voltage==redball_1_m1&&achieve_flag==1)
//		{
//			achieve_flag=2;
//			/*自动矫正函数*/
//		}
//		
//		/*发射预装球击蓝塔*/
//		//电磁阀控制气缸，摩擦轮在通电时就启动,见gimbal_task.c

//		/*就地取一个红球*/
//		//见gimbal_task.c
//		
//		/*移动至击红塔发射点*/
//	  //水平移动
//		if(chassis_move_mode->sick_data[0].voltage<redball_1_m1&&achieve_flag==4)
//		{
//		chassis_auto_control_loop(chassis_move_mode, redball_1_m1, 0);
//		}
//		//水平移动结束标志
//		if(chassis_move_mode->sick_data[0].voltage==redball_1_m1&&achieve_flag==4)
//		{
//			achieve_flag=5;	
//			/*自动矫正函数*/
//		}			
//		
//		/*击右边红塔*/
//		//见gimbal_task.c
//		
//		/*取一个蓝球*/
//		//见gimbal_task.c
//		
//		/*击头顶球*/
//		//见gimbal_task.c
//			
////		chassis_control_loop(&chassis_move);
//		flag_follow =1;
//	}
  //换挡
	else if (SWITCH_LEFT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && chassis_move_mode->chassis_RC->rc.ch[CHASSIS_HUALUN_CHANNEL]<-500)
	{
		if (SWITCH_RIGHT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
			mode_flag=0;
		else if (SWITCH_RIGHT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
			mode_flag=1;
		else if (SWITCH_RIGHT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
			mode_flag=2;
		flag_follow=1;
	}	
  /*击塔*/
	//从启动到第一个击塔发射点
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
  if (chassis_move_update == NULL)
  {
    return;
  }

  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    //更新电机速度，加速度是速度的PID微分
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		chassis_move_update->motor_chassis[0].angle = INS_data.angle_yaw;
		chassis_move_update->motor_chassis[0].gyro =INS_data.wx;
  }
	//全向轮速度分解
  //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
  chassis_move_update->vx = (chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;
  // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
  //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  // first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
  //一阶低通滤波代替斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
  // stop command, need not slow change, set zero derectly
  //停止信号，不需要缓慢加速，直接减速到零
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

  if (chassis_move_control == NULL)
  {
    return;
  }

  float vx_set = 0.0f, vy_set = 0.0f , angle_set = 0.0f;
	//vy_set =vy_set_pid_test ;      //DEBUG模式下用这句话
  //获取三个控制设置值
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_DEBUG)
	{ //Debug模式
		chassis_move_control->wz_set = angle_set;
     chassis_move_control->vx_set = float_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = float_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
		chassis_control_loop(&chassis_move);
		flag_follow=1;
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_RC_CONTROL)
	{
		chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		chassis_move_control->vx_set = vx_set ;
		chassis_move_control->vy_set = vy_set ;
		chassis_move_control->wz_set = angle_set ;
		chassis_control_loop(&chassis_move);
		flag_follow =1;
		CHASSIS_X_CHANNEL_last=chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
		CHASSIS_Y_CHANNEL_last=chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_FOLLOW)
	{
		if(flag_follow ==1)
		{
		chassis_move_control->vx_set = 0 ;
		chassis_move_control->vy_set = 0 ;
		chassis_move_control->wz_set = 0 ;
//		INS_angle_init=	chassis_move_control->chassis_INS_angle[0] ;
		}
    flag_follow =0;		
	}

  //跟随云台模式
//  if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
//  {
//    float sin_yaw = 0.0f, cos_yaw = 0.0f;
//    //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
//    sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//    cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//    chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
//    chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
//    //设置控制相对云台角度
//    chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
//    //计算旋转PID角速度
//    chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
//    //速度限幅
//    chassis_move_control->vx_set = float_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//    chassis_move_control->vy_set = float_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
//  }
//  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
//  {
//    //“angle_set” 是旋转速度控制
//    chassis_move_control->wz_set = angle_set;
//    chassis_move_control->vx_set = float_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//    chassis_move_control->vy_set = float_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
//  }
 
}

/**
 * @brief          四个全向轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个全向轮速度
 * @retval         none
 */
static void chassis_vector_to_omnidirectional_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	/*
	     0     1     2     3
	   左前  右前  左后  右后
	*/
  wheel_speed[0] = 1.414*vx_set/4 - 1.414*vy_set/4 + 0.41 * wz_set/4;
  wheel_speed[1] = 1.414*vx_set/4 + 1.414*vy_set/4 + 0.41 * wz_set/4;
  wheel_speed[2] = -1.414*vx_set/4 - 1.414*vy_set/4 + 0.41 * wz_set/4;
  wheel_speed[3] = -1.414*vx_set/4 + 1.414*vy_set/4 + 0.41 * wz_set/4;
	view=wheel_speed[0];
//	S_wheel_speed[0]=wheel_speed[0];
//	S_wheel_speed[1]=wheel_speed[1];
//	S_wheel_speed[2]=wheel_speed[2];
//	S_wheel_speed[3]=wheel_speed[3];
}
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	fp32 wheel_out[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

	//全向轮运动分解
  chassis_vector_to_omnidirectional_wheel_speed(chassis_move_control_loop->vx_set,
                                        chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
  // calculate the max speed in four wheels, limit the max speed
  //计算轮子控制最大速度，并限制其最大速度
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];//chassis_move_control_loop->motor_chassis [i].S_speed_set
		if (chassis_move_control_loop->chassis_mode ==CHASSIS_STATIC )
			chassis_move_control_loop->motor_chassis[i].speed_set = 0;
    temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
  }

//  // calculate pid
//  //计算pid
//  for (i = 0; i < 4; i++)
//  {
//   PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed  , chassis_move_control_loop->motor_chassis[i].speed_set);
//  }

//  //赋值电流值
//  for (i = 0; i < 4; i++)
//  {
//    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
//  }
	
	 // calculate pid
  //计算pid
  for (i = 0; i < 4; i++)
  {
   PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed  , chassis_move_control_loop->motor_chassis[i].speed_set);
		wheel_out[i]=chassis_move_control_loop->motor_speed_pid[i].out;
		
//		if (chassis_move_control_loop->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]!=0)
//			INS_angle_chassis =chassis_move_control_loop->motor_chassis[0].angle;
//		if (chassis_move_control_loop->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]==0)
//		{
//			if (abs(chassis_move_control_loop->motor_chassis[0].angle-INS_angle_init)>(0.2/180*3.14159))
//			{
//			chassis_move_control_loop->motor_chassis[i].speed_set=-chassis_follow_PID_calc(&chassis_move_control_loop->chassis_angle_pid  ,chassis_move_control_loop->motor_chassis[0].angle  ,INS_angle_chassis ,chassis_move_control_loop->motor_chassis[0].gyro );
//		PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed  , chassis_move_control_loop->motor_chassis[i].speed_set);
//			wheel_out[i]+=chassis_move_control_loop->motor_speed_pid[i].out;
//			}
//		}
  }

  //赋值电流值
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_out[i]);   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
  }
}

static void chassis_follow_control_loop(chassis_move_t *chassis_follow_move_control_loop)
{
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

	//全向轮运动分解
  chassis_vector_to_omnidirectional_wheel_speed(chassis_follow_move_control_loop->vx_set,
                                        chassis_follow_move_control_loop->vy_set, chassis_follow_move_control_loop->wz_set, wheel_speed);
  // calculate the max speed in four wheels, limit the max speed
  //计算轮子控制最大速度，并限制其最大速度
  for (i = 0; i < 4; i++)
  {
    chassis_follow_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];//chassis_move_control_loop->motor_chassis [i].S_speed_set
		if (chassis_follow_move_control_loop->chassis_mode ==CHASSIS_STATIC )
			chassis_follow_move_control_loop->motor_chassis[i].speed_set = 0;
    temp = fabs(chassis_follow_move_control_loop->motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_follow_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
  }

  // calculate pid
  //计算pid
//	if (abs(chassis_follow_move_control_loop->motor_chassis[0].angle-INS_angle_init)>(0.001*3.14159/180))
//	{
		INS_angle_view =100*chassis_follow_move_control_loop->motor_chassis[0].angle;
//  for (i = 0; i < 4; i++)
//  {
//		chassis_follow_move_control_loop->motor_chassis[i].speed_set=-chassis_follow_PID_calc(&chassis_follow_move_control_loop->chassis_angle_pid  ,chassis_follow_move_control_loop->motor_chassis[0].angle  ,INS_angle_init ,chassis_follow_move_control_loop->motor_chassis[0].gyro );
//		PID_calc(&chassis_follow_move_control_loop->motor_speed_pid[i], chassis_follow_move_control_loop->motor_chassis[i].speed  , chassis_follow_move_control_loop->motor_chassis[i].speed_set);
//  }
	chassis_follow_move_control_loop->motor_chassis[0].speed_set=-PID_calc(&chassis_follow_move_control_loop->chassis_angle_pid  ,board_talk_recive_data[8]  ,0 );
	for (i = 0; i < 4; i++)
		PID_calc(&chassis_follow_move_control_loop->motor_speed_pid[i], chassis_follow_move_control_loop->motor_chassis[i].speed  , chassis_follow_move_control_loop->motor_chassis[0].speed_set);
//}

  //赋值电流值
  for (i = 0; i < 4; i++)
  {
    chassis_follow_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_follow_move_control_loop->motor_speed_pid[i].out);   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
  }
}

static void chassis_auto_control_loop(chassis_move_t *chassis_auto_control_loop,float end_flag,float sick_num)
{
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;
	int t=sick_num;

	//全向轮运动分解
  chassis_vector_to_omnidirectional_wheel_speed(chassis_auto_control_loop->vx_set,
                                        chassis_auto_control_loop->vy_set, chassis_auto_control_loop->wz_set, wheel_speed);
  // calculate the max speed in four wheels, limit the max speed
  //计算轮子控制最大速度，并限制其最大速度
  for (i = 0; i < 4; i++)
  {
    chassis_auto_control_loop->motor_chassis[i].speed_set = wheel_speed[i];//chassis_move_control_loop->motor_chassis [i].S_speed_set
		if (chassis_auto_control_loop->chassis_mode ==CHASSIS_STATIC )
			chassis_auto_control_loop->motor_chassis[i].speed_set = 0;
    temp = fabs(chassis_auto_control_loop->motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_auto_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
  }

  // calculate pid
  //计算pid
//	if (abs(chassis_follow_move_control_loop->motor_chassis[0].angle-INS_angle_init)>(0.2/180*3.14159))
//	{
	if ((board_talk_recive_data[t]-0.002 )>end_flag)
	chassis_auto_control_loop->motor_chassis[0].speed_set=PID_calc(&chassis_auto_control_loop->chassis_auto_right_pid  ,board_talk_recive_data[t] ,end_flag );
	else if((board_talk_recive_data[t]+0.002) <end_flag)
	chassis_auto_control_loop->motor_chassis[0].speed_set=PID_calc(&chassis_auto_control_loop->chassis_auto_left_pid  ,board_talk_recive_data[t] ,end_flag );
		PID_calc(&chassis_auto_control_loop->motor_speed_pid[0], chassis_auto_control_loop->motor_chassis[0].speed  ,-chassis_auto_control_loop->motor_chassis[0].speed_set);
    PID_calc(&chassis_auto_control_loop->motor_speed_pid[1], chassis_auto_control_loop->motor_chassis[1].speed  ,-chassis_auto_control_loop->motor_chassis[0].speed_set);
	  PID_calc(&chassis_auto_control_loop->motor_speed_pid[2], chassis_auto_control_loop->motor_chassis[2].speed  ,chassis_auto_control_loop->motor_chassis[0].speed_set);
	  PID_calc(&chassis_auto_control_loop->motor_speed_pid[3], chassis_auto_control_loop->motor_chassis[3].speed  ,chassis_auto_control_loop->motor_chassis[0].speed_set);

  //赋值电流值
  for (i = 0; i < 4; i++)
  {
    chassis_auto_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_auto_control_loop->motor_speed_pid[i].out);   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
  }
}
static void S_type_acc_and_dec_chassis(chassis_move_t  *chassis_move_test_S_type_acc_and_dec_chassis,float start_flag,float end_flag)   
{
	int i=0;
	float speed_acc_len=0;
	float flexible=10,melo,deno,median,coefficient=1,last_speed=0;
	fp32 S_wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	speed_acc_len=abs(start_flag-end_flag);
	S_type_acc_and_dec_i=start_flag;
			chassis_vector_to_omnidirectional_wheel_speed(chassis_move_test_S_type_acc_and_dec_chassis->vx_set,
                                        chassis_move_test_S_type_acc_and_dec_chassis->vy_set, chassis_move_test_S_type_acc_and_dec_chassis->wz_set, S_wheel_speed);
	melo = flexible * (S_type_acc_and_dec_i-speed_acc_len/2) / (speed_acc_len/2);
	deno = 1.0f / (1 + expf(-melo));
	median=last_speed +(S_wheel_speed[i] -last_speed)*deno;
//	view_melo=melo;
//	view_deno=deno;
//	view=median ;
	last_speed=S_wheel_speed[i] ;
	chassis_move_test_S_type_acc_and_dec_chassis ->motor_chassis [i].S_speed_set=median*coefficient;
//	pid_current =500*chassis_move_test_S_type_acc_and_dec_chassis ->vy_set;
		S_type_acc_and_dec_i+=0.8;
}
float get_average_1(void)
{
	float sum;
	for (int i=0;i<10;++i)
	sum+=board_talk_recive_data[11];
	return (sum/10.0f);
}

