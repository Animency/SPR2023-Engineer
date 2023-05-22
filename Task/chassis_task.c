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
#include "detect_task.h"
#include "usart.h"
#include "Engineer_behaviour.h"
float INS_angle_init=0;
int zhuanwan_flag=0;
float speed_test = 0;
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
//#define SPEED_P 1000
//#define SPEED_I 500
//#define SPEED_D 100
pid_type_def speed_pid;
int speed=0;
float speed_sj=0;
float speed_set=300;
//因云台任务过重，故调整到地盘任务中接收数据
uint8_t gimbal_sensor_data;
extern int low_speed;
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

/**
 * @brief          更新由uart1传来的云台传感器数值
 * @param[out]     gimbal_sensor_data_resolve:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_sensor_data_update(gimbal_control_t *gimbal_sensor_data_resolve);
//底盘运动数据
chassis_move_t chassis_move;
//云台数据
extern gimbal_control_t gimbal_control;
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
    //设置底盘控制模式
    chassis_set_mode(&chassis_move);
		
    //底盘数据更新
    chassis_feedback_update(&chassis_move);
		
    //底盘控制量设置，底盘跟随时PID计算包含在内
    chassis_set_contorl(&chassis_move);
		
		//云台传感器数据更新
		gimbal_sensor_data_update(&gimbal_control);
      //当遥控器掉线的时候，发送给底盘电机零电流.
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_chassis(0,0,0,0);
      }
      else
      {
        //发送控制电流
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			}
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

//   const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
//  const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;
	
  //底盘开机状态
  chassis_move_init->chassis_mode = CHASSIS_VECTOR_DEBUG;
	
  //获取遥控器指针
  chassis_move_init->chassis_RC = get_remote_control_point();
  //获取陀螺仪姿态角指针
  chassis_move_init->chassis_follow_angle  = get_INS_angle_point();
	chassis_move_init->chassis_follow_gyro  = get_gyro_data_point();
	//记录底盘开机角度
	INS_angle_init=	INS_data.angle_yaw;
	
  //获取底盘电机数据指针，初始化PID
  for (i = 0; i < 2; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		//修修改前为27300，1900，7000，6500，2000
    PID_init(&chassis_move_init->motor_speed_pid[i], SPEED_P_FRONT, SPEED_I_FRONT, SPEED_D_FRONT, 13000, 3000);//13000  3000
  }
	for (i = 2; i < 4; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		//修修改前为27300，1900，7000，6500，2000
    PID_init(&chassis_move_init->motor_speed_pid[i], SPEED_P_BEHIND, SPEED_I_BEHIND, SPEED_D_BEHIND, 13000, 3000);
  }
  //初始化角度PID(旋转跟随云台)
 // PID_init(&chassis_move_init->chassis_angle_pid, 0, 0, 0, 10000, 3000);//0.015
	 //用一阶滤波代替斜波函数生成
  //first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  //first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	//初始化机器人走直PID
	PID_init(&chassis_move_init->chassis_straighten_pid, 0.7, 0, 0, 2.0, 0);
	
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
  if (chassis_move_mode == NULL)
  {
    return;
  }
	chassis_move_mode->chassis_mode = CHASSIS_VECTOR_DEBUG;

	if (SWITCH_LEFT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
	{
		chassis_move_mode->chassis_mode = CHASSIS_RC_CONTROL;
	}
	//左下右下为键盘控制，与状态机启动相同，状态机相应函数卸载Engineer_behaviour中
	else if (SWITCH_LEFT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]) && SWITCH_RIGHT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_RIGHT_CHANNEL]))
	{
		chassis_move_mode->chassis_mode = CHASSIS_KEYBOARD_CONTROL;
	}
	
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
  }
	//麦轮速度分解
  //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
  chassis_move_update->vx = (chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move_update->vy = (chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
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
  vy_set_channel = -vy_channel * CHASSIS_VY_RC_SEN;

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
  //获取三个控制设置值
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_DEBUG)
	{ //Debug模式
		chassis_move_control->wz_set = angle_set;
     chassis_move_control->vx_set = float_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = float_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
		chassis_control_loop(&chassis_move);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_RC_CONTROL)
	{
		chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		chassis_move_control->vx_set = vx_set ;
		chassis_move_control->vy_set = vy_set ;
		chassis_move_control->wz_set = angle_set ;
		/*借助C板陀螺仪控制机器走直*/
		if (angle_set !=0)
			zhuanwan_flag=1;
		if (zhuanwan_flag==1&&angle_set==0)
		{
			INS_angle_init =INS_data .angle_yaw;
			zhuanwan_flag=0;
		}
		if (angle_set==0)
			chassis_move_control->wz_set=PID_calc(&chassis_move_control->chassis_straighten_pid  , INS_data .angle_yaw   , INS_angle_init );
		speed_sj = INS_data .angle_yaw;
		
		chassis_control_loop(&chassis_move);
	}
	else if(chassis_move_control->chassis_mode == CHASSIS_KEYBOARD_CONTROL)
	{
		chassis_keyboard_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		chassis_move_control->vx_set = vx_set ;
		chassis_move_control->vy_set = vy_set ;
		chassis_move_control->wz_set = angle_set ;
		/*借助C板陀螺仪控制机器走直*/
		if (angle_set !=0)
			zhuanwan_flag=1;
		if (zhuanwan_flag==1&&angle_set==0)
		{
			INS_angle_init =INS_data .angle_yaw;
			zhuanwan_flag=0;
		}
		if (angle_set==0)
			chassis_move_control->wz_set=-PID_calc(&chassis_move_control->chassis_straighten_pid  , INS_data .angle_yaw   , INS_angle_init );
		
		chassis_control_loop(&chassis_move);
	}
}

/**
 * @brief          四个麦克纳姆速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个全向轮速度
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	////////////////电调ID/////////////////
	/////////1********前*******2///////////
	/////////*                 *///////////
	/////////*        y        *///////////
	/////////左       ↑→x     右//////////
	/////////*                 *///////////
	/////////*                 *///////////
	/////////4********后********3//////////
	///////////////////////////////////////
	wheel_speed[0] = -vy_set + vx_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = vy_set + vx_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = vy_set - vx_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vy_set - vx_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
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

	//麦轮运动分解
  chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
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

  //计算pid
  for (i = 0; i < 4; i++)
  {
   PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed  , chassis_move_control_loop->motor_chassis[i].speed_set);
	//PID_calc(&chassis_move_control_loop->chassis_straighten_pid, chassis_move_control_loop->motor_chassis[i].speed  , chassis_move_control_loop->motor_chassis[i].speed_set);
		wheel_out[i]=chassis_move_control_loop->motor_speed_pid[i].out;
  }
  //赋值电流值
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_out[i]);   //改动前(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out)
  }
}
//云台传感器解包
static void gimbal_sensor_data_update(gimbal_control_t *gimbal_sensor_data_resolve)
{
	//接受来自外部单片机的数据
	HAL_UART_Receive(&huart1,&gimbal_sensor_data,sizeof(uint8_t),0);
	gimbal_sensor_data_resolve->ore_flag.air_pump_flag_left 	= ((gimbal_sensor_data & 0x08) >> 3);
	gimbal_sensor_data_resolve->ore_flag.air_pump_flag_right 	= ((gimbal_sensor_data & 0x04) >> 2);
	gimbal_sensor_data_resolve->ore_flag.laser_flag_left 			= ((gimbal_sensor_data & 0x02) >> 1);
	gimbal_sensor_data_resolve->ore_flag.laser_flag_right 		= ((gimbal_sensor_data & 0x01));
}