/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 0
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 1

//in some mode, can use remote control to control rotation speed
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//左前滑轮的遥控器通道号码
#define CHASSIS_HUALUN_CHANNEL 4

//the channel of choosing chassis mode,
//选择底盘状态 开关通道号
#define CHASSIS_MODE_LEFT_CHANNEL 1
#define CHASSIS_MODE_RIGHT_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//摇杆死区
#define CHASSIS_RC_DEADLINE 15

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//press the key, chassis will swing
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508 rmp change to chassis speed,
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 10.0f
//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 10.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//when chassis is not set to move, swing max angle
//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 27300.0f
#define M3505_MOTOR_SPEED_PID_KI 1900.0f
#define M3505_MOTOR_SPEED_PID_KD 7000.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 3000.0f
//底盘电机调试后 前轮速度环PID
#define SPEED_P_FRONT 27000
#define SPEED_I_FRONT 0
#define SPEED_D_FRONT 0
//底盘电机调试后 后轮速度环PID
#define SPEED_P_BEHIND 20000
#define SPEED_I_BEHIND 0
#define SPEED_D_BEHIND 0
//底盘电机角度环PID
#define M3505_MOTOR_ANGLE_PID_KP 4.0f
#define M3505_MOTOR_ANGLE_PID_KI 0.1f
#define M3505_MOTOR_ANGLE_PID_KD 30.0f
#define M3505_MOTOR_ANGLE_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_ANGLE_PID_MAX_IOUT 3000.0f
//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//自动化速度设定
#define CHASSIS_AUTO_VX 0.0f
#define CHASSIS_AUTO_VY 0.0f
#define CHASSIS_AUTO_WZ 0.0f

//键盘控制旋转速度值
#define CHASSIS_KEYBOARD_REVOLVE_SPEED 3.0f
#ifndef PI
#define PI 3.1415926535898
#endif

#define DEC (PI/180)
typedef enum
{
//  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘会跟随云台相对角度
//  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘有旋转速度控制
	CHASSIS_RC_CONTROL,                   //遥控器控制
  CHASSIS_VECTOR_DEBUG,                 //Debug模式
	CHASSIS_STATIC,                        //底盘静止但有力模式
	CHASSIS_FOLLOW,
	CHASSIS_KEYBOARD_CONTROL               //底盘键盘控制
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
	float S_speed_set;
	float angle;
	float gyro;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float voltage;
} sick_data_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const float *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
  chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据
	sick_data_t sick_data[3];                  //sick的数据
  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid
	pid_type_def chassis_straighten_pid;              //chassis_straighten_pid.底盘走直线角度pid
	
  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值

  float vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
  float vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
  float wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
  float chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
  float chassis_relative_angle_set;  //the set relative angle.设置相对云台控制角度
  float chassis_yaw_set;     	

  float vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  float vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  float vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  float vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
  float chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
  float chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
  float chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度
  const float *chassis_follow_angle;
	const float *chassis_follow_gyro;

} chassis_move_t;

void chassis_task(void const *pvParameters);
#endif
