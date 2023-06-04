/**
 * @file chassis_behaviour.c/h
 * @author 何清华
 * @brief 根据遥控器的值，决定底盘行为。
 * @version 0.1
 * @date 2022-03-08
 * @verbatim
  ==============================================================================
    如果要添加一个新的行为模式
    1.首先,在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
    erum
    {
        ...
        ...
        CHASSIS_XXX_XXX, // 新添加的
    }chassis_behaviour_e,

    2. 实现一个新的函数 chassis_xxx_xxx_control(float *vx, float *vy, float *wz, chassis_move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
// #include "arm_math.h"
#include "math.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
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
extern float vx_test;
extern float vy_test;
float vx_set_use,vy_set_use;
/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_open_set_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_keyboard_control_normal(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

static void chassis_keyboard_control_low(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_keyboard_control_low);
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上 
 * @brief					 此为键盘控制低速模式
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
 static void chassis_keyboard_control_heigh(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_keyboard_control_heigh);
	 /**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上 
 * @brief					 此为键盘控制高速模式
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
// highlight, the variable chassis behaviour mode
//留意，这个底盘行为模式变量
//chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
static void chassis_Debug_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control_normal(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
extern int low_speed;
/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }

//  //遥控器设置模式
//  if (SWITCH_LEFT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]))
//  {
//    //不跟随运动
//    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
//  }
//  else if (SWITCH_LEFT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]))
//  {
//    //停止运动
//    chassis_behaviour_mode = CHASSIS_NO_MOVE;
//  }
//  else if (SWITCH_LEFT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]))
//  {
//    //跟随云台运动
//    chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
//  }

//  //当云台在某些模式下，像初始化， 底盘不动
//  if (gimbal_cmd_to_chassis_stop())
//  {
//    chassis_behaviour_mode = CHASSIS_NO_MOVE;
//  }

  //添加自己的逻辑判断进入新模式

  //根据行为模式选择一个底盘控制模式
//  if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_OPEN)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
//  }
    
		
}
/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_keyboard_to_vector,  包括底盘所有信息.
 * @retval         none
 */
void chassis_keyboard_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_keyboard_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_keyboard_to_vector == NULL)
  {
    return;
  }
	chassis_keyboard_control_normal(vx_set,vy_set,angle_set,chassis_move_keyboard_to_vector);
}
/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */
void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

//  if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
//  {
//    chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
//  {
//    chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
//  {
//    chassis_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
//  {
//    chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_OPEN)
//  {
//    chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
  chassis_remote_control_normal(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_can_set = 0.0f;
  *vy_can_set = 0.0f;
  *wz_can_set = 0.0f;
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_set = 0.0f;
  *vy_set = 0.0f;
  *wz_set = 0.0f;
}

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
  *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_open_set_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;
}
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_remote_control_normal(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
//	if(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] > 100 )
//	{
//		*vx_set = CHASSIS_STATIC_VEL;
//	}
//	else if(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] < -100 )
//	{
//		*vx_set = -CHASSIS_STATIC_VEL;
//	}
//	else
//	{
//		*vx_set = 0;
//	}
//	if(-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] > 100 )
//	{
//		*vy_set = CHASSIS_STATIC_VEL;
//	}
//	else if(-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] < -100 )
//	{
//		*vy_set = -CHASSIS_STATIC_VEL;
//	}
//	else
//	{
//		*vy_set = 0;
//	}
	
  *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL;
  *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL;//-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL
  *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL;
	
	rc_deadband_limit(*vx_set,*vx_set,0.15);
	rc_deadband_limit(*vy_set,*vy_set,0.15);
	rc_deadband_limit(*wz_set,*wz_set,0.15);
	return;
}
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_keyboard_control_normal(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_keyboard_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_keyboard_to_vector == NULL)
  {
    return;
  }
	switch(low_speed)
	{
		case 0:
			chassis_keyboard_control_heigh(vx_set, vy_set, wz_set, chassis_move_keyboard_to_vector); //在高速模式下值会先滤波再输出
			break;
		case 1:
			chassis_keyboard_control_low(vx_set, vy_set, wz_set, chassis_move_keyboard_to_vector);  //在低速模式下成倍更改速度值
			*vx_set *= 5;
			*vy_set *= 5;
			break;
		case 2:
			chassis_keyboard_control_low(vx_set, vy_set, wz_set, chassis_move_keyboard_to_vector);
			break;
		default:
			break;
	}
	return;
}
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上 
 * @brief					 此为键盘控制低速模式
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_keyboard_control_low(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_keyboard_control_low)
{
	//键盘控制设定值
	if(!(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT))
	{
		if(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_W) //处于低速模式时
		{
			*vy_set = -chassis_keyboard_control_low->vy_max_speed / 30.0;
		}
		else if(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_S)
		{
			*vy_set = -chassis_keyboard_control_low->vy_min_speed / 30.0;
		}
		
		if(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_A)
		{
			*vx_set = chassis_keyboard_control_low->vx_min_speed / 30.0;
		}
		else if(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_D)
		{
			*vx_set = chassis_keyboard_control_low->vx_max_speed / 30.0;
		}
		//键盘x轴移动速度控制底盘旋转
		if(chassis_keyboard_control_low->chassis_RC->mouse.x | 0x00)
		{
			*wz_set = -chassis_keyboard_control_low->chassis_RC->mouse.x * CHASSIS_MOUSE_CONTROL_CHANGE_TO_VEL;
		}
//		if(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
//		{
//			*wz_set = 1.0;
//		}
//		else if(chassis_keyboard_control_low->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)
//		{
//			*wz_set = -1.0;
//		}
	}
	
}
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上 
 * @brief					 此为键盘控制高速模式加缓冲起步与缓步停止
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_keyboard_control_heigh(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_keyboard_control_heigh)
{
		float vy_set_channel,vx_set_channel;
	if(!(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)) //当没有点按SHIFT时 因点按SHIFT会触发云台运动 故此处理
	{
			//键盘控制设定值
//		if(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_W) //处于高速模式时
//		{
//			*vy_set = -chassis_keyboard_control_heigh->vy_max_speed;
//		}
//		else if(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_S)
//		{
//			*vy_set = -chassis_keyboard_control_heigh->vy_min_speed;
//		}
//		if(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_A)
//		{
//			*vx_set = chassis_keyboard_control_heigh->vx_min_speed;
//		}
//		else if(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_D)
//		{
//			*vx_set = chassis_keyboard_control_heigh->vx_max_speed;
//		}
	//键盘x轴移动速度控制底盘旋转
		if(chassis_keyboard_control_heigh->chassis_RC->mouse.x | 0x00)
		{
			*wz_set = -chassis_keyboard_control_heigh->chassis_RC->mouse.x * CHASSIS_MOUSE_CONTROL_CHANGE_TO_VEL;
		}
//		if(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
//		{
//			*wz_set = 5.0;
//		}
//		else if(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)
//		{
//			*wz_set = -5.0;
//		}	
		
		//键盘控制
  if (chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_A) //左移
  {
    if( vx_set_use > 0 )
    {
      vx_set_use = 0;
    }
    if(vx_set_channel > chassis_keyboard_control_heigh->vx_min_speed  )
    {
			vx_set_use=vx_set_use-0.01f;
      vx_set_channel=vx_set_use;
    }
    else
    {
      vx_set_channel = chassis_keyboard_control_heigh->vx_min_speed;
    }
  }
  if (chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_D) //右移
  {
    if( vx_set_use < 0 )
    {
      vx_set_use = 0;
    }
    if(vx_set_channel < chassis_keyboard_control_heigh->vx_max_speed )
    {
			vx_set_use=vx_set_use+0.01f;
      vx_set_channel=vx_set_use;
    }
    else
    {
      vx_set_channel = chassis_keyboard_control_heigh->vx_max_speed;
    }
  }
	if(!(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_A)&&!(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_D)) 
	{
		vx_set_use = 0.0f;
	}

  if (chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_W) //前进
  {
    if( vy_set_use > 0 )
    {
      vy_set_use = 0;
    }
    if( vy_set_channel > chassis_keyboard_control_heigh->vy_min_speed )
    {
			vy_set_use=vy_set_use-0.01f;
      vy_set_channel=vy_set_use;
    }
    else
    {
      vy_set_channel = chassis_keyboard_control_heigh->vy_min_speed;
    }
  }
  if (chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_S) //后退
  {
    //防止响应过慢
    if( vy_set_use < 0 )
    {
      vy_set_use = 0;
    }
   if(vy_set_channel < chassis_keyboard_control_heigh->vy_max_speed )
    {
			vy_set_use=vy_set_use+0.01f;
      vy_set_channel=vy_set_use;
    }
    else
    {
      vy_set_channel = chassis_keyboard_control_heigh->vy_max_speed;
    }
  }
	if(!(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_W) && !(chassis_keyboard_control_heigh->chassis_RC->key.v & KEY_PRESSED_OFFSET_S))
	{
		vy_set_use=0.0f;
	}
		// first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
		//一阶低通滤波代替斜波作为底盘速度输入
		first_order_filter_cali(&chassis_keyboard_control_heigh->chassis_cmd_slow_set_vx, vx_set_channel);
		first_order_filter_cali(&chassis_keyboard_control_heigh->chassis_cmd_slow_set_vy, vy_set_channel);
		// stop command, need not slow change, set zero derectly
		//停止信号，不需要缓慢加速，直接减速到零
		if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
		{
			chassis_keyboard_control_heigh->chassis_cmd_slow_set_vx.out = 0.0f;
		}
	
		if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
		{
			chassis_keyboard_control_heigh->chassis_cmd_slow_set_vy.out = 0.0f;
		}
		
		//速度限幅
		if(vx_set_channel >= chassis_keyboard_control_heigh->vx_max_speed)
		{
			vx_set_channel = chassis_keyboard_control_heigh->vx_max_speed - 4;
		}
		else if(vx_set_channel < chassis_keyboard_control_heigh->vx_min_speed)
		{
			vx_set_channel = chassis_keyboard_control_heigh->vx_min_speed + 4;
		}
		
		if(vy_set_channel >= chassis_keyboard_control_heigh->vy_max_speed)
		{
			vy_set_channel = chassis_keyboard_control_heigh->vy_max_speed - 4;
		}
		else if(vy_set_channel < chassis_keyboard_control_heigh->vy_min_speed)
		{
			vy_set_channel = chassis_keyboard_control_heigh->vy_min_speed + 4;
		}
		
		*vx_set = vx_set_channel;
		*vy_set = vy_set_channel;
	
	}
	
	vx_test = *vx_set;
	vy_test = *vy_set;
}