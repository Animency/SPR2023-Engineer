/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "chassis_task.h"

#include "main.h"

float pid_current_actual=0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
 motor_measure_t motor_chassis[4]; //底盘电机
 motor_measure_t motor_gimbal[8];  //云台电机
static motor_measure_t motor_trigger;    //拨弹电机
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
/************************************定义各电机角度方便调用**********************************************/
float angle_can2_201;
float angle_can2_202;
float angle_can2_203;
float angle_can2_204;
float angle_can2_205;
float angle_can2_206;
float angle_can2_207_6020;
float angle_can2_208;
/**
 * @brief           计算3508和6020电机累计旋转角度
 * @param[out]      motor:电机结构数据指针
 */
void Calc_3508_Angle(motor_measure_t *motor)
{
  if (motor->ecd - motor->last_ecd > 4095.5)
  {
    motor->round--;
  }
  else if (motor->ecd - motor->last_ecd < -4095.5)
  {
    motor->round++;
  }
  motor->angle = (motor->round * ANGLE_T + motor->last_ecd) * 0.002288715f;//轴一圈对应360°
}
void Calc_3508_Angle_Gimbal(motor_measure_t *motor_gimbal)
{
  if (motor_gimbal->ecd - motor_gimbal->last_ecd > 4095.5)
  {
    motor_gimbal->round--;
  }
  else if (motor_gimbal->ecd - motor_gimbal->last_ecd < -4095.5)
  {
    motor_gimbal->round++;
  }
  motor_gimbal->angle = (motor_gimbal->round * ANGLE_T + motor_gimbal->last_ecd) * 0.002288715f;//轴一圈对应360°
}
void Calc_6020_Angle_Gimbal(motor_measure_t *motor_gimbal)
{
  if (motor_gimbal->ecd - motor_gimbal->last_ecd > 4095.5)
  {
    motor_gimbal->round--;
  }
  else if (motor_gimbal->ecd - motor_gimbal->last_ecd < -4095.5)
  {
    motor_gimbal->round++;
  }
  motor_gimbal->angle = (motor_gimbal->round * ANGLE_T + motor_gimbal->last_ecd) * 0.0439506776f;//轴一圈对应360°
}
//void Calc_3508_Angle_pitch(motor_measure_t *motor_gimbal)
//{
//  if (motor_gimbal->ecd - motor_gimbal->last_ecd > 4095.5)
//  {
//    motor_gimbal->round--;
//  }
//  else if (motor_gimbal->ecd - motor_gimbal->last_ecd < -4095.5)
//  {
//    motor_gimbal->round++;
//  }
//  motor_gimbal->angle = (motor_gimbal->round * ANGLE_T + motor_gimbal->last_ecd) * 0.052074631021999f;//减速比换算 pitch
//}
/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if(hcan == &hcan1)
	{
  switch (rx_header.StdId)
  {
  case CAN_3508_M1_ID:
  {
    get_motor_measure(&motor_chassis[0], rx_data);
    Calc_3508_Angle(&motor_chassis[0]);
		pid_current_actual=motor_chassis[0].speed_rpm;
    break;
  }
  case CAN_3508_M2_ID:
  {
    get_motor_measure(&motor_chassis[1], rx_data);
    Calc_3508_Angle(&motor_chassis[1]);
    break;
  }
  case CAN_3508_M3_ID:
  {
    get_motor_measure(&motor_chassis[2], rx_data);
    Calc_3508_Angle(&motor_chassis[2]);
    break;
  }
  case CAN_3508_M4_ID:
  {
    get_motor_measure(&motor_chassis[3], rx_data);
    Calc_3508_Angle(&motor_chassis[3]);
    break;
  }
 

  default:
  {
    break;
  }
  }
}
	else if (hcan == &hcan2)
{
	switch (rx_header.StdId)
  {
		case CAN2_3508_GIMBAL_HENG_ID:
  {
    get_motor_measure(&motor_gimbal[0], rx_data);
		Calc_3508_Angle(&motor_gimbal[0]);
    break;
  }
	case CAN2_3508_GIMBAL_SHU_ID:
  {
    get_motor_measure(&motor_gimbal[1], rx_data);
		Calc_3508_Angle(&motor_gimbal[1]);
    break;
  }
	case CAN2_3508_GIMBAL_YAW_ID:  //后该电机更换为2006，接收数据方式更改
  {
    get_motor_measure(&motor_gimbal[7], rx_data);
		Calc_3508_Angle(&motor_gimbal[7]);
    break;
  }
	case CAN2_3508_GIMBAL_FAN_ID:
  {
    get_motor_measure(&motor_gimbal[3], rx_data);
		Calc_3508_Angle(&motor_gimbal[3]);
    break;
  }
	case CAN2_3508_GIMBAL_TAI_ZUO_ID:
  {
    get_motor_measure(&motor_gimbal[4], rx_data);
		Calc_3508_Angle(&motor_gimbal[4]);
    break;
  }
	case CAN2_3508_GIMBAL_TAI_YOU_ID:
  {
    get_motor_measure(&motor_gimbal[5], rx_data);
		Calc_3508_Angle(&motor_gimbal[5]);
    break;
  }
		case CAN2_6020_XUAN_ID:
	{
		get_motor_measure(&motor_gimbal[2], rx_data);
		Calc_6020_Angle_Gimbal(&motor_gimbal[2]); 
		break;
	}
		case CAN2_3508_ORE_ID:
  {
    get_motor_measure(&motor_gimbal[6], rx_data);
		Calc_3508_Angle(&motor_gimbal[6]);
    break;
  }
	 default:
  {
    break;
  }
	}
}

}


void CAN2_cmd_gimbal_tai(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_TAI_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN2_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN2_3508_TAISHENG_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
 * @retval         none
 */
/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_6020_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/**
 * @brief          return the yaw 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回yaw 6020电机数据指针 //本次工程未使用
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[0];
}

/**
 * @brief          return the pitch 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_6020_motor_measure_point(void)
{
 // return &motor_gimbal[2];
}

/**
 * @brief          return the trigger 2006 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回拨弹电机 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const void *get_gimbal_motor_measure_value(void)
{
/************************************定义各电机角度方便调用**********************************************/
	angle_can2_201 = motor_gimbal[0].angle;
	angle_can2_202 = motor_gimbal[1].angle;
	//203已更换
	angle_can2_203 = motor_gimbal[7].angle;  //此电机为2006
	angle_can2_204 = motor_gimbal[3].angle;
	angle_can2_205 = motor_gimbal[4].angle;
	angle_can2_206 = motor_gimbal[5].angle;
	angle_can2_207_6020 = motor_gimbal[2].angle; //此电机为6020
	angle_can2_208 = motor_gimbal[6].angle;  
}
const motor_measure_t *get_gimbal_motor_measure_point(uint8_t t)
{
	if (t>8 || t<0)
		return NULL;
	get_gimbal_motor_measure_value();
  return &motor_gimbal[t];
}

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

