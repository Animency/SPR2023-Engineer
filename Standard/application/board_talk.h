
#ifndef __BOARD_TALK_H
#define __BOARD_TALK_H

#include "struct_typedef.h"
#include "bsp_rc.h"
#include "bsp_usart.h"
	
//从从板接收的数据
typedef enum
{
	IMU_ACC_PITCH,
	IMU_ACC_ROLL,
	IMU_ACC_YAW,
	IMU_GYR_PITCH,
	IMU_GYR_ROLL,
	IMU_GYR_YAW,
	IMU_EUL_PITCH,
	IMU_EUL_ROLL,
	IMU_EUL_YAW,
	SICK_1,
	SICK_2,
	SICK_3,
	DATA_Total,
}slave_data_e;
	
//发送给从板的数据
typedef enum
{
	CMD_PWM_COUNT,
	CMD_Total,
}cmd_data_e;

#define SLAVE_TX_DATA_LENGTH CMD_Total

#define SLAVE_DATA_LENGTH 4*DATA_Total+5
#define SLAVE_RX_BUF_LENGTH 8*DATA_Total+10

extern u8 slave_board_receive[2][SLAVE_RX_BUF_LENGTH];
extern u8 uart1_send_data[CMD_Total];
extern float board_talk_recive_data[DATA_Total];

void slave_uart_trigger_init(void);
void board_talk_data_trans_flo_to_uint(uint8_t init[],float cove[],int len);
void board_talk_data_trans_uint_to_flo(float cove[],uint8_t init[],int len);

#endif
