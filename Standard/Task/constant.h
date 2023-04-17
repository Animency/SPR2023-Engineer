/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONSTANT_H__
#define __CONSTANT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

typedef uint16_t u16;
typedef uint8_t u8;


/* ******************************************************************
				              	COMMON PARAMETER
****************************************************************** */	

#define PI 3.1415926
#define ANGLE_T 8191
	
	
/* ******************************************************************
				              	CHASIS
****************************************************************** */	
	
#define CAN_CHASSIS_ALL_ID 0x200
#define CHA_WHEEL_NUM 4


#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
