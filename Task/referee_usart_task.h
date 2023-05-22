/**
 * @file referee_usart_task.c/h
 * @author 何清华
 * @brief Robomaster裁判系统通信处理任务线程
 * @version 0.1
 * @date 2022-03-06
 * 
 * @copyright Copyright (c) 2022 SPR
 * 
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
extern uint16_t draw_init_flag;
/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);
#endif
