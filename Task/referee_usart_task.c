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
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_usart.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

void referee_usart_task(void const *pvParameters)
{
    vTaskDelay(20);

    while (1)
    {

        vTaskDelay(20);
    }
}
