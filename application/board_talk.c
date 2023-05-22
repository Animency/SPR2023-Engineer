#include "board_talk.h"
#include "usart.h"
#include "chassis_task.h"
#include "crc8_crc16.h"

u8 DATA_Receive[4][7];
u8 DATA_Receive_buf[4][7];
u8 slave_board_receive[2][SLAVE_RX_BUF_LENGTH];
u8 uart1_send_data[CMD_Total];
u8 nx_board_recive[2][NX_RX_BUF_LENGTH];

float nx_data[NX_Total];
float imu_data[9];
float board_talk_recive_data[DATA_Total];
//usart1_tx_dma_enable(send_data,CMD_Total);
void slave_uart_trigger_init(void)
{
    slave_talk_Init(slave_board_receive[0],slave_board_receive[1],SLAVE_RX_BUF_LENGTH);	
	  //uart1_send_data[CMD_PWM_COUNT ] = 1;
}

void nvidia_board_uart_init(void)
{
	cmd_board_uart_init(nx_board_recive[0], nx_board_recive[1], NX_RX_BUF_LENGTH);
}



void board_talk_data_trans_flo_to_uint(uint8_t init[],float cove[],int len)
{
	union{
		float init;
		uint8_t init_8[4];
	}trans;

	for(int i=0;i<len;i++)
	{
	  trans.init=init[i];
		
		
		cove[i*4]=trans.init_8[0];
		cove[i*4+1]=trans.init_8[1];
		cove[i*4+2]=trans.init_8[2];
		cove[i*4+3]=trans.init_8[3];
	}
}

void board_talk_data_trans_uint_to_flo(float cove[],uint8_t init[],int len)
{
	
	union{
		float cove;
		uint8_t cove_8[4];
	}trans;
	
  for(int i=0;i<len;i++)
	{
		trans.cove_8[0]=init[i*4];
		trans.cove_8[1]=init[i*4+1];
		trans.cove_8[2]=init[i*4+2];
		trans.cove_8[3]=init[i*4+3];
		
		cove[i]=trans.cove;
	}
}

void UART_DATA_RECIVE(uint8_t init_recive_data[],float board_talk_recive_data[])
{
	uint8_t valia_data[4*DATA_Total];
	uint8_t data_crc[4*DATA_Total+5];
	
	for(int i=0;i<4*DATA_Total+5;i++)
	{
		data_crc[i]=init_recive_data[i];
	}
	append_CRC16_check_sum(data_crc,4*DATA_Total+5);
	
	for(int i=0;i<4*DATA_Total;i++)
	{
		valia_data[i]=init_recive_data[i+2];
	}
	
	if(init_recive_data[0]==0x5A&&init_recive_data[1]==0xA5)
	{				
		board_talk_data_trans_uint_to_flo(board_talk_recive_data,valia_data,DATA_Total);		
			
	}
	else
  {
		return;
	}
								
}

void NX_DATA_RECIVE(uint8_t init_recive_data[],float nx_talk_recive_data[])
{
	uint8_t valia_data[4*NX_Total];
	uint8_t data_crc[4*NX_Total+5];
	
	for(int i=0;i<4*NX_Total+5;i++)
	{
		data_crc[i]=init_recive_data[i];
	}
	append_CRC16_check_sum(data_crc,4*NX_Total+5);
	
	for(int i=0;i<4*NX_Total;i++)
	{
		valia_data[i]=init_recive_data[i+2];
	}
	
	if(init_recive_data[0]==0xA5&&init_recive_data[1]==0x5A)
	{				
		board_talk_data_trans_uint_to_flo(nx_talk_recive_data,valia_data,NX_Total);		
			
	}
	else
  {
		return;
	}
								
}

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) //?????
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //??DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //????????,?? = ???? - ????
            this_time_rx_len = SLAVE_RX_BUF_LENGTH - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //????????
            hdma_usart1_rx.Instance->NDTR = SLAVE_RX_BUF_LENGTH;

            //set memory buffer 1
            //?????1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //??DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == SLAVE_DATA_LENGTH)
            {
							UART_DATA_RECIVE(slave_board_receive[0],board_talk_recive_data);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //??DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //????????,?? = ???? - ????
            this_time_rx_len = SLAVE_RX_BUF_LENGTH - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //????????
            hdma_usart1_rx.Instance->NDTR = SLAVE_RX_BUF_LENGTH;

            //set memory buffer 0
            //?????0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //??DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == SLAVE_DATA_LENGTH)
            {
							UART_DATA_RECIVE(slave_board_receive[1],board_talk_recive_data);
            }
        }
    }
}

