//
// Created by Administrator on 24-10-13.
//

#include "main.h"
#include <string.h>
#include "usart.h"

#define RC_RX_BUF_SIZE 18
#define RC_RX_DATA_SIZE 18

volatile uint8_t data_ready = 0;
extern uint8_t rx_buf_[RC_RX_BUF_SIZE];
extern uint8_t rx_data_[RC_RX_DATA_SIZE];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        memcpy(rx_data_, rx_buf_, RC_RX_BUF_SIZE); // copy to data zone
        data_ready = 1; //means data transfer is finished
        HAL_UART_Receive_DMA(&huart1, rx_buf_, RC_RX_BUF_SIZE); //Receive dma
    }
}