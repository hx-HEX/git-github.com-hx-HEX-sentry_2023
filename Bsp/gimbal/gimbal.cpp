#include "gimbal.h"


/**
 * @brief Set the UART for Gimbal
 *
 * @param UART_HandleTypeDef *huart: UART handle
 */
void Gimbal::SetUartHandle(UART_HandleTypeDef *huart)
{
    m_huart = huart; 
}



/**
 * @brief Send data to Navigation computer
 *
 * @param NULL
 */
void Gimbal::SendData(void)
{
    HAL_UART_Transmit_DMA(m_huart, (unsigned char*)&m_data_send_frame, m_data_send_size);
}