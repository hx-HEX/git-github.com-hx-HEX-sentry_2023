#include "communication.h"
#include "usart.h"
#include "spi.h"
#include "retarget.h"
#include "user_global.h"
#include "can_interface.h"
#include "uart_interface.h"

/**
 * @brief Init all communication staffs,includes interface add protocol
 *
 * @param NULL
 */
void InitCommunication(void)
{
    // Using UART2 for debug input/output
    // RetargetInit(&huart6); 

    // Using UART4 for sending visual data to Vofa+
    Global::vofa.SetUartHandle(&huart6);

    // Using UART3 for gimbal communication
    Global::gimbal.SetUartHandle(&huart3);

    // Specify UART DMA  receive buffer
    HAL_UART_Receive_DMA(&huart3, uart3_dma_rec_buffer, UART3_DMA_REC_BUFFER_SIZE); 
    HAL_UART_Receive_DMA(&huart6, uart6_dma_rec_buffer, UART6_DMA_REC_BUFFER_SIZE);          

    // Use spi1 for imu communication
    Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU] = new BMI088();
    Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->SetSPIHandle(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);

    HAL_Delay(100);
}



/**
 * @brief Start all communication staffs,includes interface add protocol
 *
 * @param NULL
 */
void StartCommunication(void)
{
    // Enable UART IDLE Interruption
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // Configure CAN filter
    CAN_Filter_Config(&hcan1, 0);
    CAN_Filter_Config(&hcan2, 8);
    // Init CAN receive message
    can1_context.Init(&hcan1, CAN_ID_STD, CAN_RTR_DATA, 8);
    can2_context.Init(&hcan2, CAN_ID_STD, CAN_RTR_DATA, 8);
    // Start CAN
    HAL_CAN_Start(&hcan1); 
    HAL_Delay(100); // it works when can2 stop working...
    HAL_CAN_Start(&hcan2);
    HAL_Delay(100); // it works when can2 stop working...
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_Delay(100); // it works when can2 stop working...
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_Delay(100); // it works when can2 stop working...
	
}


