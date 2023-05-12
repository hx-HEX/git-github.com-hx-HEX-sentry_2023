#pragma once 

#include "gimbal.h"
#include "vofa.h"

#define UART1_RECEIVE_DATA_SIZE  10
#define UART2_RECEIVE_DATA_SIZE  10
#define UART3_RECEIVE_DATA_SIZE  GIMBAL_DATA_RECEIVE_SIZE
#define UART4_RECEIVE_DATA_SIZE  VOFA_DATA_RECEIVE_SIZE
#define UART5_RECEIVE_DATA_SIZE  10
#define UART6_RECEIVE_DATA_SIZE  10

void UART_GimbalDataDecode(unsigned char* buffer);