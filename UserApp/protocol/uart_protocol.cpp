#include "common_inc.h"
#include "uart_protocol.h"
#include "usart.h"


/**
 * @brief Decode the gimbal datas
 *
 * @param NULL
 */
void UART_GimbalDataDecode(u8* buffer)
{
	if(	buffer[0]==0x55 && buffer[1]==0x00) {
		memcpy(&(Global::gimbal.m_data_receive_frame), (u8*)buffer, 
		sizeof(Global::gimbal.m_data_receive_frame));	
    }	
}