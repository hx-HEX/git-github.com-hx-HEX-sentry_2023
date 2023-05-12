#pragma once

#include "usart.h"

#define GIMBAL_DATA_SEND_FLOAT_NUM          ((uint8_t)6)
#define GIMBAL_DATA_SEND_SHORT_NUM          ((uint8_t)0)
#define GIMBAL_DATA_SEND_CHAR_NUM          ((uint8_t)0)
#define GIMBAL_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)0)
#define GIMBAL_DATA_RECEIVE_SHORT_NUM       ((uint8_t)3)
#define GIMBAL_DATA_RECEIVE_CHAR_NUM       ((uint8_t)5)
#define GIMBAL_DATA_SEND_SIZE            ((uint8_t)4 + 4 * GIMBAL_DATA_SEND_FLOAT_NUM + \
                                        2 * GIMBAL_DATA_SEND_SHORT_NUM + GIMBAL_DATA_SEND_CHAR_NUM)
#define GIMBAL_DATA_RECEIVE_SIZE            ((uint8_t)4 + 4 * GIMBAL_DATA_RECEIVE_FLOAT_NUM + \
                                        2 * GIMBAL_DATA_RECEIVE_SHORT_NUM + GIMBAL_DATA_RECEIVE_CHAR_NUM)


class Gimbal {
public:
    #pragma pack(1)
    struct GimbalDataSendFrame {
        unsigned char m_head[2];
        float m_fdata[GIMBAL_DATA_SEND_FLOAT_NUM];
        unsigned char m_tail[2];
    };
    struct GimbalDataReceiveFrame {
        unsigned char m_head[2];
        int16_t m_sdata[GIMBAL_DATA_RECEIVE_SHORT_NUM]; 
        unsigned char m_cdata[GIMBAL_DATA_RECEIVE_CHAR_NUM]; 
        unsigned char m_tail[2];
    };
    #pragma pack()

    unsigned char m_data_send_size = GIMBAL_DATA_SEND_SIZE;
    GimbalDataSendFrame m_data_send_frame = {.m_head = {0x55, 0x00}, .m_tail = {0x00, 0xAA}};
    GimbalDataReceiveFrame m_data_receive_frame = {.m_head = {0x55, 0x00}, .m_tail = {0x00, 0xAA}};

    Gimbal(){};
    Gimbal(UART_HandleTypeDef *huart) { m_huart = huart; };

    void SetUartHandle(UART_HandleTypeDef *huart);
    void SendData(void);

private:
    UART_HandleTypeDef* m_huart;
};