#include "common_inc.h"
#include "sentry_robot.h"
#include "can.h"
#include "m3508.h"
#include "gm6020.h"
#include "pid.h"
#include "adrc.h"
#include "encoder.h"
#include "mahony.h"
#include "kalman.h"
#include "smc.h"


/**
*@brief Initial the whole Sentry robot
* 
*@param 
*/
void SentryRobot::Init(void)
{
    // Initial all actuators
    InitAllActuators();

    // Initial all Sensors  
    InitAllSensors();
}



/**
*@brief Initial all actuators of the Sentry robot
* 
*@param 
*/
void SentryRobot::InitAllActuators(void)
{
    shoot_motor[LEFT_FRIC_MOTOR] = new M2006(&hcan1, LEFT_FRIC_MOTOR_ID,  FRIC_MOTOR_REDUCTION_RATIO);
    shoot_motor[LEFT_FRIC_MOTOR]->m_smc = new Smc(4.4936132, 8000, 5, 20, 16200);
    shoot_motor[LEFT_FRIC_MOTOR]->m_smc->m_TD = new Adrc_TD((float)5000, 0.001, 0.001);
    shoot_motor[LEFT_FRIC_MOTOR]->m_encoder = new AbsEncoder(LEFT_FRIC_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);  
    
    shoot_motor[RIGHT_FRIC_MOTOR] = new M2006(&hcan1, RIGHT_FRIC_MOTOR_ID,  FRIC_MOTOR_REDUCTION_RATIO);
    shoot_motor[RIGHT_FRIC_MOTOR]->m_smc = new Smc(4.4936132, 8000, 5, 20, 16200);
    shoot_motor[RIGHT_FRIC_MOTOR]->m_smc->m_TD = new Adrc_TD((float)5000, 0.001, 0.001);    
    shoot_motor[RIGHT_FRIC_MOTOR]->m_encoder = new AbsEncoder(RIGHT_FRIC_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);  

    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR] = new GM6020(&hcan2, GIMBAL_FIRST_PITCH_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td = new Adrc_TD((float)2000, 0.01, 0.01);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid = new Pid(10, 0.00, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid = new Pid(50, 0.00, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_encoder = new AbsEncoder(FIRST_GIMBAL_PITCH_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    // gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR] = new GM6020(&hcan2, GIMBAL_SECOND_YAW_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_td = new Adrc_TD((float)2000, 0.01, 0.01);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_pid = new Pid(10, 0.01, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid = new Pid(30, 0.01, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_encoder = new AbsEncoder(SECOND_GIMBAL_YAW_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    // gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);
}



/**
 *@brief Initial all sensors of the Sentry robot
 * 
 *@param 
*/
void SentryRobot::InitAllSensors(void)
{
    int imu_flag = -1;
    do {
        imu_flag = gimbal_imu[GIMBAL_FIRST_IMU]->Init(IMU_CALIBRATE);
    } while (imu_flag != 0);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_mahony_filter = new Mahony(0.001f, 0.5f, 0.001f);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_y = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
}



/**
 *@brief control the shoot motor to move 
 * 
 *@param 
*/
void SentryRobot::MoveShoot(void)
{

}



/**
 *@brief control the gimbal to move
 * 
 *@param 
*/
void SentryRobot::MoveGimbal(void)
{
    if (gimbal_mode != GIMBAL_SAFE) {
        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++) {
            // gimbal_motor[i]->AngleControl();
        }
    } else {
        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++) {
            // gimbal_motor[i]->DisableControl();
        }
    }
}



/**
 *@brief Set the sentry robot first gimbal pitch to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetFirstGimbalPitchAngleTarget(float target)
{
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_target = target;
}



/**
 *@brief Set the sentry robot second gimbal yaw to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetSecondGimbalYawAngleTarget(float target)
{
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_target = target;
}



/**
 *@brief Set the sentry robot chassis steer angle to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetFricWheelSpeedTarget(float l_target, float r_target)
{
    shoot_motor[LEFT_FRIC_MOTOR]->m_speed_target = l_target;
    shoot_motor[RIGHT_FRIC_MOTOR]->m_speed_target = r_target;
}



/**
 *@brief Send control command to all actuators
 * 
 *@param 
*/
void SentryRobot::SendControlCommand(void)
{
    // CAN1 ID 0x200    CAN2 ID 0x200
    can1_context.tx_header.StdId = 0x200;
    can2_context.tx_header.StdId = 0x200;
    for (int i = 0; i < 8; i++) {
        can1_context.tx_data[i] = 0;
        can2_context.tx_data[i] = 0;
    }

    // CanSendMessage(&can1_context);
    // CanSendMessage(&can2_context);


    // CAN1 ID 0x1ff    CAN2 ID 0x1ff
    can1_context.tx_header.StdId = 0x1ff;
    can2_context.tx_header.StdId = 0x1ff;
    for (int i = 0; i < 8; i++) {
        can1_context.tx_data[i] = 0;
        can2_context.tx_data[i] = 0;
    }

    if(Global::system_monitor.CAN2_rx_fps < 1500)
    {
         for (int i = 0; i < GIMBAL_MOTOR_NUM; i++)
         {
            gimbal_motor[i]->m_speed_pid->m_output = 0;
         }
    }
    for (int i = 0; i < GIMBAL_MOTOR_NUM; i++) {
        uint32_t id = gimbal_motor[i]->m_id;
        int16_t cmd = (int16_t)gimbal_motor[i]->m_speed_pid->m_output;
        if (id >= 1 && id <= 4) {
            if (gimbal_motor[i]->m_hcan == &hcan1) {
                can1_context.tx_data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.tx_data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (gimbal_motor[i]->m_hcan == &hcan2) {
                can2_context.tx_data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.tx_data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }


    if(Global::system_monitor.CAN1_rx_fps < 1500)
    {
         for (int i = 0; i < SHOOT_MOTOR_NUM; i++)
         {
            shoot_motor[i]->m_smc->m_fpU = 0;
         }
    }
    for (int i = 0; i < SHOOT_MOTOR_NUM; i++) {
        uint32_t id = shoot_motor[i]->m_id;
        int16_t cmd = (int16_t)shoot_motor[i]->m_smc->m_fpU;
        if (id >= 1 && id <= 4) {
            if (shoot_motor[i]->m_hcan == &hcan1) {
                can1_context.tx_data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.tx_data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[i]->m_hcan == &hcan2) {
                can2_context.tx_data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.tx_data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        } else if (id >= 5 && id <= 8) {
            if (shoot_motor[i]->m_hcan == &hcan1) {
                can1_context.tx_data[(id - 5) * 2] = (uint8_t)(cmd >> 8);
                can1_context.tx_data[(id - 5) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[i]->m_hcan == &hcan2) {
                can2_context.tx_data[(id - 5) * 2] = (uint8_t)(cmd >> 8);
                can2_context.tx_data[(id - 5) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }
    CanSendMessage(&can1_context);
    CanSendMessage(&can2_context);
}