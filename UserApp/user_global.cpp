#include "common_inc.h"
#include "user_global.h"

SystemMonitor Global::system_monitor;
Led Global::led;
Vofa Global::vofa;
Gimbal Global::gimbal;
SentryRobot Global::sentry;

Global::ControlMode Global::control_mode;


/**
 *@brief visualize chassis datas
 * 
 *@param 
*/
void Global::VisualizeGimbalData(void) {
    // Global::vofa.m_data_send_frame.m_data[1] = Global::gimbal.m_data_receive_frame.m_sdata[0];
    // Global::vofa.m_data_send_frame.m_data[2] = Global::gimbal.m_data_receive_frame.m_sdata[1];
    // Global::vofa.m_data_send_frame.m_data[3] = Global::gimbal.m_data_receive_frame.m_sdata[2];
    // Global::vofa.m_data_send_frame.m_data[4] = Global::gimbal.m_data_receive_frame.m_cdata[0];
    // Global::vofa.m_data_send_frame.m_data[5] = Global::gimbal.m_data_receive_frame.m_cdata[1];
    // Global::vofa.m_data_send_frame.m_data[6] = Global::gimbal.m_data_receive_frame.m_cdata[2];
    // Global::vofa.m_data_send_frame.m_data[7] = Global::gimbal.m_data_receive_frame.m_cdata[3];
    // Global::vofa.m_data_send_frame.m_data[8] = Global::gimbal.m_data_receive_frame.m_cdata[4];
    // Global::vofa.m_data_send_frame.m_data[0] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_encoder->m_raw_value;
    // Global::vofa.m_data_send_frame.m_data[5] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_angle->GetFilterOutput();
    // Global::vofa.m_data_send_frame.m_data[6] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    // Global::vofa.m_data_send_frame.m_data[7] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_angle->GetFilterOutput();
    // Global::vofa.m_data_send_frame.m_data[4] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output;
    // Global::vofa.m_data_send_frame.m_data[5] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output;
    Global::vofa.m_data_send_frame.m_data[1] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    Global::vofa.m_data_send_frame.m_data[2] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current;
    // Global::vofa.m_data_send_frame.m_data[8] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed->GetFilterOutput();
    Global::vofa.m_data_send_frame.m_data[9] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
    // Global::vofa.m_data_send_frame.m_data[9] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current;
    Global::vofa.m_data_send_frame.m_data[0] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.x;
    Global::vofa.m_data_send_frame.m_data[1] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.y;
    Global::vofa.m_data_send_frame.m_data[2] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.z;
    // Global::vofa.m_data_send_frame.m_data[4] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_x;
    // Global::vofa.m_data_send_frame.m_data[5] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_y;
    // Global::vofa.m_data_send_frame.m_data[6] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_z;
    Global::vofa.m_data_send_frame.m_data[3] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.x;
    Global::vofa.m_data_send_frame.m_data[4] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.y;
    Global::vofa.m_data_send_frame.m_data[5] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.z;
    Global::vofa.m_data_send_frame.m_data[6] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
    Global::vofa.m_data_send_frame.m_data[7] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
    Global::vofa.m_data_send_frame.m_data[8] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.roll;
    // Global::vofa.m_data_send_frame.m_data[0] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.x;
    // Global::vofa.m_data_send_frame.m_data[1] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.y;
    // Global::vofa.m_data_send_frame.m_data[2] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.z;
    // Global::vofa.m_data_send_frame.m_data[3] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_x;
    // Global::vofa.m_data_send_frame.m_data[4] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_y;
    // Global::vofa.m_data_send_frame.m_data[5] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_z;
    // Global::vofa.m_data_send_frame.m_data[6] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.x;
    // Global::vofa.m_data_send_frame.m_data[7] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.y;
    // Global::vofa.m_data_send_frame.m_data[8]= Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.z;
    // Global::vofa.m_data_send_frame.m_data[9] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
    // Global::vofa.m_data_send_frame.m_data[10] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
    // Global::vofa.m_data_send_frame.m_data[11] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.roll;
    // Global::vofa.m_data_send_frame.m_data[12] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.z;
    // Global::vofa.m_data_send_frame.m_data[13] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIAN2DEGREE_VALUE;
    // Global::vofa.m_data_send_frame.m_data[14] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_com_gyro.z;
    // Global::vofa.m_data_send_frame.m_data[4] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_TD->m_aim;
    // Global::vofa.m_data_send_frame.m_data[5] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_TD->m_x1;
    // Global::vofa.m_data_send_frame.m_data[6] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_TD->m_x2;
    // Global::vofa.m_data_send_frame.m_data[7] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpE;
    // Global::vofa.m_data_send_frame.m_data[8] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpU;
    // Global::vofa.m_data_send_frame.m_data[9] = Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpU;
    // Global::vofa.m_data_send_frame.m_data[10] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes;
    // Global::vofa.m_data_send_frame.m_data[11] = Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes;
    // Global::vofa.m_data_send_frame.m_data[12] = Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpFB;
    // Global::vofa.m_data_send_frame.m_data[13] = Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpFB;

    Global::vofa.m_data_send_frame.m_data[13] = Global::system_monitor.UART3_rx_fps;
    Global::vofa.m_data_send_frame.m_data[14] = Global::system_monitor.CAN2_rx_fps;
}



/**
 *@brief update system control mode
 * 
 *@param 
*/
void Global::ControlModeUpdate(void) {
};



/**
 *@brief update the state of the sentry robot
 * 
 *@param 
*/
void Global::RobotStatesUpdate(void)
{
    if (Global::system_monitor.UART3_rx_fps < 900) {
        Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output
        = 0;
        Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output
        = 0;
    } else {
        Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output
        = Global::gimbal.m_data_receive_frame.m_sdata[0];

        Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output
        = Global::gimbal.m_data_receive_frame.m_sdata[1];
    }

    Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAccData();
    Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataGyroData();
    Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAngleData();

    // Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_angle
    // ->UpdateFilter(Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current);
    // Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_angle
    // ->UpdateFilter(Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current);
    
    Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed
    ->UpdateFilter(Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current);
    Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_speed
    ->UpdateFilter(Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current);
}



/**
 *@brief update the robot targets 
 * 
 *@param 
*/
void Global::RobotTargetsUpdate(void) {
    if (Global::system_monitor.UART3_rx_fps > 100 && 
    Global::gimbal.m_data_receive_frame.m_cdata[0] == 0) {
        Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes = -FRIC_WHEEL_SPEED;
        Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes = FRIC_WHEEL_SPEED;
    } else {
        Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes = 0;
        Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes = 0;
    }
}



/**
 *@brief execute the robot control 
 * 
 *@param 
*/
void Global::RobotControlExecute(void)
{
    // Execute the robot shoot control algorithm
    if (Global::system_monitor.UART3_rx_fps > 100 && 
    Global::gimbal.m_data_receive_frame.m_cdata[0] == 0) {
        Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->CalSMC();
        Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->CalSMC();

    } else {
        Global::sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpU = 0;
        Global::sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpU = 0;
    }

    // Send the control commands to the actuators
    Global::sentry.SendControlCommand();
}