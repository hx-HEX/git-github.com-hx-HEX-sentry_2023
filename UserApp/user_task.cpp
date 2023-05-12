#include "common_inc.h"
#include "user_task.h"
#include "usart.h"


#define TASK_CREATE(NAME, FUNCTION, STACK_SIZE, PARAMETER, PRIORITY, HANDLE) xTaskCreate(\
(TaskFunction_t)(FUNCTION), (const char*)(NAME), (u16)(STACK_SIZE), (void*)(PARAMETER),\
(UBaseType_t)(PRIORITY), (TaskHandle_t*	) &(HANDLE))

TaskHandle_t LED_Task_Handle;
TaskHandle_t DataVisual_Task_Handle;
TaskHandle_t RobotControl_Task_Handle;
TaskHandle_t DataCommunicate_Task_Handle;


/**
 * @brief communication task
 *
 * @param NULL
 */
void DataCommunicate_Task(void) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(DataCommunicate_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		Global::gimbal.m_data_send_frame.m_fdata[0] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
		Global::gimbal.m_data_send_frame.m_fdata[1] = Global::sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed->GetFilterOutput();
		#ifdef FIRST_FITCH_ANGLE_ENCODER_FEEDBACK
		Global::gimbal.m_data_send_frame.m_fdata[2] = -Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
		#endif
		#ifdef FIRST_FITCH_ANGLE_IMU_FEEDBACK
		Global::gimbal.m_data_send_frame.m_fdata[2] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
		#endif
		#ifdef FIRST_FITCH_SPEED_IMU_FEEDBACK
		Global::gimbal.m_data_send_frame.m_fdata[3] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_y->GetFilterOutput() * RADIANODEGREES;
		#endif
		#ifdef FIRST_FITCH_SPEED_ENCODER_FEEDBACK
		Global::gimbal.m_data_send_frame.m_fdata[3] = -Global::sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_speed->GetFilterOutput();
		#endif
		Global::gimbal.m_data_send_frame.m_fdata[4] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
		Global::gimbal.m_data_send_frame.m_fdata[5] = Global::sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIANODEGREES;
		
		Global::gimbal.SendData();
		
		Global::system_monitor.DataCommunicateTask_cnt++; // Statistic task execution times
		Global::system_monitor.DataCommunicateTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}


/**
 * @brief Data visual task
 *
 * @param NULL
 */
void DataVisualize_Task(void) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(DataVisualize_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		// the ID of mdata should be less than 15
		#ifdef GIMBAL_DEBUG
		Global::VisualizeGimbalData();
		#endif
		
		Global::vofa.m_data_send_frame.m_data[15] = Global::system_monitor.SysTickTime;

		Global::vofa.SendData();

		Global::system_monitor.DataVisualizeTask_cnt++; // Statistic task execution times
		Global::system_monitor.DataVisualizeTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief LED task
 *
 * @param NULL
 */
void LED_Task(void) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(LED_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		Global::led.ToggleGreen();

		if (Global::system_monitor.UART3_rx_fps > 900) {
			Global::led.ToggleBlue();
		}

		Global::system_monitor.LEDTask_cnt++; // Statistic task execution times
		Global::system_monitor.LEDTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief Robot control task
 *
 * @param NULL
 */
void RobotControl_Task(void) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(RobotControl_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		// Update the control mode
		Global::ControlModeUpdate();
		// Update the robot state
		Global::RobotStatesUpdate();
		// Update the robot targets
		Global::RobotTargetsUpdate();
		// Execute the robot control
		Global::RobotControlExecute();

		Global::system_monitor.RobotControlTask_cnt++; // Statistic task execution times
		Global::system_monitor.RobotControlTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief Launch all the user tasks 
 *
 * @param NULL
 */
void LaunchAllTasks(void) {
	taskENTER_CRITICAL(); // Enter task critical section

    TASK_CREATE("LED", LED_Task, 200, NULL, 1, LED_Task_Handle); // Create LED task
	TASK_CREATE("DataVisual", DataVisualize_Task, 200, NULL, 4, DataVisual_Task_Handle); // Create data visual task
	TASK_CREATE("RobotControl", RobotControl_Task, 200, NULL, 2, RobotControl_Task_Handle); // Create data visual task
	TASK_CREATE("DataCommunicate", DataCommunicate_Task, 200, NULL, 3, DataCommunicate_Task_Handle); // Create data visual task

	taskEXIT_CRITICAL(); // Exit task critical section

	vTaskStartScheduler(); // Start FreeRTOS scheduler to launch all tasks
}