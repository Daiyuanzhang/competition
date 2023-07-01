#include "start_task.h"
#include "FreeRTOS.h"  
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "MotorCan.h"
#include "RemotDbus.h"
#include "event_groups.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "detect_task.h"
#include "BasicPeripherals.h"
#include "calibrate_task.h"
#include "chassis_task.h"
#include "MainFocus_Usart.h"
#include "Referee_DispatchTask.h"
#include "usart.h"
#include "hand_task.h"
#include "Hand_Calibrate.h"

//TimerHandle_t	OneShotTimer_Handle;			//单次定时器句柄
//void MotorRunTimeLimit(TimerHandle_t xTimer);		//单次定时器回调函数

//TimerHandle_t	AutoAdjustStepOne_Handle;			//单次定时器句柄
//void AutoAdjustStepOne(TimerHandle_t xTimer);		//单次定时器回调函数

//TimerHandle_t	AutoAdjustStepTwo_Handle;			//单次定时器句柄
//void AutoAdjustStepTwo(TimerHandle_t xTimer);		//单次定时器回调函数

//TimerHandle_t	AutoAdjustStepThree_Handle;			//单次定时器句柄
//void AutoAdjustStepThree(TimerHandle_t xTimer);		//单次定时器回调函数

//TimerHandle_t	AutoAdjustStepFour_Handle;			//单次定时器句柄
//void AutoAdjustStepFour(TimerHandle_t xTimer);		//单次定时器回调函数

//QueueHandle_t HandCalibrateFlag;   		//消息队列句柄
//QueueHandle_t Message_Queue;	        //信息队列句柄
//#define  HandCalibrateLength    5

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t Start_Task_Handle;

#define GIMBAL_TASK_PRIO 5
#define GIMBAL_STK_SIZE 1024
TaskHandle_t GIMBALTask_Handler;

//#define Hand_TASK_PRIO 7
//#define Hand_STK_SIZE  1024
//TaskHandle_t HandTask_Handler;

#define HAND_CALIBRATE_TASK_PRIO 7
#define HAND_CALIBRATE_STK_SIZE  256
TaskHandle_t Hand_Calibrate_Task_Handler;

#define INS_TASK_PRIO 6
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

#define DETECT_TASK_PRIO 3
#define DETECT_TASK_SIZE 512
static TaskHandle_t DetectTask_Handler;

#define CALIBRATE_TASK_PRIO 2
#define CALIBRATE_STK_SIZE 512
static TaskHandle_t CalibrateTask_Handler;

#define Chassis_TASK_PRIO 2
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;
void Sned_MainFoucs_Task(void *pvParameters);
void UsartDebugTask(void *parmas);
void Send_ClientData_Task(void *parmas);
void start_task(void *pvParameters)
{	
	taskENTER_CRITICAL();
	
	 //创建云台控制任务
	    xTaskCreate((TaskFunction_t)GIMBAL_task,
                (const char *)"GIMBAL_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GIMBALTask_Handler);

//								
//				    //创建单次定时器
//OneShotTimer_Handle =  xTimerCreate((const char*			)"MotorRunTimeLimit",
//											 (TickType_t			)50,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )1,
//											 (TimerCallbackFunction_t)MotorRunTimeLimit); //单次定时器，周期2s(2000个时钟节拍)，单次模式					  
//			
//											 
//											 
//AutoAdjustStepOne_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)100,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )2,
//											 (TimerCallbackFunction_t)AutoAdjustStepOne); //单次定时器，周期2s(2000个时钟节拍)，单次模式					  
//																					 
//AutoAdjustStepTwo_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)150,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )3,
//											 (TimerCallbackFunction_t)AutoAdjustStepTwo); //单次定时器，周期2s(2000个时钟节拍)，单次模式					  
//			
//AutoAdjustStepThree_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)200,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )4,
//											 (TimerCallbackFunction_t)AutoAdjustStepThree); //单次定时器，周期2s(2000个时钟节拍)，单次模式					  
//			
//AutoAdjustStepFour_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)300,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )5,
//											 (TimerCallbackFunction_t)AutoAdjustStepFour); //单次定时器，周期2s(2000个时钟节拍)，单次模式					  
//			
			 
											 
											 
		//创建消息队列
//    HandCalibrateFlag = xQueueCreate(HandCalibrateLength,sizeof(int));        //创建消息Key_Queue

											 
//	xTimerStart(OneShotTimer_Handle,0);		//开启单次定时器			
			 //创建抓手控制任务
//   xTaskCreate((TaskFunction_t)Hand_Calibrate_Task,
//                (const char *)"Hand_Calibrate_Task",
//                (uint16_t)HAND_CALIBRATE_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)HAND_CALIBRATE_TASK_PRIO,
//                (TaskHandle_t *)&Hand_Calibrate_Task_Handler);						
												 
										 
//						xTimerStart(OneShotTimer_Handle,0);		//开启单次定时器			
//			 //创建抓手控制任务
//   xTaskCreate((TaskFunction_t)Hand_task,
//                (const char *)"Hand_task",
//                (uint16_t)Hand_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)Hand_TASK_PRIO,
//                (TaskHandle_t *)&HandTask_Handler);						
								
   //创建mpu6500任务				
    xTaskCreate((TaskFunction_t)INSTask,
                (const char *)"INSTask",
                (uint16_t)INS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);	  

	 //创建模块离线监测任务	
    xTaskCreate((TaskFunction_t)DetectTask,
                (const char *)"DetectTask",
                (UBaseType_t)DETECT_TASK_SIZE,					
                (void *)NULL,
				(uint16_t)DETECT_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);	

    xTaskCreate((TaskFunction_t)calibrate_task,
                (const char *)"CaliTask",
                (uint16_t)CALIBRATE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CALIBRATE_TASK_PRIO,
                (TaskHandle_t *)&CalibrateTask_Handler);
				
    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);

    xTaskCreate((TaskFunction_t)Sned_MainFoucs_Task,
                (const char *)"Send_MainFocusTask",
                (uint16_t)512,
                (void *)NULL,
                (UBaseType_t)4,
                (TaskHandle_t *)&ChassisTask_Handler);	
				
    xTaskCreate((TaskFunction_t)Send_ClientData_Task,
                (const char *)"Send_ClientData_Task",
                (uint16_t)512,
                (void *)NULL,
                (UBaseType_t)4,
                (TaskHandle_t *)&ChassisTask_Handler);				

    xTaskCreate((TaskFunction_t)DispchRefereeTask,
                (const char *)"DispchRefereeTask",
                (uint16_t)512,
                (void *)NULL,
                (UBaseType_t)5,
                (TaskHandle_t *)&ChassisTask_Handler);				

	  xTaskCreate((TaskFunction_t)UsartDebugTask,
                (const char *)"UsartDebugTask",
                (uint16_t)512,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&ChassisTask_Handler);
    vTaskDelete(Start_Task_Handle); //删除开始任务
				
    taskEXIT_CRITICAL();            //退出临界区
		
}

void StartTask()
{
    	xTaskCreate((TaskFunction_t)      start_task     , 
						 (const char *)   "start_task"   ,
						 (uint16_t)       START_STK_SIZE ,
						 (void *)         NULL           ,    
						 (UBaseType_t)    START_TASK_PRIO,	
						 (TaskHandle_t *) &Start_Task_Handle);			 
}

int8_t pitch;
void Sned_MainFoucs_Task(void *pvParameters)
{	
    for(;;)
	{	
		
	   Send_MessToMainFocus(0,GetEenmyColor());
	   vTaskDelay(100);
	   Send_MessToMainFocus(2,getPitchAngle() * 4);
	   vTaskDelay(100);
	}
}

void Send_ClientData_Task(void *parmas)
{
	TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(500);
	PreviousWakeTime = xTaskGetTickCount();
	for(;;)
	{
//		 Draw_grahic(0x0301, 0x0104, GetRobotID(),(GetRobotID()&0xff)|0x100);
//     Draw_grahic1(0x0301, 0x0104, GetRobotID(),(GetRobotID()&0xff)|0x100);
		 Draw_grahic_status(0x0301, 0x0104, GetRobotID(),(GetRobotID()&0xff)|0x100);
		 Draw_grahic_status_change(0x0301, 0x0104, GetRobotID(),(GetRobotID()&0xff)|0x100); 
		 Draw_grahic_char(0x0301, 0x0110, GetRobotID(),(GetRobotID()&0xff)|0x100);
//		 Draw_grahic_char_check(0x0301, 0x0110, GetRobotID(),(GetRobotID()&0xff)|0x100);
	   vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);
	}		
}

void UsartDebugTask(void *parmas)
{
	Chassis_Motor_t *chassis_mess;
	chassis_mess = getChassisGive_current();
	for(;;)
	{
		//printf("power:%f\tmaxout:%f\r\n",GetRealPower(),GetChassisMaxOutput()); 
	    printf(" %f  %f\tmaxout:%f  %d  %d  %d  %d\r\n",GetPowerBuffer(),GetRealPower(),GetChassisMaxOutput(),chassis_mess[0].give_current,chassis_mess[1].give_current,chassis_mess[2].give_current,chassis_mess[3].give_current);
		vTaskDelay(100);
	}
}

