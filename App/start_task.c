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

//TimerHandle_t	OneShotTimer_Handle;			//���ζ�ʱ�����
//void MotorRunTimeLimit(TimerHandle_t xTimer);		//���ζ�ʱ���ص�����

//TimerHandle_t	AutoAdjustStepOne_Handle;			//���ζ�ʱ�����
//void AutoAdjustStepOne(TimerHandle_t xTimer);		//���ζ�ʱ���ص�����

//TimerHandle_t	AutoAdjustStepTwo_Handle;			//���ζ�ʱ�����
//void AutoAdjustStepTwo(TimerHandle_t xTimer);		//���ζ�ʱ���ص�����

//TimerHandle_t	AutoAdjustStepThree_Handle;			//���ζ�ʱ�����
//void AutoAdjustStepThree(TimerHandle_t xTimer);		//���ζ�ʱ���ص�����

//TimerHandle_t	AutoAdjustStepFour_Handle;			//���ζ�ʱ�����
//void AutoAdjustStepFour(TimerHandle_t xTimer);		//���ζ�ʱ���ص�����

//QueueHandle_t HandCalibrateFlag;   		//��Ϣ���о��
//QueueHandle_t Message_Queue;	        //��Ϣ���о��
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
	
	 //������̨��������
	    xTaskCreate((TaskFunction_t)GIMBAL_task,
                (const char *)"GIMBAL_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GIMBALTask_Handler);

//								
//				    //�������ζ�ʱ��
//OneShotTimer_Handle =  xTimerCreate((const char*			)"MotorRunTimeLimit",
//											 (TickType_t			)50,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )1,
//											 (TimerCallbackFunction_t)MotorRunTimeLimit); //���ζ�ʱ��������2s(2000��ʱ�ӽ���)������ģʽ					  
//			
//											 
//											 
//AutoAdjustStepOne_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)100,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )2,
//											 (TimerCallbackFunction_t)AutoAdjustStepOne); //���ζ�ʱ��������2s(2000��ʱ�ӽ���)������ģʽ					  
//																					 
//AutoAdjustStepTwo_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)150,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )3,
//											 (TimerCallbackFunction_t)AutoAdjustStepTwo); //���ζ�ʱ��������2s(2000��ʱ�ӽ���)������ģʽ					  
//			
//AutoAdjustStepThree_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)200,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )4,
//											 (TimerCallbackFunction_t)AutoAdjustStepThree); //���ζ�ʱ��������2s(2000��ʱ�ӽ���)������ģʽ					  
//			
//AutoAdjustStepFour_Handle =  xTimerCreate((const char*			)"AutoAdjust",
//											 (TickType_t			)300,
//											 (UBaseType_t			)pdFALSE,
//											 (void*					  )5,
//											 (TimerCallbackFunction_t)AutoAdjustStepFour); //���ζ�ʱ��������2s(2000��ʱ�ӽ���)������ģʽ					  
//			
			 
											 
											 
		//������Ϣ����
//    HandCalibrateFlag = xQueueCreate(HandCalibrateLength,sizeof(int));        //������ϢKey_Queue

											 
//	xTimerStart(OneShotTimer_Handle,0);		//�������ζ�ʱ��			
			 //����ץ�ֿ�������
//   xTaskCreate((TaskFunction_t)Hand_Calibrate_Task,
//                (const char *)"Hand_Calibrate_Task",
//                (uint16_t)HAND_CALIBRATE_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)HAND_CALIBRATE_TASK_PRIO,
//                (TaskHandle_t *)&Hand_Calibrate_Task_Handler);						
												 
										 
//						xTimerStart(OneShotTimer_Handle,0);		//�������ζ�ʱ��			
//			 //����ץ�ֿ�������
//   xTaskCreate((TaskFunction_t)Hand_task,
//                (const char *)"Hand_task",
//                (uint16_t)Hand_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)Hand_TASK_PRIO,
//                (TaskHandle_t *)&HandTask_Handler);						
								
   //����mpu6500����				
    xTaskCreate((TaskFunction_t)INSTask,
                (const char *)"INSTask",
                (uint16_t)INS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);	  

	 //����ģ�����߼������	
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
    vTaskDelete(Start_Task_Handle); //ɾ����ʼ����
				
    taskEXIT_CRITICAL();            //�˳��ٽ���
		
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

