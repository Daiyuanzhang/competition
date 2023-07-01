//#include "Hand_Calibrate.h"
//#include "gimbal_task.h"
//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "user_lib.h"
//#include "shoot.h"
//#include "StepMotorControl.h"
//#include "IO_Function_Init.h"
//#include "queue.h"
//#include "start_task.h"
//#include "hand_task.h"
//int hand_cali = 0;
//extern uint8_t RunToBalanceCaleFinish;
//int err, CaliBrateFinish = 56;
//extern	QueueHandle_t HandCalibrateFlag;   		//消息队列句柄
//extern  TaskHandle_t Hand_Calibrate_Task_Handler;
//extern void Hand_Calibrate_Task(void *pvParameters)
//	{
////		 vTaskDelay(200);
//		while(1)
//		{
//			  Cale_Hand_Link_to_Balance();
//	      Run_To_Balance_Init();  //初始化时让抓手回归平衡位置
//				CaliBrateFinish++;
////				err=xQueueSend(HandCalibrateFlag,&CaliBrateFinish,10);
////				if(err==errQUEUE_FULL)   	//发送校准完成标志位
////				{
////					printf("队列已满，数据发送失败!\r\n");
////				}
//				hand_cali++;
//			 // if(CaliBrateFinish == 500)	  vTaskSuspend(Hand_Calibrate_Task_Handler);
//				if(RunToBalanceCaleFinish == 1)	  vTaskSuspend(Hand_Calibrate_Task_Handler);
//		}
//	}






































