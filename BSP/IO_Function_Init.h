

void IO_Read_Init(void);
void IO_Out_Init(void);
void Set_Hand_Offset(void);


#define ON  1
#define OFF 0

/***************读取光电信号****************/
#define MOTOR1_MAX_P   GPIO_ReadInputDataBit(GPIOF ,GPIO_Pin_1)
#define MOTOR1_MIN_P   GPIO_ReadInputDataBit(GPIOE ,GPIO_Pin_5)

#define MOTOR2_MAX_P   GPIO_ReadInputDataBit(GPIOE ,GPIO_Pin_6)
#define MOTOR2_MIN_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_2)

#define MOTOR3_MAX_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_3)
#define MOTOR3_MIN_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_4)

#define MOTOR4_MAX_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_5)
#define MOTOR4_MIN_P   GPIO_ReadInputDataBit(GPIOA ,GPIO_Pin_5)



/***************控制电机方向****************/
#define MOTOR1_PUSH    GPIO_ResetBits(GPIOF, GPIO_Pin_0), GPIO_SetBits(GPIOE, GPIO_Pin_4);
#define MOTOR1_PULL    GPIO_SetBits(GPIOF, GPIO_Pin_0), GPIO_ResetBits(GPIOE, GPIO_Pin_4);

#define MOTOR2_PUSH    GPIO_ResetBits(GPIOE, GPIO_Pin_12), GPIO_SetBits(GPIOB, GPIO_Pin_8);
#define MOTOR2_PULL    GPIO_SetBits(GPIOE, GPIO_Pin_12), GPIO_ResetBits(GPIOB, GPIO_Pin_8);

#define MOTOR3_PUSH    GPIO_ResetBits(GPIOB, GPIO_Pin_1), GPIO_SetBits(GPIOC, GPIO_Pin_0);
#define MOTOR3_PULL    GPIO_SetBits(GPIOB, GPIO_Pin_1), GPIO_ResetBits(GPIOC, GPIO_Pin_0);

#define MOTOR4_PUSH    GPIO_ResetBits(GPIOC, GPIO_Pin_1), GPIO_SetBits(GPIOA, GPIO_Pin_4);
#define MOTOR4_PULL    GPIO_SetBits(GPIOC, GPIO_Pin_1), GPIO_ResetBits(GPIOA, GPIO_Pin_4);




////左前轮1
//#define LEFT_1_Backward      GPIO_SetBits(GPIOF,GPIO_Pin_1), GPIO_ResetBits(GPIOE,GPIO_Pin_5)
//#define LEFT_1_Forward		  GPIO_ResetBits(GPIOF,GPIO_Pin_1), GPIO_SetBits(GPIOE,GPIO_Pin_5)

////左后轮2
//#define LEFT_2_Forward      GPIO_SetBits(GPIOE,GPIO_Pin_6), GPIO_ResetBits(GPIOC,GPIO_Pin_2)
//#define LEFT_2_Backward		  GPIO_ResetBits(GPIOE,GPIO_Pin_6), GPIO_SetBits(GPIOC,GPIO_Pin_2)
////右前轮
//#define RIGHT_1_Forward     GPIO_SetBits(GPIOF,GPIO_Pin_0), GPIO_ResetBits(GPIOE,GPIO_Pin_4)
//#define RIGHT_1_Backward		GPIO_ResetBits(GPIOF,GPIO_Pin_0), GPIO_SetBits(GPIOE,GPIO_Pin_4)
////右后轮
//#define RIGHT_2_Backward     GPIO_SetBits(GPIOE,GPIO_Pin_12), GPIO_ResetBits(GPIOB,GPIO_Pin_1)
//#define RIGHT_2_Forward		   GPIO_ResetBits(GPIOE,GPIO_Pin_12), GPIO_SetBits(GPIOB,GPIO_Pin_1)


//#define LeftForward    	LEFT_1_Forward, LEFT_2_Forward
//#define LeftBackward   	LEFT_1_Backward, LEFT_2_Backward

//#define RightForward    RIGHT_1_Forward, RIGHT_2_Forward
//#define RightBackward   RIGHT_1_Backward, RIGHT_2_Backward


/*****************************用于读取外部反馈************************************/
//读取引脚状态函数 //PE引脚
#define DC_MOTOR_UP		          				GPIO_SetBits(GPIOE,GPIO_Pin_4),   GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define DC_MOTOR_DOWN   			          GPIO_ResetBits(GPIOE,GPIO_Pin_4), GPIO_SetBits(GPIOE,GPIO_Pin_5)	 

//救援抓手引脚
#define RESCUE_LEFT_UP 		   						GPIO_SetBits(GPIOE,GPIO_Pin_12) 
#define RESCUE_LEFT_DOWN 		   					GPIO_ResetBits(GPIOE,GPIO_Pin_12) 
#define RESCUE_RIGHT_UP 	   						GPIO_SetBits(GPIOE,GPIO_Pin_6)
#define RESCUE_RIGHT_DOWN 	   					GPIO_ResetBits(GPIOE,GPIO_Pin_6)



////读取引脚状态函数 //PA引脚
//#define  Hand_link_Level_Position		  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) 
//#define  Free_IO_2 		   						  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) 
//#define  Read_Step_Motor_State 	   		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取微动开关的状态
//#define  Read_Position_Min						GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)	
//#define  ReadShootBackHandState       GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14)	



//#define Read_Position_500mm				    GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_5)	
//#define Read_Position_700mm 					GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_6)	 
//#define Read_Position_710mm	    		  GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_7) 
//#define Read_Position_Max 						GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_2)
/*****************************用于控制外部元件************************************/
//气缸二级升降（一个引脚控制两个升降气缸）
#define 	CYLINDER_TO_DOWN				    GPIO_ResetBits(GPIOD, GPIO_Pin_13);//GPIO设置高
#define   CYLINDER_TO_UP 			   	    GPIO_SetBits(GPIOD, GPIO_Pin_13);//GPIO设置低
//气缸弹收抓手
#define 	TURN_BACK_HAND				 			GPIO_SetBits(GPIOD, GPIO_Pin_14);//GPIO设置高
#define 	SHOOT_OUT_HAND   			 			GPIO_ResetBits(GPIOD, GPIO_Pin_14);//GPIO设置低
//气缸张合抓手
#define   CLOSE_HAND		       			  GPIO_SetBits(GPIOD, GPIO_Pin_15);//GPIO设置高
#define 	OPEN_HAND   			     			GPIO_ResetBits(GPIOD, GPIO_Pin_15);//GPIO设置低
#define READ_HAND_OPEN_OR_CLOSE       GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15);//读取抓手状态
//控制步进电机方向引脚的函数在StepMotorControl.h内，使用引脚PI0

