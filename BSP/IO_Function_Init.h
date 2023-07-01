

void IO_Read_Init(void);
void IO_Out_Init(void);
void Set_Hand_Offset(void);


#define ON  1
#define OFF 0

/***************��ȡ����ź�****************/
#define MOTOR1_MAX_P   GPIO_ReadInputDataBit(GPIOF ,GPIO_Pin_1)
#define MOTOR1_MIN_P   GPIO_ReadInputDataBit(GPIOE ,GPIO_Pin_5)

#define MOTOR2_MAX_P   GPIO_ReadInputDataBit(GPIOE ,GPIO_Pin_6)
#define MOTOR2_MIN_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_2)

#define MOTOR3_MAX_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_3)
#define MOTOR3_MIN_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_4)

#define MOTOR4_MAX_P   GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_5)
#define MOTOR4_MIN_P   GPIO_ReadInputDataBit(GPIOA ,GPIO_Pin_5)



/***************���Ƶ������****************/
#define MOTOR1_PUSH    GPIO_ResetBits(GPIOF, GPIO_Pin_0), GPIO_SetBits(GPIOE, GPIO_Pin_4);
#define MOTOR1_PULL    GPIO_SetBits(GPIOF, GPIO_Pin_0), GPIO_ResetBits(GPIOE, GPIO_Pin_4);

#define MOTOR2_PUSH    GPIO_ResetBits(GPIOE, GPIO_Pin_12), GPIO_SetBits(GPIOB, GPIO_Pin_8);
#define MOTOR2_PULL    GPIO_SetBits(GPIOE, GPIO_Pin_12), GPIO_ResetBits(GPIOB, GPIO_Pin_8);

#define MOTOR3_PUSH    GPIO_ResetBits(GPIOB, GPIO_Pin_1), GPIO_SetBits(GPIOC, GPIO_Pin_0);
#define MOTOR3_PULL    GPIO_SetBits(GPIOB, GPIO_Pin_1), GPIO_ResetBits(GPIOC, GPIO_Pin_0);

#define MOTOR4_PUSH    GPIO_ResetBits(GPIOC, GPIO_Pin_1), GPIO_SetBits(GPIOA, GPIO_Pin_4);
#define MOTOR4_PULL    GPIO_SetBits(GPIOC, GPIO_Pin_1), GPIO_ResetBits(GPIOA, GPIO_Pin_4);




////��ǰ��1
//#define LEFT_1_Backward      GPIO_SetBits(GPIOF,GPIO_Pin_1), GPIO_ResetBits(GPIOE,GPIO_Pin_5)
//#define LEFT_1_Forward		  GPIO_ResetBits(GPIOF,GPIO_Pin_1), GPIO_SetBits(GPIOE,GPIO_Pin_5)

////�����2
//#define LEFT_2_Forward      GPIO_SetBits(GPIOE,GPIO_Pin_6), GPIO_ResetBits(GPIOC,GPIO_Pin_2)
//#define LEFT_2_Backward		  GPIO_ResetBits(GPIOE,GPIO_Pin_6), GPIO_SetBits(GPIOC,GPIO_Pin_2)
////��ǰ��
//#define RIGHT_1_Forward     GPIO_SetBits(GPIOF,GPIO_Pin_0), GPIO_ResetBits(GPIOE,GPIO_Pin_4)
//#define RIGHT_1_Backward		GPIO_ResetBits(GPIOF,GPIO_Pin_0), GPIO_SetBits(GPIOE,GPIO_Pin_4)
////�Һ���
//#define RIGHT_2_Backward     GPIO_SetBits(GPIOE,GPIO_Pin_12), GPIO_ResetBits(GPIOB,GPIO_Pin_1)
//#define RIGHT_2_Forward		   GPIO_ResetBits(GPIOE,GPIO_Pin_12), GPIO_SetBits(GPIOB,GPIO_Pin_1)


//#define LeftForward    	LEFT_1_Forward, LEFT_2_Forward
//#define LeftBackward   	LEFT_1_Backward, LEFT_2_Backward

//#define RightForward    RIGHT_1_Forward, RIGHT_2_Forward
//#define RightBackward   RIGHT_1_Backward, RIGHT_2_Backward


/*****************************���ڶ�ȡ�ⲿ����************************************/
//��ȡ����״̬���� //PE����
#define DC_MOTOR_UP		          				GPIO_SetBits(GPIOE,GPIO_Pin_4),   GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define DC_MOTOR_DOWN   			          GPIO_ResetBits(GPIOE,GPIO_Pin_4), GPIO_SetBits(GPIOE,GPIO_Pin_5)	 

//��Ԯץ������
#define RESCUE_LEFT_UP 		   						GPIO_SetBits(GPIOE,GPIO_Pin_12) 
#define RESCUE_LEFT_DOWN 		   					GPIO_ResetBits(GPIOE,GPIO_Pin_12) 
#define RESCUE_RIGHT_UP 	   						GPIO_SetBits(GPIOE,GPIO_Pin_6)
#define RESCUE_RIGHT_DOWN 	   					GPIO_ResetBits(GPIOE,GPIO_Pin_6)



////��ȡ����״̬���� //PA����
//#define  Hand_link_Level_Position		  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) 
//#define  Free_IO_2 		   						  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) 
//#define  Read_Step_Motor_State 	   		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ΢�����ص�״̬
//#define  Read_Position_Min						GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)	
//#define  ReadShootBackHandState       GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14)	



//#define Read_Position_500mm				    GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_5)	
//#define Read_Position_700mm 					GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_6)	 
//#define Read_Position_710mm	    		  GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_7) 
//#define Read_Position_Max 						GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_2)
/*****************************���ڿ����ⲿԪ��************************************/
//���׶���������һ�����ſ��������������ף�
#define 	CYLINDER_TO_DOWN				    GPIO_ResetBits(GPIOD, GPIO_Pin_13);//GPIO���ø�
#define   CYLINDER_TO_UP 			   	    GPIO_SetBits(GPIOD, GPIO_Pin_13);//GPIO���õ�
//���׵���ץ��
#define 	TURN_BACK_HAND				 			GPIO_SetBits(GPIOD, GPIO_Pin_14);//GPIO���ø�
#define 	SHOOT_OUT_HAND   			 			GPIO_ResetBits(GPIOD, GPIO_Pin_14);//GPIO���õ�
//�����ź�ץ��
#define   CLOSE_HAND		       			  GPIO_SetBits(GPIOD, GPIO_Pin_15);//GPIO���ø�
#define 	OPEN_HAND   			     			GPIO_ResetBits(GPIOD, GPIO_Pin_15);//GPIO���õ�
#define READ_HAND_OPEN_OR_CLOSE       GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15);//��ȡץ��״̬
//���Ʋ�������������ŵĺ�����StepMotorControl.h�ڣ�ʹ������PI0

