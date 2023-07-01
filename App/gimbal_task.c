#include "gimbal_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h" 
#include "INS_task.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "MotorCAN.h"
#include "shoot.h"
#include "StepMotorControl.h"

#include "start_task.h"
#include "queue.h"
#include "timers.h"
#include "usart6.h"
#include "led.h"
#include "FrictionMoterPWM.h"
#include "capture.h"
#include "IO_Function_Init.h"
#include "SBUS.h"



uint16_t while_loop = 0;

extern uint8_t RunToBalanceCaleFinish;
int  CaliBrateFinish = 56;

uint8_t low_speed_mode; //降低底盘速度的标志位
//急忙之下按错按键，1、需要关闭软件计时器   2、清除软件定时器的标志位
fp32 DC_motor_position = StepMotor_init_degreen;
extern uint8_t SecondCalebrate, end_turn;

extern int RunToBalanceCountFlag;


fp32 add_pitch_yaw = 100,relative_add = 30,speed_add = 2000;


static gimbal_motor_mode_e  hand_motor_mode; //定义抓手的结构体，用于获取云台的模式

//int16_t hand_link_motor_set[4] = {0, 0, 0, 0};

		//电机编码值规整 0—8191
		#define ECD_Format(ecd)     \
		{                           \
				if ((ecd) > ecd_range)  \
						(ecd) -= ecd_range; \
				else if ((ecd) < 0)     \
						(ecd) += ecd_range; \
		}

		#define gimbal_total_pid_clear(gimbal_clear)                                                   \
		{                                                                                          \
				Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
				Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
				PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
																																															 \
				Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
				Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
				PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
		}

static void J_scope_gimbal_test(void);
//云台控制所有相关数据
Gimbal_Control_t gimbal_control;
Position_t  position; //一级电机升降当前位置状态标志位
Engineer_variable engineer_data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		
Control_IO_State_t control_IO_state;//读取控制IO的状态
extern QueueHandle_t  HandCalibrateFlag;   		//消息队列句柄
	

	
/***********************自动调整统一标志位清零区***************************/
//	QR_Cude_is_up_count = 0;
//  QR_Cude_is_right_count = 0;
/******************************************************************************************************/

//3508电机pid控制模式
static void MOTOR_3508_Control_loop(Gimbal_Control_t *gimbal_control_loop);


//初始化云台 
 static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
 //初始化云台位置环PID参数
 static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
//将pid误差值，设定值，输出值清零
 static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
//更新云台控制参数
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//根据拨杆选择云台控制模式，再根据云台状态机选择电机状态机	
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);	
//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//云台控制状态使用不同控制pid
//static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);


//		 LED_GREEN = !LED_GREEN;  
//		 GIMBAL_Set_Mode(&gimbal_control);                    //设置云台控制模式
//		 GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡
//  	 GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
//		 GIMBAL_Set_Contorl(&gimbal_control);                 //设置云台控制量
//	   MOTOR_3508_Control_loop(&gimbal_control);            //3508控制PID计算
//		 Hand_Init_in_gimbal();
//		 Hand_task(&hand_motor_set[0], &hand_motor_set[1], &hand_motor_set[3]);



//extern  TaskHandle_t Hand_Calibrate_Task_Handler;   //测试
void control_motor(void);
static void Read_state_motor(void);
static void deal_speed(long long *input, float *output);
static void deal_Dir(long long *input, float *output);
//static void voctor_speed_dir(float *putout_left_speed, float *putout_right_speed);
//static void voctor_speed_dir();
	
uint16_t left_pwm = 0, right_pwm = 0, Run = 0;
uint8_t left_dir = 1 , right_dir = 1, start = 0;


 struct
{
	long long X;
	long long Y;
	long long Dir;
	long long Time;
	float Distance;
}CH_VAL;

 
struct{
  int8_t motor1_state;
	int8_t motor2_state;
	int8_t motor3_state;
	int8_t motor4_state;
} motor_state;

int PWM_speed = 50000;
int motor1 = 0, motor2 = 0, motor3 = 0, motor4 = 0;
int init = 0, x_cale = 0, y_cale = 0, start_cale = 0;
int tim8_1 = 10000, tim8_2 = 10000, tim8_3 = 10000, tim8_4 = 10000;
int ch_use_to_init = 0, distance_use_to_close = 0;


int led_i = 0, LED_gain = 4, led_light = 4000;
void GIMBAL_task(void *pvParameters)
{
		TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(GIMBAL_CONTROL_TIME);
	PreviousWakeTime = xTaskGetTickCount();
	TIM4_PWM_Init();
	TIM5_PWM_Init();
	TIM8_PWM_Init();
	TIM2_Cap_Init(0XFFFFFFFF,84-1);
	GIMBAL_Init(&gimbal_control);
	LED_GREEN = !LED_GREEN; 
	LED_RED = !LED_RED;
	IO_Read_Init();
	IO_Out_Init();

	while(1)  
	{
		LED_GREEN = !LED_GREEN; 
		LED_RED = !LED_RED;
//	vTaskDelay(15);
		Read_state_motor();
		Run++;
//	TIM_SetCompare1(TIM8, tim8_1);
//	TIM_SetCompare2(TIM8, tim8_2);
	
	if(led_i <= 0) led_i = led_light;
		
		TIM_SetCompare3(TIM8, led_i);
		TIM_SetCompare4(TIM8, led_i);
		led_i = led_i - LED_gain;
		
		   if(SBUS_CH[7] == 1) ch_use_to_init = 0; //复位
	else if(SBUS_CH[7] == 2 && ch_use_to_init == 0)
	{
		init = 0, motor1 = 0, motor2 = 0, motor3 = 0,motor4 = 0,start_cale = 0, x_cale = 0, y_cale = 0, ch_use_to_init = 1;
	}

		if(SBUS_CH[5] == 1) 
		{
			control_motor();
		}
		if(SBUS_CH[5] == 1)
		{
//			 if(SBUS_CH[4] == 1) motor1 = 0, motor2 = 0, motor3 = 0,motor4 = 0, PWM_speed = 0;
			  if(SBUS_CH[4] == 2) 
			{
				TIM_SetCompare1(TIM4, 0),TIM_SetCompare2(TIM4, 0),
				TIM_SetCompare3(TIM4, 0),TIM_SetCompare4(TIM4, 0),
				TIM_SetCompare1(TIM5, 0),TIM_SetCompare2(TIM5, 0),
				TIM_SetCompare3(TIM5, 0),TIM_SetCompare4(TIM5, 0);
			}
			else if(SBUS_CH[4] == 3)
			{
					 if(motor1 == 0) TIM_SetCompare1(TIM4, 0), TIM_SetCompare2(TIM4, 0);
			else if(motor1 == 1) TIM_SetCompare1(TIM4, PWM_speed), TIM_SetCompare2(TIM4, 0);
			else if(motor1 == 2) TIM_SetCompare1(TIM4, 0), TIM_SetCompare2(TIM4, PWM_speed);
			
					 if(motor2 == 0) TIM_SetCompare3(TIM4, 0), TIM_SetCompare4(TIM4, 0);
			else if(motor2 == 1) TIM_SetCompare3(TIM4, PWM_speed), TIM_SetCompare4(TIM4, 0);
			else if(motor2 == 2) TIM_SetCompare3(TIM4, 0), TIM_SetCompare4(TIM4, PWM_speed);

					 if(motor3 == 0) TIM_SetCompare1(TIM5, 0), TIM_SetCompare2(TIM5, 0);
			else if(motor3 == 1) TIM_SetCompare1(TIM5, 0), TIM_SetCompare2(TIM5, PWM_speed);
			else if(motor3 == 2) TIM_SetCompare1(TIM5, PWM_speed), TIM_SetCompare2(TIM5, 0);
			
					 if(motor4 == 0) TIM_SetCompare3(TIM5, 0), TIM_SetCompare4(TIM5, 0);
			else if(motor4 == 1) TIM_SetCompare3(TIM5, 0), TIM_SetCompare4(TIM5, PWM_speed);
			else if(motor4 == 2) TIM_SetCompare3(TIM5, PWM_speed), TIM_SetCompare4(TIM5, 0);
			}
	  }
		else if(SBUS_CH[5] == 2)//自定义调试，pwm设置速度50000
		{
			 if(SBUS_CH[4] == 1) motor1 = 0, motor2 = 0, motor3 = 0,motor4 = 0, PWM_speed = 0;
			 else if(SBUS_CH[4] == 2) 
			{
				TIM_SetCompare1(TIM4, 0),TIM_SetCompare2(TIM4, 0),
				TIM_SetCompare3(TIM4, 0),TIM_SetCompare4(TIM4, 0),
				TIM_SetCompare1(TIM5, 0),TIM_SetCompare2(TIM5, 0),
				TIM_SetCompare3(TIM5, 0),TIM_SetCompare4(TIM5, 0);
			}
			else if(SBUS_CH[4] == 3)
			{
					 if(motor1 == 0) TIM_SetCompare1(TIM4, 0), TIM_SetCompare2(TIM4, 0);
			else if(motor1 == 1) TIM_SetCompare1(TIM4, PWM_speed), TIM_SetCompare2(TIM4, 0);
			else if(motor1 == 2) TIM_SetCompare1(TIM4, 0), TIM_SetCompare2(TIM4, PWM_speed);
			
					 if(motor2 == 0) TIM_SetCompare3(TIM4, 0), TIM_SetCompare4(TIM4, 0);
			else if(motor2 == 1) TIM_SetCompare3(TIM4, PWM_speed), TIM_SetCompare4(TIM4, 0);
			else if(motor2 == 2) TIM_SetCompare3(TIM4, 0), TIM_SetCompare4(TIM4, PWM_speed);

					 if(motor3 == 0) TIM_SetCompare1(TIM5, 0), TIM_SetCompare2(TIM5, 0);
			else if(motor3 == 1) TIM_SetCompare1(TIM5, 0), TIM_SetCompare2(TIM5, PWM_speed);
			else if(motor3 == 2) TIM_SetCompare1(TIM5, PWM_speed), TIM_SetCompare2(TIM5, 0);
			
					 if(motor4 == 0) TIM_SetCompare3(TIM5, 0), TIM_SetCompare4(TIM5, 0);
			else if(motor4 == 1) TIM_SetCompare3(TIM5, 0), TIM_SetCompare4(TIM5, PWM_speed);
			else if(motor4 == 2) TIM_SetCompare3(TIM5, PWM_speed), TIM_SetCompare4(TIM5, 0);
			}
		}
							 if(motor1 == 0) TIM_SetCompare1(TIM4, 0), TIM_SetCompare2(TIM4, 0);
			else if(motor1 == 1) TIM_SetCompare1(TIM4, PWM_speed), TIM_SetCompare2(TIM4, 0);
			else if(motor1 == 2) TIM_SetCompare1(TIM4, 0), TIM_SetCompare2(TIM4, PWM_speed);
			
					 if(motor2 == 0) TIM_SetCompare3(TIM4, 0), TIM_SetCompare4(TIM4, 0);
			else if(motor2 == 1) TIM_SetCompare3(TIM4, PWM_speed), TIM_SetCompare4(TIM4, 0);
			else if(motor2 == 2) TIM_SetCompare3(TIM4, 0), TIM_SetCompare4(TIM4, PWM_speed);

					 if(motor3 == 0) TIM_SetCompare1(TIM5, 0), TIM_SetCompare2(TIM5, 0);
			else if(motor3 == 1) TIM_SetCompare1(TIM5, 0), TIM_SetCompare2(TIM5, PWM_speed);
			else if(motor3 == 2) TIM_SetCompare1(TIM5, PWM_speed), TIM_SetCompare2(TIM5, 0);
			
					 if(motor4 == 0) TIM_SetCompare3(TIM5, 0), TIM_SetCompare4(TIM5, 0);
			else if(motor4 == 1) TIM_SetCompare3(TIM5, 0), TIM_SetCompare4(TIM5, PWM_speed);
			else if(motor4 == 2) TIM_SetCompare3(TIM5, PWM_speed), TIM_SetCompare4(TIM5, 0);
		
//		     if(motor1 != 0) motor2 = 0, motor3 = 0,motor4 = 0;
//		else if(motor2 != 0) motor1 = 0, motor3 = 0,motor4 = 0;
//		else if(motor3 != 0) motor1 = 0, motor2 = 0,motor4 = 0;
//		else if(motor4 != 0) motor1 = 0, motor3 = 0,motor2 = 0;
		
//		TIM_SetCompare1(TIM5, PWM_speed);
//		TIM_SetCompare2(TIM5, PWM_speed);
//		TIM_SetCompare3(TIM5, PWM_speed);
//		TIM_SetCompare4(TIM5, PWM_speed);	

		GetCaptureVal(&CH_VAL.Time,&CH_VAL.X, &CH_VAL.Y, &CH_VAL.Dir);
		CH_VAL.Distance = CH_VAL.Time*SOUND_VELOCITY/2.0f/1000000.0f;
		if(CH_VAL.Distance > 0.20 && CH_VAL.Distance < 0.40)
		   distance_use_to_close = 1;
		else distance_use_to_close = 0;
		
		
   vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);
	}
	
}

int i = 0 ,count = 1000;
void control_motor(void)
{
	if(motor_state.motor1_state == -2 || motor_state.motor2_state == -2 || motor_state.motor3_state == -2 || motor_state.motor4_state == -2)
	{	
		motor1 = 0, motor2 = 0, motor3 = 0,motor4 = 0, PWM_speed = 0;
		return; 
	}
	
	if(init == 0 && (motor_state.motor1_state != 2 
		|| motor_state.motor2_state != 2 
		|| motor_state.motor3_state != 2 
		|| motor_state.motor4_state != 2 ))
	{
		motor1 = 2, motor2 = 2, motor3 = 2, motor4 = 2, init = 1, start_cale = 0, x_cale = 0, y_cale = 0;
	}
	
	if(init == 1)
	{
		if(motor_state.motor1_state == 2) motor1 = 0;
		if(motor_state.motor2_state == 2) motor2 = 0;
		if(motor_state.motor3_state == 2) motor3 = 0;
		if(motor_state.motor4_state == 2) motor4 = 0;
	}
	
	if(motor1 == 0 && motor2 == 0 && motor3 == 0 && motor4 == 0)
	{
		motor1 = 0, motor2 = 0, motor3 = 0, motor4 = 0;
		x_cale = 0, y_cale = 0, init = 2;
	}
	
	 if((distance_use_to_close == 1 || SBUS_CH[4] == 1)
		 && motor_state.motor1_state == 2 
		 && motor_state.motor2_state == 2 
	   && motor_state.motor3_state == 2 
	   && motor_state.motor4_state == 2 
	   && init == 2)
		 start_cale = 1, x_cale = 0, y_cale = 0;
	
	 /******************x方向校准***********************/
	if(start_cale == 1)
	{
		motor1 = 1, motor2 = 1, motor3 = 0, motor4 = 0, start_cale = 2;
	}
	else if(start_cale == 2)
	{
		if(motor_state.motor1_state == 1) motor1 = 0;
		if(motor_state.motor2_state == 1) motor2 = 0;
		if(motor1 == 0 && motor2 == 0)  x_cale = 1, start_cale = 3;
	}
		 /****************y方向校准************************/
	if(start_cale == 3) 
	{
		if(i < count)
		{
			motor1 = 2, motor2 = 2, motor3 = 0, motor4 = 0;
			i++;
		}
		else  start_cale = 4;
	}
	else if(start_cale == 4)
	{
		motor1 = 0, motor2 = 0, motor3 = 1, motor4 = 1, start_cale = 5;
	}
	else if(start_cale == 5)
	{
		if(motor_state.motor3_state == 1) motor3 = 0;
		if(motor_state.motor4_state == 1) motor4 = 0;
		if(motor3 == 0 && motor4 == 0)	y_cale = 1, start_cale = 6;
	}
	/***************二次校准*******************/
	else if(start_cale == 6)
	{
		motor1 = 1, motor2 = 1, motor3 = 0, motor4 = 0, start_cale = 7;
	}
	else if(start_cale == 7)
	{
		if(motor_state.motor1_state == 1) motor1 = 0;
		if(motor_state.motor2_state == 1) motor2 = 0;
		if(motor1 == 0 && motor2 == 0) start_cale = 0;
	}
	
	
	
}


void Read_state_motor(void)
{
			 if(MOTOR1_MAX_P == OFF && MOTOR1_MIN_P == OFF)  motor_state.motor1_state = 0;
	else if(MOTOR1_MAX_P == OFF && MOTOR1_MIN_P == ON)   motor_state.motor1_state = 1;
	else if(MOTOR1_MAX_P == ON  && MOTOR1_MIN_P == OFF)  motor_state.motor1_state = 2;
	else motor_state.motor1_state = -2;

			 if(MOTOR2_MAX_P == OFF && MOTOR2_MIN_P == OFF)  motor_state.motor2_state = 0;
	else if(MOTOR2_MAX_P == OFF && MOTOR2_MIN_P == ON)   motor_state.motor2_state = 1;
	else if(MOTOR2_MAX_P == ON  && MOTOR2_MIN_P == OFF)  motor_state.motor2_state = 2;
	else motor_state.motor2_state = -2;

			 if(MOTOR3_MAX_P == OFF && MOTOR3_MIN_P == OFF)  motor_state.motor3_state = 0;
	else if(MOTOR3_MAX_P == OFF && MOTOR3_MIN_P == ON)   motor_state.motor3_state = 1;
	else if(MOTOR3_MAX_P == ON  && MOTOR3_MIN_P == OFF)  motor_state.motor3_state = 2;
	else motor_state.motor3_state = -2;

			 if(MOTOR4_MAX_P == OFF && MOTOR4_MIN_P == OFF)  motor_state.motor4_state = 0;
	else if(MOTOR4_MAX_P == OFF && MOTOR4_MIN_P == ON)   motor_state.motor4_state = 1;
	else if(MOTOR4_MAX_P == ON  && MOTOR4_MIN_P == OFF)  motor_state.motor4_state = 2;
	else motor_state.motor4_state = -2;
	
}


int left_speed = 0, right_speed = 0, error = 13000, basic_vol = 7000;
float speed = 0, dir = 0;
float chassis_speed = 0, chassis_dir = 0, chassis_level = 0;
static void voctor_speed_dir()
{

		deal_speed(&CH_VAL.X, &speed);
		deal_Dir(&CH_VAL.Dir, &dir);
	  chassis_speed = speed;
	  chassis_dir = dir;
	
	left_speed = speed*error + dir*error;
	right_speed = speed*error - dir*error;

	
	if(left_speed >= 0)
	{	}
	else
	{	left_speed = -left_speed;}
	
	if(right_speed >= 0)
	{	;	}
	else
	{	right_speed = -right_speed;}
	
	
	left_speed = basic_vol + (uint32_t)(left_speed);
	right_speed = basic_vol + (uint32_t)(right_speed);
	
  if(speed == 0 && dir == 0)
		left_speed = 0, right_speed = 0;
	
//	*putout_left_speed = left_speed;
//	*putout_right_speed = right_speed;	
	
	
		TIM_SetCompare1(TIM4, right_speed);
		TIM_SetCompare2(TIM4, right_speed);
		TIM_SetCompare3(TIM4, left_speed);
		TIM_SetCompare4(TIM4, left_speed);
}

static void deal_speed(long long  *input, float *output)
{
	int error = 0;
	error = *input - 1616;
	if(error < 20 && error > -20)
		error = 0;
  *output = error/447.5;
}

static void deal_Dir(long long  *input, float *output)
{
	int error = 0;
	error = *input - 1596;
	if(error < 20 && error > -20)
		error = 0;
  *output = error/445.0;

}


static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static void motor_3508_speed_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

//		gimbal_motor->motor_gyro_set =  GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, 
//		gimbal_motor->relative_angle, gimbal_motor->speed_3508_motor_set,  0 );
			
		gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_speed_pid, 
		gimbal_motor->gimbal_motor_measure->speed_rpm*RC_Send, gimbal_motor->speed_3508_motor_set);
			
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //3508位置环，速度环串级pid调试
    gimbal_motor->motor_gyro_set =  GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, 
		gimbal_motor->relative_angle, gimbal_motor->relative_angle_set,  0 );
    //3508速度环
		gimbal_motor->current_set = PID_Calc(	&gimbal_motor->gimbal_motor_gyro_pid, 
		gimbal_motor->gimbal_motor_measure->speed_rpm*UNIT_CHANGE, gimbal_motor->motor_gyro_set );
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
extern 	uint8_t getFireMode();


static void MOTOR_3508_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
	    if (gimbal_control_loop == NULL)
    {
        return;
    }
	gimbal_control_loop->gimbal_yaw_motor.null_redifine = 0;
	hand_motor_mode = gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode; 
  //yaw不同模式对于不同的控制函数
	   if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde角度控制zx
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
		 else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED && (position.Position_Max==0||position.Position_500mm==0||position.Position_700mm==0||position.Position_Min==0))
    {
        //gyro角度控制
       motor_3508_speed_control(&gimbal_control_loop->gimbal_yaw_motor);
			
    }
		//    //yaw不同模式对于不同的控制函数
   else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
		
		   //pitch不同模式对于不同的控制函数
     if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //gyro角度控制
       gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);

    }
		 else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED && (position.Position_Max==0||position.Position_500mm==0||position.Position_700mm==0||position.Position_Min==0))
    {
        //gyro角度控制
         motor_3508_speed_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
		  else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
}

static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
   gimbal_motor->relative_angle_set = add*relative_add;
}

static void MOTOR_3508_speed_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
     gimbal_motor->speed_3508_motor_set = 0;//add*speed_add;
}


//云台控制量设置
uint8_t errors=0;
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
		if(gimbal_set_control->gimbal_rc_ctrl->rc.ch[2]==0 && gimbal_set_control->gimbal_rc_ctrl->mouse.x==0)
		{
			errors=1;
		}		
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
		gimbal_set_control->gimbal_yaw_motor.add_left_angle      = add_yaw_angle;
		gimbal_set_control->gimbal_pitch_motor.add_right_angle  = add_pitch_angle;
    //yaw电机模式控制
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED)
    {
        //gyro模式下，陀螺仪角度控制
			  MOTOR_3508_speed_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle );
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
			 GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor,  add_yaw_angle );
    }

    //pitch电机模式控制
    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED)
    {
        //gyro模式下，陀螺仪角度控制
			 MOTOR_3508_speed_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
//    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_SPEED && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED)
//    {
//        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
//    }
		    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_SPEED && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED)
    {
        gimbal_mode_change->gimbal_yaw_motor.speed_3508_motor_set = 0;//gimbal_mode_change->gimbal_yaw_motor.speed_3508_motor;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
//    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_SPEED && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED)
//    {
//        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
//    }
		    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_SPEED && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_SPEED)
    {
        gimbal_mode_change->gimbal_pitch_motor.speed_3508_motor_set = 0;//gimbal_mode_change->gimbal_pitch_motor.speed_3508_motor;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
//    gimbal_set_mode->gimbal_rc_ctrl->key.v 
    if (gimbal_set_mode == NULL)
    {
        return;
    }
//		if(OneShotTimer_Handle!=NULL)  {};

    gimbal_behaviour_mode_set(gimbal_set_mode);
}


//初始化pid 数据指针
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{

    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	
    //电机数据指针获取		
    gimbal_init->gimbal_yaw_motor.step_ecd = RealDegree_point();  
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point(); 
    //陀螺仪数据指针获取
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

		//初始化pitch电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_speed_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);


    //清除所有PID
    gimbal_total_pid_clear(gimbal_init);
    //更新云台反馈数据
    GIMBAL_Feedback_Update(gimbal_init);
		//初始化抓手，使抓手回到平衡位置
//	  Run_To_Balance_Init();
	  //初始化妙算数据
	  gimbal_init->autodata = GetAutoDataPoint();
	  //初始化升降3508电机
			gimbal_control.gimbal_yaw_motor.speed_3508_motor_set   =  0;
      gimbal_control.gimbal_pitch_motor.speed_3508_motor_set =  0;

//    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
		gimbal_init->gimbal_yaw_motor.speed_3508_motor_set = gimbal_init->gimbal_yaw_motor.speed_3508_motor;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;


//    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
	  gimbal_init->gimbal_pitch_motor.speed_3508_motor_set = gimbal_init->gimbal_pitch_motor.speed_3508_motor;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

	double value_abs( double value)
	{
  if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
	
	}
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Step_Ecd_to_Degree;
}


//int current_ecd = 0, last_ecd = 0, ecd_flag = 0, error_ecd = 0, error_flag = 0;
//fp64 relative_ecd;	
//double motor_3508_speed_rpm;
//static fp64 motor_3508_ecd_to_angle_change(uint16_t ecd, double motor_speed)
//{

////	Gimbal_Motor_t *motor_speed;
// 	motor_3508_speed_rpm = motor_speed;
//	current_ecd = ecd;
//	if (ecd_flag == 0) 
//	{
//		last_ecd = current_ecd;
//		 ecd_flag = 1;
//	}
//	error_flag = current_ecd - last_ecd;
//	if(error_flag > 100 || error_flag < -100 ) 
//	{
//		 error_ecd  = current_ecd - last_ecd;
////		if(motor_3508_speed_rpm > 0)
////		{
////	          error_ecd =	value_abs( error_ecd );
////		}
////	   else  	error_ecd =	-value_abs( error_ecd );	
//		 last_ecd   = current_ecd;
//	}
//	else error_ecd = 0;
////问题：转速  转的过快，编码值会反着减小
////电机正传：电机采样时间内 慢于一圈，8192-error	
//    relative_ecd +=  error_ecd* Motor_Ecd_to_Rad;
////	  last_ecd = current_ecd;
// return relative_ecd;  
//}




static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.speed_3508_motor = 0;// -*(gimbal_feedback_update->gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = 0; //  *(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);

    gimbal_feedback_update->gimbal_yaw_motor.speed_3508_motor = 0;// (gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(*gimbal_feedback_update->gimbal_yaw_motor.step_ecd,
                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd );

    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = 0; //-*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);

	//-(arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET))\
                                                            - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)));
}


//校准计算，相对最大角度，云台中值
static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN


    temp_ecd = gimbal_cali->max_pitch_ecd - gimbal_cali->min_pitch_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_pitch_ecd - (temp_ecd / 2);
	
    ECD_Format(temp_ecd);
    *pitch_offset = temp_ecd;	
    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#else

    temp_ecd = gimbal_cali->max_pitch_ecd - gimbal_cali->min_pitch_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_pitch_ecd - (temp_ecd / 2);
	
    ECD_Format(temp_ecd);
    *pitch_offset = temp_ecd;	
    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *yaw_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *yaw_offset);


#endif
}

/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @author         RM
  * @param[in]      yaw 中值
  * @param[in]      pitch 中值
  * @param[in]      yaw 最大相对角度
  * @param[in]      yaw 最小相对角度
  * @param[in]      pitch 最大相对角度
  * @param[in]      pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
  * @brief          云台校准计算，将校准记录的最大 最小值 来计算云台 中值和最大最小机械角度
  * @author         RM
  * @param[in]      yaw 中值 指针
  * @param[in]      pitch 中值 指针
  * @param[in]      yaw 最大相对角度 指针
  * @param[in]      yaw 最小相对角度 指针
  * @param[in]      pitch 最大相对角度 指针
  * @param[in]      pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch     = gimbal_control.gimbal_pitch_motor.speed_3508_motor;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw 			 = gimbal_control.gimbal_yaw_motor.speed_3508_motor;
        gimbal_control.gimbal_cali.max_yaw_ecd	 = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
			
        gimbal_control.gimbal_cali.min_pitch     = gimbal_control.gimbal_pitch_motor.speed_3508_motor;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw       = gimbal_control.gimbal_yaw_motor.speed_3508_motor;
        gimbal_control.gimbal_cali.min_yaw_ecd   = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        gimbal_control.gimbal_yaw_motor.offset_ecd           = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle   = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle   = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd         = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;

        return 1;
    }  
    else
    {
        return 0;
    }
}


const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

const gimbal_motor_mode_e *get_yaw_motor_mode_point(void)
{
    return &gimbal_control.gimbal_yaw_motor.gimbal_motor_mode;
}

const gimbal_motor_mode_e *get_pitch_motor_mode_point(void)
{
    return &gimbal_control.gimbal_pitch_motor.gimbal_motor_mode;
}

 fp32* get_add_yaw_angle_pointer(void)
{
    return &gimbal_control.gimbal_yaw_motor.add_left_angle;
}
 fp32* get_add_pitch_angle_pointer(void)
{
    return &gimbal_control.gimbal_pitch_motor.add_right_angle;
}


int8_t getPitchAngle()
{
   return   (int8_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 180.0f / 3.14f);
}
int8_t getEenmyColor()
{
   static char  switch_enemy_color = 0;
   if((gimbal_control.gimbal_rc_ctrl->key.v & SwitchEnemyColor_Red_KeyBoard) !=  0  && switch_enemy_color == 1)
   {
       switch_enemy_color = 0;
   }
   else if((gimbal_control.gimbal_rc_ctrl->key.v & SwitchEnemyColor_Blue_KeyBoard) !=  0 && switch_enemy_color == 0)
   {
       switch_enemy_color = 1;
   }
   return switch_enemy_color;
}





/******************
void start_one_shot_time();
开启操作系统定时器，
当按键按下后计时，
防止电机一直旋转，
破坏机械结构或卡死烧坏电机
******************/





const fp32 *send_hand_motor_set(void)
{
     return &gimbal_control.transmit_cube_motor_set;
}

void clear_flag_tutal()
{
 engineer_data.Motion.engineer_ready_state = 0;
 engineer_data.Motion.ready_catch_small_cube = 0;
 engineer_data.Motion.get_cube_success = 0;
 engineer_data.Motion.ready_catch_big_cube = 0;
 engineer_data.Motion.catch_cube_on_the_ground = 0;
 engineer_data.Motion.open_close_hand = 0;	
}




	u8 *retur_hand_to_balance(void)
{
    return &engineer_data.hand_to_balance;

}
uint8_t *return_low_speed_mode(void)
{
  return &low_speed_mode;
}
fp32 *return_DC_motor_position_set(void)
{
  return &DC_motor_position;
}

u8 *DC_motor_stop_flag(void)
{
  return &engineer_data.DC_motor_motionless;
}

u8 *get_official_check_flag(void)
{
    return &engineer_data.official_check;
}



