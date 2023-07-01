#ifndef _HAND_TASK_H
#define _HAND_TASK_H
#include "gimbal_task.h"
#include "main.h"
#include "pid.h"
#include "RemotDbus.h"
#include "MotorCAN.h"
#include "user_lib.h"
#include "Auto_attackTask.h"
#include "BasicPeripherals.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

////左边电机的pid
#define HAND_LINK_SPEED_PID_KP  MOTOR_3508_KP//2000.0f
#define HAND_LINK_SPEED_PID_KI 0.0f//20.0f
#define HAND_LINK_SPEED_PID_KD 0.0f
#define HAND_LINK_SPEED_PID_MAX_OUT   13000.0f//30000.0f
#define HAND_LINK_SPEED_PID_MAX_IOUT 0.0f//5000.0f

#define HAND_LINK_ENCODE_RELATIVE_PID_KP 8.0f
#define HAND_LINK_ENCODE_RELATIVE_PID_KI 0.00f
#define HAND_LINK_ENCODE_RELATIVE_PID_KD 0.0f
#define HAND_LINK_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define HAND_LINK_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define HAND_LINK_GYRO_ABSOLUTE_PID_KP 5.0f
#define HAND_LINK_GYRO_ABSOLUTE_PID_KI 0.0f
#define HAND_LINK_GYRO_ABSOLUTE_PID_KD 0.0f
#define HAND_LINK_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define HAND_LINK_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f


//左边电机的pid
#define LEFT_HAND_SPEED_PID_KP 1000.0f//2000.0f
#define LEFT_HAND_SPEED_PID_KI 0.0f//20.0f
#define LEFT_HAND_SPEED_PID_KD 0.0f
#define LEFT_HAND_SPEED_PID_MAX_OUT   MOTOR_3508_MAX_OUT//30000.0f
#define LEFT_HAND_SPEED_PID_MAX_IOUT 0.0f//5000.0f

#define LEFT_HAND_ENCODE_RELATIVE_PID_KP 6.0f//8.0f
#define LEFT_HAND_ENCODE_RELATIVE_PID_KI 0.00f
#define LEFT_HAND_ENCODE_RELATIVE_PID_KD 0.0f
#define LEFT_HAND_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define LEFT_HAND_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define LEFT_HAND_GYRO_ABSOLUTE_PID_KP 5.0f
#define LEFT_HAND_GYRO_ABSOLUTE_PID_KI 0.0f
#define LEFT_HAND_GYRO_ABSOLUTE_PID_KD 0.0f
#define LEFT_HAND_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define LEFT_HAND_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//右边电机的pid
#define RIGHT_HAND_SPEED_PID_KP  MOTOR_3508_KP//2000.0f
#define RIGHT_HAND_SPEED_PID_KI 150.0f//20.0f
#define RIGHT_HAND_SPEED_PID_KD 0.0f
#define RIGHT_HAND_SPEED_PID_MAX_OUT MOTOR_3508_MAX_OUT//30000.0f
#define RIGHT_HAND_SPEED_PID_MAX_IOUT 4000.0f//5000.0f

#define RIGHT_HAND_ENCODE_RELATIVE_PID_KP  6.0f//8.0f
#define RIGHT_HAND_ENCODE_RELATIVE_PID_KI 0.00f
#define RIGHT_HAND_ENCODE_RELATIVE_PID_KD 0.0f
#define RIGHT_HAND_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define RIGHT_HAND_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define RIGHT_HAND_GYRO_ABSOLUTE_PID_KP 5.0f
#define RIGHT_HAND_GYRO_ABSOLUTE_PID_KI 0.0f
#define RIGHT_HAND_GYRO_ABSOLUTE_PID_KD 0.0f
#define RIGHT_HAND_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define RIGHT_HAND_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

#define MOTOR_3508_KP       500.0f // 1500.0f
#define MOTOR_3508_MAX_OUT  9000.0f//16000.0f


#define RUN_VALUE  1024
//回到平衡位置的阈值（初始化时让抓手旋转回平衡位置）
#define CLOCKWISE_RUN_TO_BALANCE_TIMES         6000//1200//旋转一圈为2048
#define ANTICLOCKWISE_RUN_TO_BALANCE_TIMES    -6000//-600

//回到平衡位置的阈值（初始化时让抓手旋转回平衡位置）
#define HAND_CLOCKWISE_RUN_TO_BALANCE_TIMES         1500//1200//旋转一圈为2048
#define HAND_ANTICLOCKWISE_RUN_TO_BALANCE_TIMES    -600//-600
//3508电机是否反装
#define LEFT_TURN    0
#define RIGHT_TURN   0
#define HAND_LINK_TURN 0
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000


//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转

#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
//#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 16000
#define MAX_RELATIVE_ANGLE 1.57079632679f     //3.141592653589793      //6.283185307179586f
#define MIN_RELATIVE_ANGLE -1.57079632679f    //-3.141592653589793     //-6.283185307179586f
#define Motor_Ecd_to_ANGLE    0.222980217528f// 0.2223243935588f //差0.5度//0.227555555555555555f //将编码值转化为角度    8912/360/100 输入100转1度
#define Motor_Ecd_to_ANGLE_OFFSET  
//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

#define Motor_3508_Ecd_to_PI        0.0009765625f
#define Motor_3508_Ecd_to_ANGLE     0.0007669903943f
#define Motor_3508_Ecd_to_ANGLE_compensate   1.01000036f // 1.00999936f
//定义此结构体，方便将遥控器的同一拨杆值对应不同电机的模式
typedef enum
{
    HAND_MOTOR_RAW = 0, //电机原始值控制    对应云台的原始值控制 
	  HAND_MOTOR_SPEED,    //电机速度模式  gyro  单环速度控制
	  HAND_MOTOR_ENCONDE,  //电机编码值角度控制  对用云台的编码值控制
} hand_motor_mode_e;




typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Hand_PID_t;


typedef struct
{
   
	  const motor_measure_t *hand_motor_measure;
	  Hand_PID_t hand_motor_relative_angle_pid;
	  Hand_PID_t hand_motor_relative_speed_pid;
		PidTypeDef hand_motor_speed_pid;
	  PidTypeDef motor_2006_speed_pid;
	  fp32* add_angle;
	  gimbal_motor_mode_e hand_mode;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
    uint16_t offset_ecd; //云台电机中间指

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
	  fp32 speed_set;
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad
    fp32 motor_position_set;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	
} Hand_Motor_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
		Hand_Motor_t hand_left_motor;
		Hand_Motor_t hand_right_motor;
	  Hand_Motor_t hand_link_motor;
	
	  const external_encoder_t* hand_pitch_motor;
	
	const	fp32 *tow_hand_motors_add;
} Hand_Control_t;

//抓手关节平衡位置校准函数	
int16_t *get_link_motor_current(void);
void Cale_Hand_Link_to_Balance(void);
void Run_To_Balance_Init(void);  //抓手回归平衡位置
//void Cale_Run_To_Balance();  //按键校准
void Run_To_Balance(void);
void Clockwise_90(void); //顺时针旋转90度
void Clockwise_180(void); //顺时针旋转180度
void Anticlockwise_90(void); //逆时针旋转90度
void  Link_Motor_Self_lock(void);
//void EXTI2_IQRHandler(void);
void  Hand_Init_in_gimbal(void);
static fp32 Hand_PID_Calc(Hand_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void Hand_3508_Control_loop(Hand_Control_t *gimbal_control_loop);
static void Hand_Feedback_Update(Hand_Control_t *hand_motor_feedback_update);
double value_abs( double value);
extern void Hand_task(int16_t *left_hand_set_current, int16_t * right_hand_set_current, int16_t * hand_link_set_current);
void set_cali_key_hand_hook(const uint16_t left_offset, const uint16_t right_offset);
bool_t cmd_cali_hand_3508_hook( uint16_t *left_offset, uint16_t *right_offset );
#endif

