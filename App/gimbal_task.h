#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include "pid.h"
#include "RemotDbus.h"
#include "MotorCAN.h"
#include "user_lib.h"
#include "Auto_attackTask.h"


#define  SOUND_VELOCITY  342.62f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP          GIMBAL_MOTOR_3508_KP//2000.0f
#define PITCH_SPEED_PID_KI 0.0f//20.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT     GIMBAL_MOTOR_3508_MAX_OUT//30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 0.0f//5000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 5.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f


//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 8.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f


//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP   GIMBAL_MOTOR_3508_KP
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT GIMBAL_MOTOR_3508_MAX_OUT
#define YAW_SPEED_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 5.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 8.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define  GIMBAL_MOTOR_3508_KP          1000.0f//1500.0f
#define	 GIMBAL_MOTOR_3508_MAX_OUT     12000.0f//16000.0f

#define  ADD_SET   1000
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

//yaw,pitch控制通道以及状态开关通道
#define YawChannel   2
#define PitchChannel 3
#define ModeChannel  0

//掉头180 按键
#define TurnKeyBoard KEY_PRESSED_OFFSET_F
//掉头云台速度
#define TurnSpeed 0.04f

//测试按键尚未使用
#define TestKeyBoard KEY_PRESSED_OFFSET_R

//辅助射击按键
#define VisonONKeyBoard KEY_PRESSED_OFFSET_Q
#define VisonOFFKeyBoard KEY_PRESSED_OFFSET_E

#define SwitchEnemyColor_Red_KeyBoard KEY_PRESSED_OFFSET_R
#define SwitchEnemyColor_Blue_KeyBoard KEY_PRESSED_OFFSET_G


//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10

//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN   -0.000002f
#define Pitch_RC_SEN -0.000006f //0.005

//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen   0.00006f
#define Pitch_Mouse_Sen 0.00002f

//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen   0.01f
#define Pitch_Encoder_Sen 0.01f

//云台控制周期
#define GIMBAL_CONTROL_TIME 1

//视觉给定的yaw数据太大，导致底盘移动过快，摄像头丢失目标
#define   CHASSIS_FLLOW_YAW_RATE   0.2f
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否反装
#define PITCH_TURN   0
#define YAW_TURN     0

#define OpenErrorCheck    0

#define Motor_Drive       0
#define Motor_Self_Lock   1

#define Hand_In_Level     0
#define Hand_In_Down      1
//气缸电磁阀气流方向
#define CYLINDER_SWITCH_ON    1   // cylinder 
#define CYLINDER_SWITCH_OFF   0

#define HandOpen     0
#define HandClose    1  

#define StepMotorRunTime 500
#define  MIN_COUNT_DELAY 500  //gimbal_task 循环两千次来做延时，防止电机在拨杆开始其他模式时暴走


////3508电机是否反装
//#define LEFT_TURN 0
//#define RIGHT_TURN 0

//电机码盘值最大以及中值
#define Half_ecd_range 32768
#define ecd_range 65535

//3508电机RPM转化为标准单位的比率
#define  UNIT_CHANGE   0.0093f     //测量时0.01为最大的值 0.0093比较稳定（这是一个偶然值）
#define  RC_Send       0.001f//0.00097f     
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000


//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET   0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET         0      //8000 
#define GIMBAL_CALI_PITCH_MOTOR_SET   0      //8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 16000
#define MAX_RELATIVE_ANGLE 1.57079632679f     //3.141592653589793      //6.283185307179586f
#define MIN_RELATIVE_ANGLE -1.57079632679f    //-3.141592653589793     //-6.283185307179586f
#define Motor_Ecd_to_ANGLE    0.222980217528f// 0.2223243935588f //差0.5度//0.227555555555555555f //将编码值转化为角度    8912/360/100 输入100转1度
#define Motor_Ecd_to_ANGLE_COMPENSATE 1.02055556f
#define Motor_Ecd_to_ANGLE_OFFSET  
//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad   0.000766990394f //      2*  PI  /8192
#define Step_Ecd_to_Degree 0.0054931640625f //      360°/65536


#define portMAX_DELAY   5  //任务阻塞时间
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_SPEED,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef enum
{
    QR_Cude_IS_UP = 0,   //二维码在上面
    QR_Cude_IS_RIGHT = 1,    //二维码在右边
    QR_Cude_IS_DOWN,     //二维码在下方
	  QR_Cude_IS_LEFT,     //二维码在左边
	  QR_Cude_IS_FRONT,     // 二维码在正前
	  QR_Cude_IS_BACK,     // 二维码在后面
} cube_state_e;


typedef  struct
{
		/************************自动调整需要定义的变量***************************/
		uint32_t QR_Cude_is_up_count ;
		uint32_t QR_Cude_is_right_count ;
		uint32_t QR_Cude_is_left_count ;
		uint32_t QR_Cude_is_front_count ;
		uint32_t QR_Cude_is_down_count ;
		uint32_t QR_Cude_is_back_count ;
		uint32_t shoot_back_delay;
		uint32_t step_motor_init_delay;
		uint32_t hand_init_finish;
} QR_Code_flag;

typedef struct
{
		u8 engineer_ready_state ;
		u8 ready_catch_small_cube ;
		u8 catch_cube_up ;
		u8 get_cube_success ;
		u8 ready_catch_big_cube ;
		u8 catch_cube_on_the_ground ;
		u8 open_close_hand ;
} Engineer_motion;

typedef struct
{
   u8 drag_robots_lift_hand;
	 u8 drag_robots_right_hand;
   u8 rescue_robots_by_Card;
} Rescue_Robots_t;

typedef  struct
{
		/************************自动调整需要定义的变量***************************/
		u8 qr_code_is_up;
		u8 qr_code_is_right;
		u8 qr_code_is_left;
		u8 qr_code_is_front;
		u8 qr_code_is_down;
		u8 qr_code_is_back;
		u8 qr_code_test ;
} QR_Code_direction;

typedef struct
{
	  Engineer_motion   Motion;
		QR_Code_direction Dir;
		QR_Code_flag      QR_Cude_count;
	   Rescue_Robots_t  Rescue_Robots;
/***************************************工程动作标志位************************************************/		
    u8 position_init;
	  u8 DC_motor_motionless;
		u8 hand_to_down_and_level ;
		u8 shoot_and_back_hand ; 
		u8 cylinder_up_and_down ;       
		//抓手旋转
	  u8 cale_hand_to_balance  ;      //抓手在平衡位置
		u8 hand_to_balance  ;      //抓手在平衡位置
		u8 hand_to_90   ;           //抓手顺时针旋转90度		
		u8 hand_to_180  ;          //抓手顺时针旋转180度
		u8 hand_to_minus_90 ;      //抓手逆时针旋转90度		
		//一级电机升降
		u8 position_min ;          
		u8 position_500 ;       
		u8 position_700 ;
		u8 position_710 ;
		u8 position_max ;
		//抓手俯仰
		u8 hand_go_down ;
		u8 hand_go_level ;

		u8 auto_adjust_cube;
		u8 get_cube_in_out;
		u8 official_check;  //用于检录 抓手下降到一个较大角度
		u8 get_cube_angle_set;
		uint32_t count_delay;
} Engineer_variable;
	


typedef  struct
{
	char read_hand_open_or_close ;

} Control_IO_State_t;//一级电机升降当前位置状态标志位

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
} Gimbal_PID_t;

typedef struct
{
	  uint16_t *step_ecd;
    const motor_measure_t *gimbal_motor_measure;
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;
    Gimbal_PID_t gimbal_motor_relative_angle_pid;

    PidTypeDef gimbal_motor_gyro_pid;
	  PidTypeDef gimbal_motor_speed_pid;
    gimbal_motor_mode_e gimbal_motor_mode; //云台电机模式状态机
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd; //云台电机中间指
	
	  fp32 add_left_angle;
    fp32 add_right_angle;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
    fp32 absolute_angle;
	
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
	  fp32 speed_3508_motor;
	  fp32 speed_3508_motor_set;
//    fp32 absolute_angle;     //rad
//    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;	
		fp32 null_redifine;
} Gimbal_Motor_t;


typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
	
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
    Gimbal_Cali_t gimbal_cali;
	  AUTODATA *autodata;
	  char auto_aim_flag;
	  fp32 transmit_cube_motor_set;
} Gimbal_Control_t;

typedef  struct
{
	uint8_t Position_Min ;
	uint8_t Position_500mm ;
	uint8_t Position_700mm ;
	uint8_t Position_710mm ;	
	uint8_t Position_Max ;
	uint8_t Read_Step_Motor_IO;
	uint8_t Hand_Link_To_Balance;
} Position_t;//一级电机升降当前位置状态标志位



uint8_t *return_low_speed_mode(void);//降低底盘速度的标志位
fp32 *return_DC_motor_position_set(void);


void clear_flag_tutal(void);
/**************************************/

u8 *DC_motor_stop_flag(void);

void  start_one_shot_time(void); //开启操作系统定时器，当启动一级升降后计时，防止电机一直旋转，破坏机械结构或卡死烧坏电机
void  Run_To_Position_Min(void);
void  Run_To_Position_500mm(void);
void  Run_To_Position_700mm(void);
void  Run_To_Position_710mm(void);
void  Run_To_Position_Max(void);
u8 *retur_hand_to_balance(void);
const fp32 *send_hand_motor_set(void);
void  Motor_Speed_Down_Control(void);
void  Motor_Speed_Up_Control(void);
void  Motor_Self_lock(void);
double value_abs( double value);
//static void motor_3508_speed_control(Gimbal_Motor_t *gimbal_motor);
//static void MOTOR_3508_speed_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
extern int8_t getPitchAngle(void);
extern int8_t getEenmyColor(void);
extern const gimbal_motor_mode_e *get_yaw_motor_mode_point(void);
extern const gimbal_motor_mode_e *get_pitch_motor_mode_point(void);
 fp32* get_add_yaw_angle_pointer(void);
 fp32* get_add_pitch_angle_pointer(void);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
extern void GIMBAL_task(void *pvParameters);
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
#endif

