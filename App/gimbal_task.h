#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include "pid.h"
#include "RemotDbus.h"
#include "MotorCAN.h"
#include "user_lib.h"
#include "Auto_attackTask.h"


#define  SOUND_VELOCITY  342.62f

//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP          GIMBAL_MOTOR_3508_KP//2000.0f
#define PITCH_SPEED_PID_KI 0.0f//20.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT     GIMBAL_MOTOR_3508_MAX_OUT//30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 0.0f//5000.0f

//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 5.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f


//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 8.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f


//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP   GIMBAL_MOTOR_3508_KP
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT GIMBAL_MOTOR_3508_MAX_OUT
#define YAW_SPEED_PID_MAX_IOUT 0.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP 5.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP 8.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 20.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define  GIMBAL_MOTOR_3508_KP          1000.0f//1500.0f
#define	 GIMBAL_MOTOR_3508_MAX_OUT     12000.0f//16000.0f

#define  ADD_SET   1000
//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201

//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YawChannel   2
#define PitchChannel 3
#define ModeChannel  0

//��ͷ180 ����
#define TurnKeyBoard KEY_PRESSED_OFFSET_F
//��ͷ��̨�ٶ�
#define TurnSpeed 0.04f

//���԰�����δʹ��
#define TestKeyBoard KEY_PRESSED_OFFSET_R

//�����������
#define VisonONKeyBoard KEY_PRESSED_OFFSET_Q
#define VisonOFFKeyBoard KEY_PRESSED_OFFSET_E

#define SwitchEnemyColor_Red_KeyBoard KEY_PRESSED_OFFSET_R
#define SwitchEnemyColor_Blue_KeyBoard KEY_PRESSED_OFFSET_G


//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband 10

//yaw��pitch�Ƕ���ң�����������
#define Yaw_RC_SEN   -0.000002f
#define Pitch_RC_SEN -0.000006f //0.005

//yaw,pitch�ǶȺ��������ı���
#define Yaw_Mouse_Sen   0.00006f
#define Pitch_Mouse_Sen 0.00002f

//��̨����������ʱ��ʹ�õı���
#define Yaw_Encoder_Sen   0.01f
#define Pitch_Encoder_Sen 0.01f

//��̨��������
#define GIMBAL_CONTROL_TIME 1

//�Ӿ�������yaw����̫�󣬵��µ����ƶ����죬����ͷ��ʧĿ��
#define   CHASSIS_FLLOW_YAW_RATE   0.2f
//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

//����Ƿ�װ
#define PITCH_TURN   0
#define YAW_TURN     0

#define OpenErrorCheck    0

#define Motor_Drive       0
#define Motor_Self_Lock   1

#define Hand_In_Level     0
#define Hand_In_Down      1
//���׵�ŷ���������
#define CYLINDER_SWITCH_ON    1   // cylinder 
#define CYLINDER_SWITCH_OFF   0

#define HandOpen     0
#define HandClose    1  

#define StepMotorRunTime 500
#define  MIN_COUNT_DELAY 500  //gimbal_task ѭ����ǧ��������ʱ����ֹ����ڲ��˿�ʼ����ģʽʱ����


////3508����Ƿ�װ
//#define LEFT_TURN 0
//#define RIGHT_TURN 0

//�������ֵ����Լ���ֵ
#define Half_ecd_range 32768
#define ecd_range 65535

//3508���RPMת��Ϊ��׼��λ�ı���
#define  UNIT_CHANGE   0.0093f     //����ʱ0.01Ϊ����ֵ 0.0093�Ƚ��ȶ�������һ��żȻֵ��
#define  RC_Send       0.001f//0.00097f     
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000


//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET   0.0f
#define INIT_PITCH_SET 0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
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

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 16000
#define MAX_RELATIVE_ANGLE 1.57079632679f     //3.141592653589793      //6.283185307179586f
#define MIN_RELATIVE_ANGLE -1.57079632679f    //-3.141592653589793     //-6.283185307179586f
#define Motor_Ecd_to_ANGLE    0.222980217528f// 0.2223243935588f //��0.5��//0.227555555555555555f //������ֵת��Ϊ�Ƕ�    8912/360/100 ����100ת1��
#define Motor_Ecd_to_ANGLE_COMPENSATE 1.02055556f
#define Motor_Ecd_to_ANGLE_OFFSET  
//�������ֵת���ɽǶ�ֵ
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad   0.000766990394f //      2*  PI  /8192
#define Step_Ecd_to_Degree 0.0054931640625f //      360��/65536


#define portMAX_DELAY   5  //��������ʱ��
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_SPEED,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

typedef enum
{
    QR_Cude_IS_UP = 0,   //��ά��������
    QR_Cude_IS_RIGHT = 1,    //��ά�����ұ�
    QR_Cude_IS_DOWN,     //��ά�����·�
	  QR_Cude_IS_LEFT,     //��ά�������
	  QR_Cude_IS_FRONT,     // ��ά������ǰ
	  QR_Cude_IS_BACK,     // ��ά���ں���
} cube_state_e;


typedef  struct
{
		/************************�Զ�������Ҫ����ı���***************************/
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
		/************************�Զ�������Ҫ����ı���***************************/
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
/***************************************���̶�����־λ************************************************/		
    u8 position_init;
	  u8 DC_motor_motionless;
		u8 hand_to_down_and_level ;
		u8 shoot_and_back_hand ; 
		u8 cylinder_up_and_down ;       
		//ץ����ת
	  u8 cale_hand_to_balance  ;      //ץ����ƽ��λ��
		u8 hand_to_balance  ;      //ץ����ƽ��λ��
		u8 hand_to_90   ;           //ץ��˳ʱ����ת90��		
		u8 hand_to_180  ;          //ץ��˳ʱ����ת180��
		u8 hand_to_minus_90 ;      //ץ����ʱ����ת90��		
		//һ���������
		u8 position_min ;          
		u8 position_500 ;       
		u8 position_700 ;
		u8 position_710 ;
		u8 position_max ;
		//ץ�ָ���
		u8 hand_go_down ;
		u8 hand_go_level ;

		u8 auto_adjust_cube;
		u8 get_cube_in_out;
		u8 official_check;  //���ڼ�¼ ץ���½���һ���ϴ�Ƕ�
		u8 get_cube_angle_set;
		uint32_t count_delay;
} Engineer_variable;
	


typedef  struct
{
	char read_hand_open_or_close ;

} Control_IO_State_t;//һ�����������ǰλ��״̬��־λ

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
    gimbal_motor_mode_e gimbal_motor_mode; //��̨���ģʽ״̬��
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd; //��̨����м�ָ
	
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
} Position_t;//һ�����������ǰλ��״̬��־λ



uint8_t *return_low_speed_mode(void);//���͵����ٶȵı�־λ
fp32 *return_DC_motor_position_set(void);


void clear_flag_tutal(void);
/**************************************/

u8 *DC_motor_stop_flag(void);

void  start_one_shot_time(void); //��������ϵͳ��ʱ����������һ���������ʱ����ֹ���һֱ��ת���ƻ���е�ṹ�����ջ����
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

