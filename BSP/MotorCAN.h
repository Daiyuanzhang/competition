#ifndef __MOTORCAN_H_
#define __MOTORCAN_H_
#include "stm32f4xx.h"
#include "pid_modify.h"
#include "main.h"
#define SEND_ID201_204 0x200
#define SEND_ID205_208 0X1FF
#define M2006_REDUCTIONRTION 36
#define M3510_REDUCTIONRTION  27

//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    uint8_t data_lenth;
    uint8_t encoder_ID;
    uint8_t func_cmd;
    uint32_t ecd;

} external_encoder_t;
/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
//	  CAN_LEFT_HAND_MOTOR_ID  = 0x207,
//    CAN_RIGHT_HAND_MOTOR_ID = 0x208,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

typedef enum
{
	//can2ͨѶ
	EXTER_ENCODER = 0x01,
	CAN2_HAND_MOTOR_3508    = 0x204,  //ץ�ֹؽ�ѡװ���
//	CAN2_HAND_PITCH_3510_ID = 0x202,  //ץ�ָ������
	CAN_LEFT_HAND_MOTOR_ID  = 0x201,  //ץ�ִ��ʹ����
	CAN_RIGHT_HAND_MOTOR_ID = 0x202,  //ץ�ִ��ʹ����
//	CAN_TRIGGER_42_MOTOR_ID = 0x207,
 
}can2_msg_id_e;


typedef struct Motor_SpeedLoopData
{
	int16_t getspeed;  			 
	int16_t setSpeed;	
	pid_t pid;
	uint32_t onlinecnt;
}Motor_SpeedLoopData_t;

struct Encoder
{
  int16_t cnt;
  int16_t lastValue;
  int16_t previousValue;
  int16_t currValue;
};

typedef struct  Motor_Posi_LoopData
{ 
    struct Encoder	encoder_p;
	int16_t encoderInitValue;
	int16_t setPosition;
	int16_t getPosition;
	pid_t pid;
}Motor_Posi_LoopData_t ;

typedef struct  Motor_Posi_A_Speed_LoopData
{
	struct  Motor_Posi_LoopData position;
	struct Motor_SpeedLoopData speed;
	uint32_t onlinecnt;
}Motor_Posi_A_Speed_LoopData_t;


//ץ�ָ��������������
const external_encoder_t *get_Hand_Pitch_Measure_Point(void);

void MoterCanInit(void);
void CanSendMess(CAN_TypeDef* CANx,uint32_t SendID,int16_t *message);
//�����ö���������ʹ��
void 	EncoderCan2SendMess(CAN_TypeDef* CANx,uint32_t SendID,uint8_t *message);
//��ȡץ�ֹؽڴ���3508���������
const motor_measure_t *get_Hand_Motor_3508_Measure_Point(void);

//��ȡ�����ֱ۵��������
const motor_measure_t *get_RightHand_Motor_Measure_Point(void);
const motor_measure_t *get_LeftHand_Motor_Measure_Point(void);



const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_shoot_Motor_Measure_Point(uint8_t i);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Fric_Motor_Measure_Point(uint8_t i);

//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
void CAN_hook(CAN_TypeDef* CANx,CanRxMsg *rx_message);

#endif

