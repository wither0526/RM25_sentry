#ifndef  __MOTOR_DEFINE_H__
#define  __MOTOR_DEFINE_H__
#include "main.h"
#include "pid.h"

typedef struct
{
	uint16_t can_id;//���ID
    uint16_t rotor_angle;//����Ƕ�
    int16_t  rotor_speed;//����ٶ�
    int16_t  torque_current;//���Ť��
    uint8_t  temp;//�¶�
    int16_t Set_Speed;//�趨�ٶ�
    uint16_t Set_Angle;//�趨�Ƕ�
    pid_struct_t PID;//PID���ֲ���
}motor_3508_t;
typedef struct
{
    uint16_t can_id;//���ID
    int16_t  set_voltage;//�趨�ĵ�ѹֵ
    uint16_t rotor_angle;//����Ƕ�
    int16_t  rotor_speed;//����ٶ�
    int16_t  torque_current;//���Ť��
    uint8_t  temp;//�¶�
    int16_t Set_Speed;//�趨�ٶ�
    float Set_Angle;//�趨�Ƕ�
    pid_struct_t Speed_PID;    
    pid_struct_t Angle_PID;
}motor_6020_t;
typedef struct{
	uint16_t angle;
	uint16_t speed;
	uint16_t target_angle;
	uint16_t target_speed;
	pid_struct_t Speed_PID;
	pid_struct_t Angle_PID;
}motor_2006_t;
typedef struct
{
    float vx;
    float vy;
    float vw;
}Chassis_Speed_t;
void Chassis_PID_Init_All();


#endif