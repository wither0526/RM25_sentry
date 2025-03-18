#ifndef  __VISION_H__
#define  __VISION_H__
#include "main.h"

typedef struct
{
	uint8_t type;   //0����  1�����   2������
	uint8_t content;  //��������
}computer_on;

typedef struct
{
	uint8_t type;//0����  1��С����  2����̨��������
	uint8_t content;   //��������
}computer_off;

typedef struct
{
	float x_speed;   //x���ٶȣ�y���ٶ�
	float y_speed;
}computer_chassis;

typedef struct
{
	char fine_bool;   //�Ƿ�׷��
	float big_yaw;    //yaw��Ƕ�
	float pitch;    //pitch��Ƕ�
	float xiao_yaw;
}computer_zimiao;

typedef struct
{
	computer_zimiao zimiao;
	computer_chassis computer_vpm;
	computer_on computer_juece;
	computer_off computer_exercise;
}computer_all;



typedef struct
{
	float x_speed;   //x���ٶȣ�y���ٶ�
	float y_speed;
}motor_chassis;


void vision_call_back_handler(uint8_t *buff);
void vision_transmit_package_AutoaimFeedback (uint8_t *buff);
void vision_transmit();
void vision_transmit_judge();

#endif