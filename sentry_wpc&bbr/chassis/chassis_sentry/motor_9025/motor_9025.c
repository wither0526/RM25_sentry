#include "main.h"
#include "can.h"
#include "LK_9025.h"

//����֡��ID�͵����������IDһ����0x140+ID(1~32);

void Motor_LK9025_Enable(CAN_HandleTypeDef* hcan,uint16_t id)   //ʹ��
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x88;
	Txtemp[1] = 0x00;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	Txtemp[4] = 0x00;
	Txtemp[5] = 0x00;
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}
void Motor_LK9025_disable(CAN_HandleTypeDef* hcan,uint16_t id)   //ʧ��
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08; //���������ݳ���
	Txtemp[0] = 0x81;
	Txtemp[1] = 0x00;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	Txtemp[4] = 0x00;
	Txtemp[5] = 0x00;
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}	
void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan,uint16_t id,int16_t speed)   //�ٶȿ���
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xA1;
	Txtemp[1] = 0x00;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	
	Txtemp[4] = speed;   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = speed>>8; //��8λ
	
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_control_angle(CAN_HandleTypeDef* hcan,uint16_t id,int32_t angle)     //�Ƕȿ���
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xA3;
	Txtemp[1] = 0x00;   //˳ʱ����ʱ��
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	
	Txtemp[4] = angle;   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = angle>>8; //��8λ
	Txtemp[6] = (angle>>8)>>8;
	Txtemp[7] = ((angle>>8)>>8)>>8;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_for_angle(CAN_HandleTypeDef* hcan,uint16_t id)     //�õ���ǰ�ĽǶ�
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x94; 
	Txtemp[1] = 0x00;//0x00��0x01����˳ʱ�뻹����ʱ��
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;  //
	
	Txtemp[4] = 0x00;   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = 0x00; //��8λ
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_for_encode(CAN_HandleTypeDef* hcan,uint16_t id)     //�õ���ǰ�ĽǶ�
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x90; 
	Txtemp[1] = 0x00;//0x00��0x01����˳ʱ�뻹����ʱ��
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;  //
	
	Txtemp[4] = 0x00;   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = 0x00; //��8λ
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}






void Motor_LK9025_clear_error(CAN_HandleTypeDef* hcan,uint16_t id)   //�������֡  
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x9B;
	Txtemp[1] = 0x00;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	
	Txtemp[4] = 0x00;   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = 0x00; //��8λ
	
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}	