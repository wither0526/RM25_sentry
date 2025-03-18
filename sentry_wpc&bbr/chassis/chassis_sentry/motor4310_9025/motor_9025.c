#include "main.h"
#include "can.h"
#include "LK_9025.h"

//����֡��ID�͵����������IDһ����0x140+ID(1~32);
void Motor_LK9025_Enable_baozha1i(CAN_HandleTypeDef* hcan,uint16_t id,uint8_t bitt)   //ʹ��
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x8C;
	Txtemp[1] = bitt;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	Txtemp[4] = 0x00;
	Txtemp[5] = 0x00;
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1);
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2);
}
void Motor_LK9025_baozha1i_out_post(CAN_HandleTypeDef* hcan,uint16_t id,uint8_t bitt)   //ʹ��
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x8C;
	Txtemp[1] = bitt;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	Txtemp[4] = 0x00;
	Txtemp[5] = 0x00;
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1);
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2);
}


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
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1);
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2);
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

uint8_t speedControl[4];

void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan,uint16_t id,int32_t speed)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xA2;
	Txtemp[1] = 0x00;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	
	Txtemp[4] = *(uint8_t *)(&speed);   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = *((uint8_t *)(&speed)+1); //��8λ
	
	Txtemp[6] = *((uint8_t *)(&speed)+2);
	Txtemp[7] = *((uint8_t *)(&speed)+3);  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_control_FN(CAN_HandleTypeDef* hcan,uint16_t id,int16_t speed)   //ת�رջ�����   //-2048~+2048
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
	
	Txtemp[0] = 0xA3;  // ????
	Txtemp[1] = 0x00;  // NULL
	Txtemp[2] = 0x00;  // NULL
	Txtemp[3] = 0x00;  // NULL
	Txtemp[4] = *(uint8_t *)(&angle);  // ???????
	Txtemp[5] = *((uint8_t *)(&angle)+1);  // ????
	Txtemp[6] = *((uint8_t *)(&angle)+2);  // ????
	Txtemp[7] = *((uint8_t *)(&angle)+3);  // ???????
	
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK){
				if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2)!=HAL_OK){
		
		
		}
		
		}
	}
}
void Motor_LK9025_control_angle_pro(CAN_HandleTypeDef* hcan,uint16_t id,int32_t angle,uint16_t maxspeed)     //�Ƕȿ���
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	Txtemp[0] = 0xA4;  // ????
	Txtemp[1] = 0x00;  // NULL
	Txtemp[2] = *(uint8_t *)(&maxspeed);  // NULL
	Txtemp[3] = *(uint8_t *)((&maxspeed)+1);  // NULL
	Txtemp[4] = *(uint8_t *)(&angle);  // ???????
	Txtemp[5] = *((uint8_t *)(&angle)+1);  // ????
	Txtemp[6] = *((uint8_t *)(&angle)+2);  // ????
	Txtemp[7] = *((uint8_t *)(&angle)+3);  // ???????
	
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK){
				if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2)!=HAL_OK){
		
		
		}
		
		}
	}
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

void Motor_LK9025_for_speed(CAN_HandleTypeDef* hcan,uint16_t id)     //�õ���ǰ���ٶ�
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x9C; 
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

void Motor_LK9025_angle_more(CAN_HandleTypeDef* hcan,uint16_t id,int32_t anglecrement)     //�õ���ǰ�ĽǶ�
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xA7; 
	Txtemp[1] = 0x00;//0x00��0x01����˳ʱ�뻹����ʱ��
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;  //
	
	Txtemp[4] = *(uint8_t *)(& anglecrement) ;   //�����;�����uint8=int16_t,�������˼���ǵ�8λ
	Txtemp[5] = *((uint8_t *)(& anglecrement)+1); //��8λ
	Txtemp[6] = *((uint8_t *)(& anglecrement)+2);
	Txtemp[7] = *((uint8_t *)(& anglecrement)+3);  //FD���˳��������ʧ��
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}
