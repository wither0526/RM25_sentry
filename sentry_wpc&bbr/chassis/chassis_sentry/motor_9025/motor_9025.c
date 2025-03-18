#include "main.h"
#include "can.h"
#include "LK_9025.h"

//发送帧的ID和电机反馈报文ID一样，0x140+ID(1~32);

void Motor_LK9025_Enable(CAN_HandleTypeDef* hcan,uint16_t id)   //使能
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
	Txtemp[7] = 0x00;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}
void Motor_LK9025_disable(CAN_HandleTypeDef* hcan,uint16_t id)   //失能
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08; //代表发送数据长度
	Txtemp[0] = 0x81;
	Txtemp[1] = 0x00;
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	Txtemp[4] = 0x00;
	Txtemp[5] = 0x00;
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}	
void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan,uint16_t id,int16_t speed)   //速度控制
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
	
	Txtemp[4] = speed;   //由类型决定，uint8=int16_t,这里的意思就是低8位
	Txtemp[5] = speed>>8; //高8位
	
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_control_angle(CAN_HandleTypeDef* hcan,uint16_t id,int32_t angle)     //角度控制
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xA3;
	Txtemp[1] = 0x00;   //顺时针逆时针
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;
	
	Txtemp[4] = angle;   //由类型决定，uint8=int16_t,这里的意思就是低8位
	Txtemp[5] = angle>>8; //高8位
	Txtemp[6] = (angle>>8)>>8;
	Txtemp[7] = ((angle>>8)>>8)>>8;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_for_angle(CAN_HandleTypeDef* hcan,uint16_t id)     //得到当前的角度
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x94; 
	Txtemp[1] = 0x00;//0x00和0x01决定顺时针还是逆时针
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;  //
	
	Txtemp[4] = 0x00;   //由类型决定，uint8=int16_t,这里的意思就是低8位
	Txtemp[5] = 0x00; //高8位
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}

void Motor_LK9025_for_encode(CAN_HandleTypeDef* hcan,uint16_t id)     //得到当前的角度
{  
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x90; 
	Txtemp[1] = 0x00;//0x00和0x01决定顺时针还是逆时针
	Txtemp[2] = 0x00;
	Txtemp[3] = 0x00;  //
	
	Txtemp[4] = 0x00;   //由类型决定，uint8=int16_t,这里的意思就是低8位
	Txtemp[5] = 0x00; //高8位
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}






void Motor_LK9025_clear_error(CAN_HandleTypeDef* hcan,uint16_t id)   //清除错误帧  
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
	
	Txtemp[4] = 0x00;   //由类型决定，uint8=int16_t,这里的意思就是低8位
	Txtemp[5] = 0x00; //高8位
	
	Txtemp[6] = 0x00;
	Txtemp[7] = 0x00;  //FD是退出电机，即失能
	
	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}	