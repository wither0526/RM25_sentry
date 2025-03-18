#include "main.h"
#include "remote.h"
#include "can.h"


//�����£� 1  3  2

uint8_t pdata[18];

RC_Ctl_t RC_CtrlData;     //��������û�м�ʧ������

void RemoteDataProcess(uint8_t *pData) //��UART�Ļص������ﱻ���ã�������pData����Ϣ������RC
{
 if(pData == NULL)
 {
 return;
 }
 
 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
 RC_CtrlData.rc.ch0=RC_CtrlData.rc.ch0-1024;
 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) 
& 0x07FF;
 RC_CtrlData.rc.ch1=RC_CtrlData.rc.ch1-1024;
 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
 ((int16_t)pData[4] << 10)) & 0x07FF;
 RC_CtrlData.rc.ch2=RC_CtrlData.rc.ch2-1024;   
 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 
0x07FF;
 RC_CtrlData.rc.ch3=RC_CtrlData.rc.ch3-1024;
 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
 RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
 RC_CtrlData.mouse.press_l = pData[12];
 RC_CtrlData.mouse.press_r = pData[13];
 RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
 RC_CtrlData.wheel=(pData[16] | pData[17] << 8) - 1024;
 
 if(RC_CtrlData.rc.ch0>1000 || RC_CtrlData.rc.ch1>1000 || RC_CtrlData.rc.ch2>1000 || RC_CtrlData.rc.ch3>1000){
	 RC_CtrlData.rc.ch0=0;
	 RC_CtrlData.rc.ch1=0;
	 RC_CtrlData.rc.ch2=0;
	 RC_CtrlData.rc.ch3=0;
 }
 
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8] = {0};
    
    tx_header.StdId = 0x11F;//��ʶ�������ֲ�P6��
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	tx_data[0] = pData[0];
    tx_data[1] = pData[1];
	
	tx_data[2] = pData[2];
	tx_data[3]= pData[5];
	
	tx_data[4]=pData[16];
	tx_data[5]=pData[17];
//    tx_data[4] = ((int16_t)GM6020_Yaw.Speed_PID.output>>8)&0xff;
//    tx_data[5] = ((int16_t)GM6020_Yaw.Speed_PID.output)&0xff;
    
    //tx_data[0] = ((int16_t)GM6020_Pitch.Speed_PID.output>>8)&0xff;
    //tx_data[1] = ((int16_t)GM6020_Pitch.Speed_PID.output)&0xff;   //��Ϊid��ԭ������������

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
	
}//���뺯��,���еĽ��뺯����һ�����ӣ��������е�PID�㷨����һ��ģ�������Ƽ��ɣ�

