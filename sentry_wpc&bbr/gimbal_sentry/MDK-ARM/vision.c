#include "main.h"



void vision_call_back_handler_Left_Control(uint8_t *buff)//���Ӿ��������ݽ��д���
{

    if( buff[0]=='s' && buff[31]=='e' )
    {
        if(buff[1] == 0xA0) // ����̨����
        {
            check_ok++;  //������ȷ����
            vision_missing_time = 0;  //����ʧ��ʱ��
					        
					
						VisionValue.find_bool = buff[2];  //�Ƿ�ʶ��Ŀ��
						memcpy(&(VisionValue.vision_left_yaw_value.buff[0]),&buff[3],4);  //yaw ƫ����
						memcpy(&(VisionValue.vision_left_pitch_value.buff[0]),&buff[7],4);  // pitch ƫ����
						memcpy(&(VisionValue.vision_Left_Gimbal_distance.buff[0]),&buff[11],4);  // װ�װ����
				
        }
    }
		if( (VisionValue.vision_left_yaw_value.value > 100 ) || (VisionValue.vision_left_pitch_value.value > 100 ) )
    {
        VisionValue.vision_left_yaw_value.value = 0 ;
        VisionValue.vision_left_pitch_value.value = 0 ;
    }        
}	
