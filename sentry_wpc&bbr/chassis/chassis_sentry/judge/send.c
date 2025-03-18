#include "main.h"
#include "can.h"
#include "judge.h"
#include "cmsis_os.h"



extern frame_t judge_frame_rx;
extern frame_t judge_frame_tx;
//Student_Data receive_robot_interactive_data.data = {0};
extern uint8_t receive_student_data[113];
extern uint8_t judge_rx_buff[JUDGE_MAX_LENGTH];
//judge_show_data_t    Show_data = {0};//�ͻ�����Ϣ
extern robot_type_t robot_type;

//bool Judge_Data_TF = false;//���������Ƿ���ã�������������
//uint8_t Judge_Self_ID;//��ǰ�����˵�ID
//uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID
///**************����ϵͳ���ݸ���****************/
//uint16_t ShootNum=0;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
//uint16_t Shoot2Num=0;
//bool Hurt_Data_Update = false;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
//uint16_t Hurt_num = 0;
//#define BLUE  0
//#define RED   1
union speed_zidan {
	float speed;
    uint8_t bytes[sizeof(float)]; // ??float??????
};
union speed_zidan speed_sentry;

union target_place {
    struct {
        float place_x;
		float place_y;
    } places;
    uint8_t bytes[sizeof(float) * 2]; // ??float??????
};
union target_place target_robot;

union game_time {
	uint16_t remain_time;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union game_time computition_type;

union money_remain {
	uint16_t money;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union money_remain money_game;


union game_time computition_type;



union friendplace {
    struct {
        float place_x;
		float place_y;
    } places;
    uint8_t bytes[sizeof(float) * 2]; // ??float??????
};
union friendplace infantry_1_place,infantry_2_place,hero_place,engineer_place,sentry_place;
union friendHP_red {
    struct {
		uint16_t hp;
    } shengming;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};

union friendHP_red infantry_1_hp_red,infantry_2_hp_red,infantry_3_hp_red,hero_hp_red,engineer_hp_red,sentry_hp_red,outpost_red_hp,base_red_hp;

union friendHP_blue {
    struct {
		uint16_t hp;
    } shengming;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};

union friendHP_blue infantry_1_hp_blue,infantry_2_hp_blue,infantry_3q_hp_blue,hero_hp_blue,engineer_hp_blue,sentry_hp_blue,outpost_blue_hp,base_blue_hp;

union shoot_17 {
	uint16_t shoot_remain;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union shoot_17 remain_zidan;

extern uint8_t yaw_flag_hit;	

uint8_t hit_flag=0;

union shoot_heat {
	uint16_t heat;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union shoot_heat heat_2006,limit_17_2006;


void send_all_judge(){
/***********************�ҷ�������*************************/	
	
	
	
	/*******1�Ų������� ******/
		CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8] = {0};
    
    tx_header.StdId = 0x320;//��ʶ�������ֲ�P6��   //4�Ų���
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	infantry_1_place.places.place_x=judge_frame_rx.data.robot_position.standard_4_x;
	infantry_1_place.places.place_y=judge_frame_rx.data.robot_position.standard_4_y;
	
	tx_data[0] = infantry_1_place.bytes[0];
    tx_data[1] = infantry_1_place.bytes[1];
	
	tx_data[2] = infantry_1_place.bytes[2];
	tx_data[3]= infantry_1_place.bytes[3];
	
	tx_data[4]=infantry_1_place.bytes[4];
	tx_data[5]=infantry_1_place.bytes[5];
	
	tx_data[6]=infantry_1_place.bytes[6];
	tx_data[7]=infantry_1_place.bytes[7];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);

	osDelay(1);
/******����*****/
    tx_header.StdId = 0x321;//��ʶ�������ֲ�P6��    //5�Ų���
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	engineer_place.places.place_x=judge_frame_rx.data.robot_position.engineer_x;
	engineer_place.places.place_y=judge_frame_rx.data.robot_position.engineer_y;
	
	tx_data[0] = engineer_place.bytes[0];
    tx_data[1] = engineer_place.bytes[1];
	
	tx_data[2] = engineer_place.bytes[2];
	tx_data[3] = engineer_place.bytes[3];
	
	tx_data[4] = engineer_place.bytes[4];
	tx_data[5] = engineer_place.bytes[5];

	tx_data[6] = engineer_place.bytes[6];
	tx_data[7] = engineer_place.bytes[7];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
///*******5����******/	
	tx_header.StdId = 0x322;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	infantry_2_place.places.place_x=judge_frame_rx.data.robot_position.standard_5_x;
	infantry_2_place.places.place_y=judge_frame_rx.data.robot_position.standard_5_y;
	
	tx_data[0] = infantry_2_place.bytes[0];
    tx_data[1] = infantry_2_place.bytes[1];
	
	tx_data[2] = infantry_2_place.bytes[2];
	tx_data[3] = infantry_2_place.bytes[3];
	
	tx_data[4] = infantry_2_place.bytes[4];
	tx_data[5] = infantry_2_place.bytes[5];
	
	tx_data[6] = infantry_2_place.bytes[6];
	tx_data[7] = infantry_2_place.bytes[7];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
/******Ӣ��λ��********/	
	
	tx_header.StdId = 0x323;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	hero_place.places.place_x=judge_frame_rx.data.robot_position.hero_x;
	hero_place.places.place_y=judge_frame_rx.data.robot_position.hero_y;
	
	tx_data[0] = hero_place.bytes[0];
    tx_data[1] = hero_place.bytes[1];
	
	tx_data[2] = hero_place.bytes[2];
	tx_data[3] = hero_place.bytes[3];
	
	tx_data[4] = hero_place.bytes[4];
	tx_data[5] = hero_place.bytes[5];
	
	tx_data[6] = hero_place.bytes[6];
	tx_data[7] = hero_place.bytes[7];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
	tx_header.StdId = 0x324;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
//0x203
	sentry_place.places.place_x=judge_frame_rx.data.game_robot_pos.x;
	sentry_place.places.place_y=judge_frame_rx.data.game_robot_pos.y;
	
	tx_data[0] = sentry_place.bytes[0];
    tx_data[1] = sentry_place.bytes[1];
	
	tx_data[2] = sentry_place.bytes[2];
	tx_data[3] = sentry_place.bytes[3];
	
	tx_data[4] = sentry_place.bytes[4];
	tx_data[5] = sentry_place.bytes[5];
	
	tx_data[6] = sentry_place.bytes[6];
	tx_data[7] = sentry_place.bytes[7];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
///*****************Ѫ������****************/
///////red///////

////judge_frame_rx.data.game_robot_HP.red_1_robot_HP //�췽Ӣ��
////judge_frame_rx.data.game_robot_HP.red_2_robot_HP //�췽����
////judge_frame_rx.data.game_robot_HP.red_3_robot_HP //��3����
////judge_frame_rx.data.game_robot_HP.red_4_robot_HP //��4����
////judge_frame_rx.data.game_robot_HP.red_5_robot_HP //��5����
////judge_frame_rx.data.game_robot_HP.red_5_robot_HP //��7�ڱ�
////judge_frame_rx.data.game_robot_HP.red_outpost_HP //��ǰ��ս
////judge_frame_rx.data.game_robot_HP.red_base_HP//�����


////judge_frame_rx.data.game_robot_HP.blue_1_robot_HP //lan��Ӣ��
////judge_frame_rx.data.game_robot_HP.blue_2_robot_HP //lan������
////judge_frame_rx.data.game_robot_HP.blue_3_robot_HP //lan3����
////judge_frame_rx.data.game_robot_HP.blue_4_robot_HP //lan4����
////judge_frame_rx.data.game_robot_HP.blue_5_robot_HP //lan5����
////judge_frame_rx.data.game_robot_HP.blue_5_robot_HP //lan7�ڱ�
////judge_frame_rx.data.game_robot_HP.blue_outpost_HP //lanǰ��ս
////judge_frame_rx.data.game_robot_HP.blue_base_HP //lan����
///*****����**********/

	hero_hp_red.shengming.hp=judge_frame_rx.data.game_robot_HP.red_1_robot_HP;
	engineer_hp_red.shengming.hp=judge_frame_rx.data.game_robot_HP.red_2_robot_HP;
	infantry_1_hp_red.shengming.hp=judge_frame_rx.data.game_robot_HP.red_3_robot_HP;
	infantry_2_hp_red.shengming.hp=judge_frame_rx.data.game_robot_HP.red_4_robot_HP;
	sentry_hp_red.shengming.hp=judge_frame_rx.data.game_robot_HP.red_7_robot_HP;
	
	hero_hp_blue.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_1_robot_HP;
	engineer_hp_blue.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_2_robot_HP;
	infantry_1_hp_blue.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_3_robot_HP;
	infantry_2_hp_blue.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_4_robot_HP;
	sentry_hp_blue.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_7_robot_HP;
	
	outpost_red_hp.shengming.hp=judge_frame_rx.data.game_robot_HP.red_outpost_HP;
	base_red_hp.shengming.hp=judge_frame_rx.data.game_robot_HP.red_base_HP;
	outpost_blue_hp.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_base_HP;
	base_blue_hp.shengming.hp=judge_frame_rx.data.game_robot_HP.blue_base_HP;
	
	tx_header.StdId = 0x330;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	tx_data[0] = hero_hp_red.bytes[0];
    tx_data[1] = hero_hp_red.bytes[1];
	
	tx_data[2] = engineer_hp_red.bytes[0];
	tx_data[3] = engineer_hp_red.bytes[1];
	
	tx_data[4] = infantry_1_hp_red.bytes[0];
	tx_data[5] = infantry_1_hp_red.bytes[1];
	
	tx_data[6] = infantry_2_hp_red.bytes[0];
	tx_data[7] = infantry_2_hp_red.bytes[1];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
	
	
	
	tx_header.StdId = 0x331;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	
	tx_data[0] = sentry_hp_red.bytes[0];
    tx_data[1] = sentry_hp_red.bytes[1];
	
	tx_data[2] = outpost_red_hp.bytes[0];
	tx_data[3] = outpost_red_hp.bytes[1];
	
	tx_data[4] = base_red_hp.bytes[0];
	tx_data[5] = base_red_hp.bytes[1];
	
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
	
	
	tx_header.StdId = 0x332;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	
	tx_data[0] = hero_hp_blue.bytes[0];
    tx_data[1] = hero_hp_blue.bytes[1];
	
	tx_data[2] = engineer_hp_blue.bytes[0];
	tx_data[3] = engineer_hp_blue.bytes[1];
	
	tx_data[4] = infantry_1_hp_blue.bytes[0];
	tx_data[5] = infantry_1_hp_blue.bytes[1];
	
	tx_data[6] = infantry_2_hp_blue.bytes[0];
	tx_data[7] = infantry_2_hp_blue.bytes[1];
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
	
	tx_header.StdId = 0x333;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	
	tx_data[0] = sentry_hp_blue.bytes[0];
    tx_data[1] = sentry_hp_blue.bytes[1];
	
	tx_data[2] = 0;
	tx_data[3] = 0;
	
	tx_data[4] = base_blue_hp.bytes[0];
	tx_data[5] = base_blue_hp.bytes[1];
	
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
/***************************����Ѫ��*********************************/	
/********************************������Ϣ***************************************/
//judge_frame_rx.data.bullet_remaining
//judge_frame_rx.data.event_data.event_type

//judge_frame_rx.data.game_status
money_game.money=judge_frame_rx.data.bullet_remaining.coin_remaining_num;
computition_type.remain_time=judge_frame_rx.data.game_status.stage_remain_time;
	tx_header.StdId = 0x350;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	
	tx_data[0] = judge_frame_rx.data.game_status.game_progress;
    tx_data[1] = computition_type.bytes[0];
	
	tx_data[2] = computition_type.bytes[1];
	tx_data[3] = money_game.bytes[0];
	
	tx_data[4] = money_game.bytes[1];
	tx_data[5] = 0;
	
	tx_data[6] = 0;
	
	tx_data[7] = 0;
	
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);

	/********************************��������***************************************/
//judge_frame_rx.data.bullet_remaining
//
target_robot.places.place_x=judge_frame_rx.data.smallmap_communicate.target_position_x;
target_robot.places.place_y=judge_frame_rx.data.smallmap_communicate.target_position_y;
	tx_header.StdId = 0x360;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	
	tx_data[0] = target_robot.bytes[0];
    tx_data[1] = target_robot.bytes[1];
	
	tx_data[2] = target_robot.bytes[2];
	tx_data[3] = target_robot.bytes[3];
	
	tx_data[4] = target_robot.bytes[4];
	tx_data[5] = target_robot.bytes[5];
	
	tx_data[6] = target_robot.bytes[6];
	tx_data[7] = target_robot.bytes[7];
	
	
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
	tx_header.StdId = 0x361;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	
	tx_data[0] = judge_frame_rx.data.smallmap_communicate.commd_keyboard;
    tx_data[1] = judge_frame_rx.data.smallmap_communicate.target_robot_ID;
	
	tx_data[2] = 0;
	tx_data[3] = 0;
	
	tx_data[4] = 0;
	tx_data[5] = 0;
	
	tx_data[6] = 0;
	tx_data[7] = 0;
	

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
/*****************************�ܻ�����********************************/

		tx_header.StdId = 0x370;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	if( yaw_flag_hit==1 ){
		tx_data[0] = 1;
	}
	else{
		tx_data[0] = 0;
	}
	
    tx_data[1] = 0;
	
	tx_data[2] = 0;
	tx_data[3] = 0;
	
	tx_data[4] = 0;
	tx_data[5] = 0;
	
	tx_data[6] = 0;
	tx_data[7] = 0;
	

	//yaw_flag_hit=0;

	hit_flag++;
	
	if(hit_flag>10){
		yaw_flag_hit=0;
	}
	
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
/***************************����״̬��*******************************/
speed_sentry.speed=judge_frame_rx.data.shoot_data.bullet_speed;
remain_zidan.shoot_remain=judge_frame_rx.data.bullet_remaining.bullet_remaining_num_17mm;

		tx_header.StdId = 0x371;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	tx_data[0] = remain_zidan.bytes[0];
    tx_data[1] = remain_zidan.bytes[1];
	
	tx_data[2] = 0;
	tx_data[3] = 0;
	
	tx_data[4] = speed_sentry.bytes[0];
	tx_data[5] = speed_sentry.bytes[1];
	
	tx_data[6] = speed_sentry.bytes[2];
	tx_data[7] = speed_sentry.bytes[3];
	

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
	
	heat_2006.heat=judge_frame_rx.data.power_heat_data.shooter_id1_17mm_cooling_heat;
	limit_17_2006.heat=judge_frame_rx.data.game_robot_status.shooter_id1_17mm_cooling_limit*0.8;
	tx_header.StdId = 0x372;//��ʶ�������ֲ�P6��     
    tx_header.IDE   = CAN_ID_STD;//��׼ID
    tx_header.RTR   = CAN_RTR_DATA;//����֡
    tx_header.DLC   = 8;//�ֽڳ���
	
	
	tx_data[0] = heat_2006.bytes[0];
    tx_data[1] = heat_2006.bytes[1];
	
	tx_data[2] = limit_17_2006.bytes[0];
	tx_data[3] = limit_17_2006.bytes[1];
	
	tx_data[4] = 0;
	tx_data[5] = 0;
	
	tx_data[6] = 0;
	tx_data[7] = 0;
	

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	osDelay(1);
}
void send_judge(void const * argument)
{
    portTickType currentTime;
	osDelay(1);
    for(;;)
    { 
        currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		
		send_all_judge();
		
		osDelay(100);
		
        vTaskDelayUntil(&currentTime,2);//������ʱ
    }

}


//	/*******1�Ų������� ******/
	

//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//��ʶ�������ֲ�P6��
//    tx_header.IDE   = CAN_ID_STD;//��׼ID
//    tx_header.RTR   = CAN_RTR_DATA;//����֡
//    tx_header.DLC   = 8;//�ֽڳ���
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);