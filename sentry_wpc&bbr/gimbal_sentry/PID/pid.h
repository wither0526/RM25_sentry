#ifndef __pid_h__
#define __pid_h__
#include "main.h"


typedef enum
{
    PID_POSITION = 0,//λ��ʽPID
    PID_DELTA, //����ʽPID
} PID_MODE;

typedef struct PidTypeDef
{
    float Dead_Zone; //���������ֵ
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set; //�趨ֵ
    float fdb; //����ֵ

    float out;
    float lastout;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
    int angle_max;
    int angle_min;	//�Ƕ�����ֵ ����һ��Բ�ڣ�0���360�����ڣ���max=360��min=0
    //			��һ������� 0��8192���ڣ���max=8192��min=0
    float I_Separation; //���ַ�����ֵ
    float gama;			//΢�������˲�ϵ��
    float lastdout;		//��һ��΢�����


} PidTypeDef;
typedef struct pid_struct_t
{
  float kp;//??
  float ki;//??
  float kd;//??
  float i_max;//????
  float out_max;//????
  float i_seperate;
float d_seperate;
	
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error??
 
  float p_out;//????
  float i_out;//????
  float d_out;//????
  float output;//pid???
}pid_struct_t;
 
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
//void gimbal_PID_init(void);			  
float pid_calc(pid_struct_t *pid, float ref, float fdb); 
extern float pid_caculate(PidTypeDef *pid, const float ref, const float set);
void PID_clear(PidTypeDef *pid);
void pid_reset(PidTypeDef	*pid, float PID[3]);
extern void pid_param_init(PidTypeDef *pid, uint8_t mode, float PID[3], float max_out, float max_iout, float I_Separation, float Dead_Zone, float gama, float angle_max, float angle_min);
int Limit_Min_Max(int value,int min,int max);
void Pid_Protect(pid_struct_t *pid);
float PID_Calc_Angle(pid_struct_t *PID, float ref, float fdb);
float PID_Calc_Speed(pid_struct_t *PID, float ref, float fdb);
float float_Limit_Min_Max(float value,float min,float max);
#endif
