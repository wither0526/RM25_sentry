/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             ��Ҫ����������bmi088��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  //---------------------------�����ǳ��������ͷ�ڽ�����-----------------------------------//
  *2022.7.02 �����ȵ�Ҫ�� ����ܲ�����
  *1�����ڵĴ���yaw�������Ʈ��� ������ô������ܽ�������Ǵ�����������������ǰ�ڵ���
  *Ʈʮ�ֳ��󣨸ó�һ��debugֵ������Ʈ1�ȶ࣬ʮ�ֳ���
  *2�����ô����ƾ��ǳ�ʼ�������Ʋ��Ұ�ist8310_read_mag(ist8310_real_data.mag)����ȡ��ע��
  *�������� ����ȡ��ע�Ϳ���Ч�� Ȼ����������Ϊʲô�������ˣ����ţ�
  *3���Աȹٷ�����ᷢ�ֹٷ�������gyro_update_flag����IMU_NOTIFY_SHFITS
  *���������� �������ﳭ�����ó���IMU_UPDATE_SHFITS ��������Ϊʲô ���ԸĻ�ȥ���ԣ�΢Ц.jpg��
  *4���Ǹ��ٷ����ԵĽ����Ʈ�Ĵ���û������ ��ֻҪϸ������ �ͻᷢ��ֵ����û�������������ٷ��궨����
  *�������������У׼������������ ���ǲ�û�е����Ǹ�У׼���� ��֮���ǹٷ���������ûдҲ��֪����
  *�����ڸ�ʲô���Ƿ���ʹ���ˣ�
  *5���䰮���� Զ�������� ����
  */
	//--------------------------2022.8.30----------------------------------------------------//
	/*1���ô����ƽ������Ư���⣬�����˺ܶ��������⣺
	��һ��������ҪУ׼����ȡ��������matlab����ͼ����һ������ƫ���پ������������ˡ�
	�ڶ�����������ʹų��������أ��и���Ƭ����C��5cm���Ҿͻ�ɵ���
	�����ƾͽ�����һλ��Ե���ˣ���Ҫ�����Խ����ˡ�
	*/

#include "INS_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_imu_pwm.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "bsp_spi.h"
#include "../PID/pid.h"

//#include "GimbleTask.h"


#include "MahonyAHRS.h"
#include "arm_math.h"


#define    TIME_STAMP_1MS        1
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm����
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
	
			
/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
//static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void);


fp32 get_yaw_ist8310(ist8310_real_data_t *ist8310,fp32 pitch,fp32 roll);
void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
volatile bool_t imu_init_finish_flag = 0;
volatile bool_t gimbal_return_finish_flag = 0;

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PidTypeDef imu_temp_pid;
static const float timing_time = 0.001f;   //tast run time , unit s.�������е�ʱ�� ��λ s

//���ٶȼƵ�ͨ�˲�
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.ŷ���� ��λ rad
fp32 IMU_angle[3] = {0.0f, 0.0f, 0.0f};
fp32 Last_ol[3];

float *IMU_kf_result;
float yaw_add_one_tick=0.000065515769;//-0.000000005769f
float yaw_offset;
				//����Ʈ
int pre_tick = 0;
int tick;

float angle_init=130;
void IMU_InitArgument(void)
{
    while(BMI088_init())
    {
        osDelay(100);
    }
		while(ist8310_init())
		{
			osDelay(100);
		}
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    pid_param_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT,3e38,0,0,0,0);

    AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = bmi088_real_data.accel[0];//p
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = bmi088_real_data.accel[1];//yoll
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = bmi088_real_data.accel[2];//z

    //get the handle of task
    //��ȡ��ǰ�������������
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
}
/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

float add_angle_yaw = 0;
void INS_task(void const *pvParameters)
{
    uint16_t count = 0;
		portTickType currentTime;
    IMU_InitArgument();
    while(1)
    {
				currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
        //wait spi DMA tansmit done
        //�ȴ�SPI DMA����
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if(!imu_init_finish_flag)
            count++;

        if(!imu_init_finish_flag && count > 500)
        {
            imu_init_finish_flag = 1;
            count = 700;
        }

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        //���ٶȼƵ�ͨ�˲�
        /*accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + bmi088_real_data.accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + bmi088_real_data.accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + bmi088_real_data.accel[2] * fliter_num[2];*/
				
//				imu_cali_slove(bmi088_real_data.gyro,accel_fliter_3,ist8310_real_data.mag,&bmi088_real_data,&ist8310_real_data);
        //AHRS_update(INS_quat, timing_time, bmi088_real_data.gyro, accel_fliter_3, ist8310_real_data.mag);
		AHRS_update(INS_quat, timing_time, bmi088_real_data.gyro, accel_fliter_3, ist8310_real_data.mag);

        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
        //get_angle(INS_quat, INS_angle, INS_angle, INS_angle);

        tick = HAL_GetTick();
				if (pre_tick)
				yaw_offset +=  yaw_add_one_tick * (tick - pre_tick);
		
        IMU_angle[0] = 180.0f*INS_angle[0]/PI + yaw_offset; //yaw   yaw_offset
		//IMU_angle[0] = 180.0f*INS_angle[0]/PI;
        IMU_angle[1] = 180.0f*INS_angle[1]/PI; //pitch
        IMU_angle[2] = 180.0f*INS_angle[2]/PI; //roll
		//IMU_angle[0]=IMU_angle[0]+180;
				pre_tick = tick; 
				


				
      
        vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);
    }
}

fp32 get_yaw_ist8310(ist8310_real_data_t *ist8310,fp32 pitch,fp32 roll)
{
	fp32 mx,my;
	mx = ist8310->mag[0] * cos(pitch) + ist8310->mag[1] * sin(pitch) * sin(roll) - ist8310->mag[2] * sin(pitch) * cos(roll);
	my = ist8310->mag[1] * cos(roll) + ist8310->mag[2] * sin(roll);
	return atan2(my,mx);
}


void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

/**
  * @brief          ������������Ư
  * @param[out]     gyro_offset:������Ư
  * @param[in]      gyro:���ٶ�����
  * @param[out]     offset_time_count: �Զ���1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
	
}

//������Ԫ�ؽ������ŷ����
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        pid_caculate(&imu_temp_pid, temp, bmi088_real_data.temp);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        //in beginning, max power
        if (temp > bmi088_real_data.temp)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

//            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //��������
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_1)
    {
        gimbal_return_finish_flag = 1;
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    //���������ǵ�DMA����
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //�������ٶȼƵ�DMA����
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}
uint32_t count_accel = 0;
uint32_t count_gyro = 0;
uint32_t count_temp = 0;
void ins_dma_data_read(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //�����Ƕ�ȡ���
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
						count_gyro++;
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        //accel read over
        //���ٶȼƶ�ȡ���
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
						count_accel++;
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //�¶ȶ�ȡ���
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
						count_temp++;
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}


