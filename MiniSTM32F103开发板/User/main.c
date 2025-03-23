/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0Mģ�����ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� MiniSTM32 V4������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LCD/lcd.h"
#include "demo.h"
#include "pwm.h"
#include <math.h>
//5������ȡƽ��ֵ
u32 i = 0;
uint16_t buf[5];
u32 cnt;
uint16_t sum;
u32 flag = 0;

/**
 * @brief       ��ʾʵ����Ϣ
 * @param       ��
 * @retval      ��
 */
void show_mesg(void)
{
    /* LCD��ʾʵ����Ϣ */
    lcd_show_string(10, 10, 220, 32, 32, "STM32", RED);
    lcd_show_string(10, 47, 220, 24, 24, "ATK-MS53L0M", RED);
    lcd_show_string(10, 76, 220, 16, 16, "ATOM@ALIENTEK", RED);
    lcd_show_string(10, 97, 220, 16, 16, "KEY0: read data", BLUE);
    lcd_show_string(10,118, 220, 16, 16, "KEY1: switch mode", BLUE);
    
    /* �������ʵ����Ϣ */
    printf("\n");
    printf("********************************\r\n");
    printf("STM32\r\n");
    printf("ATK-MS53L0M\r\n");
    printf("ATOM@ALIENTEK\r\n");
    printf("KEY0: read data\r\n");
    printf("KEY1: switch mode\r\n");
    printf("********************************\r\n");
    printf("\r\n");
}

/**
 * @brief      	�������
 * @param       ԭʼֵ
 * @retval      ��Ϻ��ֵ
 */
uint16_t FitData(uint16_t dat)
{
		uint16_t fit;
	
	//����ʽ���
//		fit = 10.9553 * pow(dat, 4) + 43.009 * pow( dat, 3) + 47.1402 * pow(dat, 2) + 113.1728 * dat + 226.9602;
	
	//����Ҷ���
	fit = 501.3434 - 367.628 * cos((double)dat * 0.007) - 449.618 * sin((double)dat * 0.007) - 167.328 * cos((double)dat * 0.007 * 2) + 190.7319 * sin((double)dat * 0.007 * 2) + 45.2586 * cos((double)dat * 0.007 * 3) + 31.0833 * sin((double)dat * 0.007 * 3);
	
		return fit;
}

/**
 * @brief       ��ֵ�˲�5������
 * @param       ��
 * @retval      ƽ��ֵ
 */
uint16_t GetData()
{
		u32 j;
		uint16_t dat;
		uint16_t aver;
		
		i++;
		if (i >= 5) 
		{
				i = 0;
				flag = 1;
		}
		
		if (ATK_MS53L0MWork()) 
		{
				dat = ATK_MS53L0MWork();
//			  printf("dat = %d  ", dat);
			//	dat = FitData(dat);
//				printf("fit = %d\r\n", dat);
				buf[i] = dat;
		}
		else 
		{
				return 0;
		}
		
		if (flag) 
		{
				sum = 0;
				cnt = 0;
			
				for (j = 0; j < 5; j++)
				{
						if (buf[i] - buf[j] < 10 && buf[i] - buf[j] > -10)
						{
								sum += buf[j];
								cnt++;
//								printf("buf[%d] = %d", j, buf[j]);
						}
				}
				
//				printf("\r\n");
				aver = sum / cnt;
				return aver;
		}
		
		return dat;
}

//���ȶ���PID�ṹ�����ڴ��һ��PID������
typedef struct
{
   	float kp, ki, kd; //����ϵ��
    float error, lastError; //���ϴ����
    float integral, maxIntegral; //���֡������޷�
    float output, maxOutput; //���������޷�
}PID;
 
//���ڳ�ʼ��pid�����ĺ���
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}
 
//����һ��pid����
//����Ϊ(pid�ṹ��,Ŀ��ֵ,����ֵ)������������pid�ṹ���output��Ա��
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//��������
    pid->lastError = pid->error; //����error������
    pid->error = reference - feedback; //������error
    //����΢��
    float dout = (pid->error - pid->lastError) * pid->kd;
    //�������
    float pout = pid->error * pid->kp;
    //�������
    pid->integral += pid->error * pid->ki;
    //�����޷�
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //�������
    pid->output = pout+dout + pid->integral;
    //����޷�
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}
 
PID mypid = {0}; //����һ��PID�ṹ�����
// 
//int main()
//{
//    //...������Щ������ʼ������
//    PID_Init(&mypid, 10, 1, 5, 800, 1000); //��ʼ��PID����
//    while(1)//����ѭ������
//    {
//        float feedbackValue = ...; //�����ȡ�����ض���ķ���ֵ
//        float targetValue = ...; //�����ȡ��Ŀ��ֵ
//        PID_Calc(&mypid, targetValue, feedbackValue); //����PID���㣬�����output��Ա������
//        �趨ִ���������С(mypid.output);
//        delay(10); //�ȴ�һ��ʱ���ٿ�ʼ��һ��ѭ��
//    }
//}

int main(void)
{
    HAL_Init();                         /* ��ʼ��HAL�� */
    sys_stm32_clock_init(RCC_PLL_MUL9); /* ����ʱ��, 72Mhz */
    delay_init(72);                     /* ��ʱ��ʼ�� */
    usart_init(115200);                 /* ���ڳ�ʼ��Ϊ115200 */
    led_init();                         /* ��ʼ��LED */
    key_init();                         /* ��ʼ������ */
    lcd_init();                         /* ��ʼ��LCD */
	TIM1_PWM_Init(20000 - 1,72 - 1);				//����Ƶ��
		ATK_MS53L0MInit();
    show_mesg();                        /* ��ʾʵ����Ϣ */
		uint16_t dat;
	float angle;
//    demo_run();                         /* ����ʾ������ */
//		Servo_test();
//	Servo_SetAngle(27);
	PID_Init(&mypid, 0.5, 2, 10, 800, 1000); //��ʼ��PID����
	
	while(1)
	{		
//			Servo_test();
			dat =	GetData();
			printf("Distance: %dmm ", dat);
			float feedbackValue = dat; //�����ȡ�����ض���ķ���ֵ
      float targetValue = 200; //�����ȡ��Ŀ��ֵ
			PID_Calc(&mypid, targetValue, feedbackValue); //����PID���㣬�����output��Ա������
			printf("%f", mypid.output);
		
			angle = 54 * (mypid.output + 1000) / 2000;
			Servo_SetAngle(angle);
			printf("angle=%f", angle);
		
			printf("\r\n");
	}
}
