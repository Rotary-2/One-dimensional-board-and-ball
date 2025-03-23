#include "pwm.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//��ʱ���ж���������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2019/11/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
TIM_HandleTypeDef 	TIM1_Handler;      	//��ʱ����� 
TIM_OC_InitTypeDef 	TIM1_CH1Handler;	//��ʱ��1ͨ��1���


//TIM1 PWM���ֳ�ʼ�� 
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
    TIM1_Handler.Instance=TIM1;         	//��ʱ��1
    TIM1_Handler.Init.Prescaler=psc;       	//��ʱ����Ƶ
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM1_Handler.Init.Period=arr;          	//�Զ���װ��ֵ
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM1_Handler);       	//��ʼ��PWM
    
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM1_CH1Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_LOW; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//����TIM1ͨ��1
	
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);//����PWMͨ��1
	 	   
}

//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			//ʹ�ܶ�ʱ��1
		__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
		
		GPIO_Initure.Pin=GPIO_PIN_8;           	//PA8
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;          //����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
	}
}

//����TIM1ͨ��1��ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM1Compare1(u32 compare)
{
	TIM1->CCR1=compare; 
}

//��ʱ��1�жϷ�����
void TIM1_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM1_Handler);
}

//�Ƕȿ��ƶ�� ��Χ 0 - 54��
void Servo_SetAngle(float Angle)
{
		//180�ȶ�� �˴����ƿ�ת���Ƕ�Ϊ54��
		u32 range = 54;
	
		//�����趨�����ת���Ƕȹ�����װ��
		if ( range > 54) range = 54;
	
		//���⴫�νǶȹ�����װ��
		if( Angle < 0) Angle = 0;
		if (Angle > range) Angle = range;
	
		//����CCR
		u32 ConvertAngle = 2000 * (Angle + (180 - range) / 2) / 180 + 17500;

	TIM_SetTIM1Compare1(ConvertAngle);	
	
}

//������Ժ���
void Servo_test(void)
{
		u32 i;
		for (i = 0; i < 54; i++)
		{
				Servo_SetAngle(i);
				delay_ms(50);
		}
		for (i = 54; i > 0; i--)
		{
				Servo_SetAngle(i);
				delay_ms(50);
		}
}

