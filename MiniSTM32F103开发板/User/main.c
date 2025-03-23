/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0M模块测试实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 MiniSTM32 V4开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
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
//5个数据取平均值
u32 i = 0;
uint16_t buf[5];
u32 cnt;
uint16_t sum;
u32 flag = 0;

/**
 * @brief       显示实验信息
 * @param       无
 * @retval      无
 */
void show_mesg(void)
{
    /* LCD显示实验信息 */
    lcd_show_string(10, 10, 220, 32, 32, "STM32", RED);
    lcd_show_string(10, 47, 220, 24, 24, "ATK-MS53L0M", RED);
    lcd_show_string(10, 76, 220, 16, 16, "ATOM@ALIENTEK", RED);
    lcd_show_string(10, 97, 220, 16, 16, "KEY0: read data", BLUE);
    lcd_show_string(10,118, 220, 16, 16, "KEY1: switch mode", BLUE);
    
    /* 串口输出实验信息 */
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
 * @brief      	计算拟合
 * @param       原始值
 * @retval      拟合后的值
 */
uint16_t FitData(uint16_t dat)
{
		uint16_t fit;
	
	//多项式拟合
//		fit = 10.9553 * pow(dat, 4) + 43.009 * pow( dat, 3) + 47.1402 * pow(dat, 2) + 113.1728 * dat + 226.9602;
	
	//傅里叶拟合
	fit = 501.3434 - 367.628 * cos((double)dat * 0.007) - 449.618 * sin((double)dat * 0.007) - 167.328 * cos((double)dat * 0.007 * 2) + 190.7319 * sin((double)dat * 0.007 * 2) + 45.2586 * cos((double)dat * 0.007 * 3) + 31.0833 * sin((double)dat * 0.007 * 3);
	
		return fit;
}

/**
 * @brief       均值滤波5个数据
 * @param       无
 * @retval      平均值
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

//首先定义PID结构体用于存放一个PID的数据
typedef struct
{
   	float kp, ki, kd; //三个系数
    float error, lastError; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
}PID;
 
//用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}
 
//进行一次pid计算
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//更新数据
    pid->lastError = pid->error; //将旧error存起来
    pid->error = reference - feedback; //计算新error
    //计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    float pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout+dout + pid->integral;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}
 
PID mypid = {0}; //创建一个PID结构体变量
// 
//int main()
//{
//    //...这里有些其他初始化代码
//    PID_Init(&mypid, 10, 1, 5, 800, 1000); //初始化PID参数
//    while(1)//进入循环运行
//    {
//        float feedbackValue = ...; //这里获取到被控对象的反馈值
//        float targetValue = ...; //这里获取到目标值
//        PID_Calc(&mypid, targetValue, feedbackValue); //进行PID计算，结果在output成员变量中
//        设定执行器输出大小(mypid.output);
//        delay(10); //等待一定时间再开始下一次循环
//    }
//}

int main(void)
{
    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9); /* 设置时钟, 72Mhz */
    delay_init(72);                     /* 延时初始化 */
    usart_init(115200);                 /* 串口初始化为115200 */
    led_init();                         /* 初始化LED */
    key_init();                         /* 初始化按键 */
    lcd_init();                         /* 初始化LCD */
	TIM1_PWM_Init(20000 - 1,72 - 1);				//不分频。
		ATK_MS53L0MInit();
    show_mesg();                        /* 显示实验信息 */
		uint16_t dat;
	float angle;
//    demo_run();                         /* 运行示例程序 */
//		Servo_test();
//	Servo_SetAngle(27);
	PID_Init(&mypid, 0.5, 2, 10, 800, 1000); //初始化PID参数
	
	while(1)
	{		
//			Servo_test();
			dat =	GetData();
			printf("Distance: %dmm ", dat);
			float feedbackValue = dat; //这里获取到被控对象的反馈值
      float targetValue = 200; //这里获取到目标值
			PID_Calc(&mypid, targetValue, feedbackValue); //进行PID计算，结果在output成员变量中
			printf("%f", mypid.output);
		
			angle = 54 * (mypid.output + 1000) / 2000;
			Servo_SetAngle(angle);
			printf("angle=%f", angle);
		
			printf("\r\n");
	}
}
