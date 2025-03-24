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
#include "./BSP/ATK_MS53L0M/atk_ms53l0m.h"
#include "./BSP/ATK_MS53L0M/atk_ms53l0m_uart.h"
#include "demo.h"
#include "pwm.h"

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

PID mypid = {0}; //创建一个PID结构体变量

int main(void)
{
	uint16_t OKFlag = 0;
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

	PID_Init(&mypid, 3, 0.25, 10, 800, 1000); //初始化PID参数

	while(1)
	{		

	}
}
