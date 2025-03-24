/**
 ****************************************************************************************************
 * @file        demo.c
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

#include "demo.h"
#include "./BSP/ATK_MS53L0M/atk_ms53l0m.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LCD/lcd.h"
#include <stdio.h>

uint16_t Id;

/**
 * @brief       显示设备地址
 * @param       无
 * @retval      无
 */
static void demo_show_id(uint16_t id)
{
    char buf[23];
    
    sprintf(buf, "ATK-MS53L0M ID: 0x%04x", id);
    
    printf("%s\r\n", buf);
    lcd_show_string(32, 151, 176, 16, 16, buf, BLUE);
}

/**
 * @brief       按键0功能，获取ATK-MS53L0M测量值
 * @param       is_normal: 0，Modbus模式
 *                         1，Normal模式
 *              device_id: ATK-MS53L0M设备地址
 * @retval      无
 */
static void demo_key0_fun(uint8_t is_normal, uint16_t device_id)
{
    uint8_t ret;
    uint16_t dat;
    
    if (is_normal == 0)
    {
        /* ATK-MS53L0M Modbus工作模式获取测量值 */
        ret = atk_ms53l0m_modbus_get_data(device_id, &dat);
        if (ret == 0)
        {
            printf("[Modbus]Distance: %dmm\r\n", dat);
        }
        else
        {
            printf("Modbus mode get data failed!\r\n");
        }
    }
    else
    {
        /* ATK-MS53L0M Normal工作模式获取测量值 */
        ret = atk_ms53l0m_normal_get_data(&dat);
        if (ret == 0)
        {
            printf("[Normal]Distance: %dmm\r\n", dat);
        }
        else
        {
            printf("Normal mode get data failed!\r\n");
        }
    }
}

/**
 * @brief       按键1功能，切换ATK-MS53L0M工作模式
 * @param       is_normal: 0，Modbus模式
 *                         1，Normal模式
 *              device_id: ATK-MS53L0M设备地址
 * @retval      无
 */
static void demo_key1_fun(uint8_t *is_normal, uint16_t device_id)
{
    uint8_t ret;
    
    if (*is_normal == 0)
    {
        /* 设置ATK-MS53L0M的工作模式为Normal模式 */
        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_NORMAL);
        if (ret == 0)
        {
            /* 设置ATK-MS53L0M的回传速率为5Hz */
            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_BACKRATE, ATK_MS53L0M_BACKRATE_5HZ);
            /* 设置ATK-MS53L0M的测量模式为长距离模式 */
            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_LONG);
						/* 设置ATK-MS53L0M的测量模式为高精度模式 */
//						atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_HIPRECI);
            
            *is_normal = 1;
            printf("Set to Normal mode.\r\n");
        }
        else
        {
            printf("Set Normal failed!\r\n");
        }
    }
    else
    {
        /* 设置ATK-MS53L0M的工作模式为Modbus模式 */
        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_MODBUS);
        if (ret == 0)
        {
            /* 设置ATK-MS53L0M的测量模式为高速测量模式 */
//            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_HISPEED);
						/* 设置ATK-MS53L0M的测量模式为高精度模式 */
						atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_HIPRECI);
            
            *is_normal = 0;
            printf("Set to Modbus mode.\r\n"); 
        }
        else
        {
            printf("Set Modbus mode failed!\r\n");
        }
    }
}

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t ret;
    uint8_t key;
    uint16_t id;
    uint8_t is_normal = 0;
    
    /* 初始化ATK-MS53L0M */
    ret = atk_ms53l0m_init(115200, &id);
    if (ret != 0)
    {
        printf("ATK-MS53L0M init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* ATK-MS53L0M初始化成功，显示设备地址 */
    demo_show_id(id);
    
    while (1)
    {
        key = key_scan(0);
				demo_key0_fun(is_normal, id);
				delay_ms(300);
        
        switch (key)
        {
            case KEY0_PRES:
            {
                /* 获取ATK-MS53L0M测量值 */
                demo_key0_fun(is_normal, id);
                break;
            }
            case KEY1_PRES:
            {
                /* 切换ATK-MS53L0M工作模式 */
                demo_key1_fun(&is_normal, id);
                break;
            }
            default:
            {
                break;
            }
        }
        
        delay_ms(10);
    }
}

/**
 * @brief       ATK-MS53L0M初始化
 * @param       无
 * @retval      无
 */
void ATK_MS53L0MInit(void)
{
    uint8_t ret;
    uint16_t id;
    
    /* 初始化ATK-MS53L0M */
    ret = atk_ms53l0m_init(115200, &id);
//	printf("ret =  %dmm\r\n", ret);
    if (ret != 0)
    {
        printf("11111ATK-MS53L0M init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* ATK-MS53L0M初始化成功，显示设备地址 */
    demo_show_id(id);
    Id = id;
		
		/* 设置ATK-MS53L0M的工作模式为Normal模式 */
//		ret = atk_ms53l0m_write_data(Id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_NORMAL);
////		/* 设置ATK-MS53L0M的回传速率为5Hz */
////			atk_ms53l0m_write_data(Id, ATK_MS53L0M_FUNCODE_BACKRATE, ATK_MS53L0M_BACKRATE_20HZ);
////			/* 设置ATK-MS53L0M的测量模式为高速模式 */
//		atk_ms53l0m_write_data(Id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_HISPEED);
}

/**
 * @brief       ATK-MS53L0M工作函数
 * @param       无
 * @retval      距离
 */
uint16_t ATK_MS53L0MWork(void)
{
		uint8_t ret;
    uint16_t dat;
	
////		/* ATK-MS53L0M Modbus工作模式获取测量值 */
//		ret = atk_ms53l0m_modbus_get_data(Id, &dat);
	
	
	/* ATK-MS53L0M Normal工作模式获取测量值 */
        ret = atk_ms53l0m_normal_get_data(&dat);
	
	
		if (ret == 0)
		{
//				printf("[Modbus]Distance: %dmm\r\n", dat);	
				return dat;
		}
		else
		{
				printf("Get data failed!\r\n");
		}

		return 0;
		
}
