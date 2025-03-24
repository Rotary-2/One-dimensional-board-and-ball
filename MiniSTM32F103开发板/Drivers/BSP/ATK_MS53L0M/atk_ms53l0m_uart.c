/**
 ****************************************************************************************************
 * @file        atk_ms53l0m_uart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0M模块UART接口驱动代码
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

#include "./BSP/ATK_MS53L0M/atk_ms53l0m_uart.h"
#include <string.h>
#include <math.h>

static UART_HandleTypeDef g_uart_handle;                    /* ATK-MS53L0M UART */
static struct
{
    uint8_t buf[ATK_MS53L0M_UART_RX_BUF_SIZE];              /* 帧接收缓冲 */
    struct
    {
        uint16_t len    : 15;                               /* 帧接收长度，sta[14:0] */
        uint16_t finsh  : 1;                                /* 帧接收完成标志，sta[15] */
    } sta;                                                  /* 帧状态信息 */
} g_uart_rx_frame = {0};                                    /* ATK-MS53L0M UART接收帧缓冲信息结构体 */

///////可修改范围//////////////////////////////////////////////

uint16_t DatFlag = 0;
uint16_t dat = 0;

extern uint16_t OKFlag;

//5个数据取平均值
u32 i = 0;
uint16_t buf[5];
u32 cnt;
uint16_t sum;

float angle;

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
	
	//高精度模式傅里叶拟合
	fit = 501.3434 - 367.628 * cos((double)dat * 0.007) - 449.618 * sin((double)dat * 0.007) - 167.328 * cos((double)dat * 0.007 * 2) + 190.7319 * sin((double)dat * 0.007 * 2) + 45.2586 * cos((double)dat * 0.007 * 3) + 31.0833 * sin((double)dat * 0.007 * 3);
		
	//高速模式傅里叶拟合
	fit = 288.0581 - 193.5657 * cos((double)dat * 0.0085) - 187.9641 * sin((double)dat * 0.0085) - 89.6982 * cos((double)dat * 0.0085 * 2) + 59.9308 * sin((double)dat * 0.0085 * 2) + 7.8271 * cos((double)dat * 0.0085 * 3) + 24.8684 * sin((double)dat * 0.0085 * 3);
	
		return fit;
}

/**
 * @brief       均值滤波5个数据
 * @param       dat
 * @retval      平均值
 */
uint16_t GetData(uint16_t dat)
{
		u32 j;
		uint16_t aver;
		
		i++;
		if (i >= 5) 
		{
				i = 0;
		}
			
		dat = FitData(dat);
		buf[i] = dat;

		sum = 0;
		cnt = 0;
	
		for (j = 0; j < 5; j++)
		{
				if (buf[i] - buf[j] < 10 && buf[i] - buf[j] > -10)
				{
						sum += buf[j];
						cnt++;
				}
		}
		
		if (cnt)
		{
				aver = sum / cnt;
				return aver;
		}
		
		return dat;
}
 
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


//////////////////////////////////////////////////////////////

/**
 * @brief       ATK-MS53L0M UART发送数据
 * @param       dat: 待发送的数据
 *              len: 待发送数据的长度
 * @retval      无
 */
void atk_ms53l0m_uart_send(uint8_t *dat, uint8_t len)
{
    HAL_UART_Transmit(&g_uart_handle, dat, len, HAL_MAX_DELAY);
}

/**
 * @brief       ATK-MS53L0M UART重新开始接收数据
 * @param       无
 * @retval      无
 */
void atk_ms53l0m_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief       获取ATK-MS53L0M UART接收到的一帧数据
 * @param       无
 * @retval      NULL: 未接收到一帧数据
 *              其他: 接收到的一帧数据
 */
uint8_t *atk_ms53l0m_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }
    else
    {
        return NULL;
    }
}

/**
 * @brief       获取ATK-MS53L0 UART接收到的一帧数据的长度
 * @param       无
 * @retval      0   : 未接收到一帧数据
 *              其他: 接收到的一帧数据的长度
 */
uint16_t atk_ms53l0m_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        return g_uart_rx_frame.sta.len;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief       ATK-MS53L0M UART初始化
 * @param       baudrate: UART通讯波特率
 * @retval      无
 */
void atk_ms53l0m_uart_init(uint32_t baudrate)
{
    g_uart_handle.Instance          = ATK_MS53L0M_UART_INTERFACE;   /* ATK-MS53L0M UART */
    g_uart_handle.Init.BaudRate     = baudrate;                     /* 波特率 */
    g_uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;           /* 数据位 */
    g_uart_handle.Init.StopBits     = UART_STOPBITS_1;              /* 停止位 */
    g_uart_handle.Init.Parity       = UART_PARITY_NONE;             /* 校验位 */
    g_uart_handle.Init.Mode         = UART_MODE_TX_RX;              /* 收发模式 */
    g_uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;          /* 无硬件流控 */
    g_uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;         /* 过采样 */
    HAL_UART_Init(&g_uart_handle);                                  /* 使能ATK-ESP8266 UART
                                                                     * HAL_UART_Init()会调用函数HAL_UART_MspInit()
                                                                     * 该函数定义在文件usart.c中
                                                                     */
}

/**
 * @brief       ATK-MS53L0M UART中断回调函数
 * @param       无
 * @retval      无
 */
void ATK_MS53L0M_UART_IRQHandler(void)
{
    uint8_t tmp;
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_ORE) != RESET)        /* UART接收过载错误中断 */
    {
        __HAL_UART_CLEAR_OREFLAG(&g_uart_handle);                           /* 清除接收过载错误中断标志 */
        (void)g_uart_handle.Instance->SR;                                   /* 先读SR寄存器，再读DR寄存器 */
        (void)g_uart_handle.Instance->DR;
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_RXNE) != RESET)       /* UART接收中断 */
    {
        HAL_UART_Receive(&g_uart_handle, &tmp, 1, HAL_MAX_DELAY);           /* UART接收数据 */
        
        if (g_uart_rx_frame.sta.len < (ATK_MS53L0M_UART_RX_BUF_SIZE - 1))   /* 判断UART接收缓冲是否溢出
                                                                             * 留出一位给结束符'\0'
                                                                             */
        {
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* 将接收到的数据写入缓冲 */
						
						///////可修改范围//////////////////////////////////////////////
						if (OKFlag){
						if (tmp == 'd') DatFlag = 1;
						if (DatFlag)
						{
							if (tmp >= '0' && tmp <= '9')
							{
									dat = dat * 10 + (tmp - '0');
							}
						}
						if (tmp == 'm' && DatFlag)
						{			
							dat =	GetData(dat);
							float feedbackValue = dat; //这里获取到被控对象的反馈值
							float targetValue = 200; //这里获取到目标值
							PID_Calc(&mypid, targetValue, feedbackValue); //进行PID计算，结果在output成员变量中
						
							//舵机最大角度修改此处//////////////////////////////////////////////////////////////
							angle = 100 * (mypid.output + 1000) / 2000;
							///////////////////////////////////////////////////////////////////////////////////
							Servo_SetAngle(angle);
							printf("Distance: %dmm\r\n", dat);	
							
							DatFlag = 0;
							dat = 0;
						}
					}
						//////////////////////////////////////////////////////////////
					
            g_uart_rx_frame.sta.len++;                                      /* 更新接收到的数据长度 */
        }
        else                                                                /* UART接收缓冲溢出 */
        {
            g_uart_rx_frame.sta.len = 0;                                    /* 覆盖之前接收的数据 */
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* 将接收到的数据写入缓冲 */
            g_uart_rx_frame.sta.len++;                                      /* 更新接收到的数据长度 */
        }
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_IDLE) != RESET)       /* UART总线空闲中断 */
    {
        g_uart_rx_frame.sta.finsh = 1;                                      /* 标志帧接收完成 */
        
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart_handle);                          /* 清除UART总线空闲中断 */
    }
		
}
