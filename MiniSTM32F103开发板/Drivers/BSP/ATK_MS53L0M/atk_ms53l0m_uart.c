/**
 ****************************************************************************************************
 * @file        atk_ms53l0m_uart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0Mģ��UART�ӿ���������
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

#include "./BSP/ATK_MS53L0M/atk_ms53l0m_uart.h"
#include <string.h>
#include <math.h>

static UART_HandleTypeDef g_uart_handle;                    /* ATK-MS53L0M UART */
static struct
{
    uint8_t buf[ATK_MS53L0M_UART_RX_BUF_SIZE];              /* ֡���ջ��� */
    struct
    {
        uint16_t len    : 15;                               /* ֡���ճ��ȣ�sta[14:0] */
        uint16_t finsh  : 1;                                /* ֡������ɱ�־��sta[15] */
    } sta;                                                  /* ֡״̬��Ϣ */
} g_uart_rx_frame = {0};                                    /* ATK-MS53L0M UART����֡������Ϣ�ṹ�� */

///////���޸ķ�Χ//////////////////////////////////////////////

uint16_t DatFlag = 0;
uint16_t dat = 0;

extern uint16_t OKFlag;

//5������ȡƽ��ֵ
u32 i = 0;
uint16_t buf[5];
u32 cnt;
uint16_t sum;

float angle;

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
	
	//�߾���ģʽ����Ҷ���
	fit = 501.3434 - 367.628 * cos((double)dat * 0.007) - 449.618 * sin((double)dat * 0.007) - 167.328 * cos((double)dat * 0.007 * 2) + 190.7319 * sin((double)dat * 0.007 * 2) + 45.2586 * cos((double)dat * 0.007 * 3) + 31.0833 * sin((double)dat * 0.007 * 3);
		
	//����ģʽ����Ҷ���
	fit = 288.0581 - 193.5657 * cos((double)dat * 0.0085) - 187.9641 * sin((double)dat * 0.0085) - 89.6982 * cos((double)dat * 0.0085 * 2) + 59.9308 * sin((double)dat * 0.0085 * 2) + 7.8271 * cos((double)dat * 0.0085 * 3) + 24.8684 * sin((double)dat * 0.0085 * 3);
	
		return fit;
}

/**
 * @brief       ��ֵ�˲�5������
 * @param       dat
 * @retval      ƽ��ֵ
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


//////////////////////////////////////////////////////////////

/**
 * @brief       ATK-MS53L0M UART��������
 * @param       dat: �����͵�����
 *              len: ���������ݵĳ���
 * @retval      ��
 */
void atk_ms53l0m_uart_send(uint8_t *dat, uint8_t len)
{
    HAL_UART_Transmit(&g_uart_handle, dat, len, HAL_MAX_DELAY);
}

/**
 * @brief       ATK-MS53L0M UART���¿�ʼ��������
 * @param       ��
 * @retval      ��
 */
void atk_ms53l0m_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief       ��ȡATK-MS53L0M UART���յ���һ֡����
 * @param       ��
 * @retval      NULL: δ���յ�һ֡����
 *              ����: ���յ���һ֡����
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
 * @brief       ��ȡATK-MS53L0 UART���յ���һ֡���ݵĳ���
 * @param       ��
 * @retval      0   : δ���յ�һ֡����
 *              ����: ���յ���һ֡���ݵĳ���
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
 * @brief       ATK-MS53L0M UART��ʼ��
 * @param       baudrate: UARTͨѶ������
 * @retval      ��
 */
void atk_ms53l0m_uart_init(uint32_t baudrate)
{
    g_uart_handle.Instance          = ATK_MS53L0M_UART_INTERFACE;   /* ATK-MS53L0M UART */
    g_uart_handle.Init.BaudRate     = baudrate;                     /* ������ */
    g_uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;           /* ����λ */
    g_uart_handle.Init.StopBits     = UART_STOPBITS_1;              /* ֹͣλ */
    g_uart_handle.Init.Parity       = UART_PARITY_NONE;             /* У��λ */
    g_uart_handle.Init.Mode         = UART_MODE_TX_RX;              /* �շ�ģʽ */
    g_uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;          /* ��Ӳ������ */
    g_uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;         /* ������ */
    HAL_UART_Init(&g_uart_handle);                                  /* ʹ��ATK-ESP8266 UART
                                                                     * HAL_UART_Init()����ú���HAL_UART_MspInit()
                                                                     * �ú����������ļ�usart.c��
                                                                     */
}

/**
 * @brief       ATK-MS53L0M UART�жϻص�����
 * @param       ��
 * @retval      ��
 */
void ATK_MS53L0M_UART_IRQHandler(void)
{
    uint8_t tmp;
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_ORE) != RESET)        /* UART���չ��ش����ж� */
    {
        __HAL_UART_CLEAR_OREFLAG(&g_uart_handle);                           /* ������չ��ش����жϱ�־ */
        (void)g_uart_handle.Instance->SR;                                   /* �ȶ�SR�Ĵ������ٶ�DR�Ĵ��� */
        (void)g_uart_handle.Instance->DR;
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_RXNE) != RESET)       /* UART�����ж� */
    {
        HAL_UART_Receive(&g_uart_handle, &tmp, 1, HAL_MAX_DELAY);           /* UART�������� */
        
        if (g_uart_rx_frame.sta.len < (ATK_MS53L0M_UART_RX_BUF_SIZE - 1))   /* �ж�UART���ջ����Ƿ����
                                                                             * ����һλ��������'\0'
                                                                             */
        {
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* �����յ�������д�뻺�� */
						
						///////���޸ķ�Χ//////////////////////////////////////////////
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
							float feedbackValue = dat; //�����ȡ�����ض���ķ���ֵ
							float targetValue = 200; //�����ȡ��Ŀ��ֵ
							PID_Calc(&mypid, targetValue, feedbackValue); //����PID���㣬�����output��Ա������
						
							//������Ƕ��޸Ĵ˴�//////////////////////////////////////////////////////////////
							angle = 100 * (mypid.output + 1000) / 2000;
							///////////////////////////////////////////////////////////////////////////////////
							Servo_SetAngle(angle);
							printf("Distance: %dmm\r\n", dat);	
							
							DatFlag = 0;
							dat = 0;
						}
					}
						//////////////////////////////////////////////////////////////
					
            g_uart_rx_frame.sta.len++;                                      /* ���½��յ������ݳ��� */
        }
        else                                                                /* UART���ջ������ */
        {
            g_uart_rx_frame.sta.len = 0;                                    /* ����֮ǰ���յ����� */
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* �����յ�������д�뻺�� */
            g_uart_rx_frame.sta.len++;                                      /* ���½��յ������ݳ��� */
        }
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_IDLE) != RESET)       /* UART���߿����ж� */
    {
        g_uart_rx_frame.sta.finsh = 1;                                      /* ��־֡������� */
        
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart_handle);                          /* ���UART���߿����ж� */
    }
		
}
