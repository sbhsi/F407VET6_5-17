/**			                                                    
		   ____                    _____ _______ _____       XTARK@���˴���
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   �����˿�����
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ��˾��վ�� www.xtark.cn   www.tarkbot.com
  * �Ա����̣� https://xtark.taobao.com  
  * ����΢�ţ� ���˴��£���ע���ںţ���ȡ���¸�����Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  TTL����ͨ��
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.USB����ͨ�ţ�printf�����Ѷ��򵽸ô��ڣ��������������Ϣ
  * 2.����ʹ�þ���X-ProtocolЭ��������ݷ��͡�
  * 3.����UART�Ĵ������ݽ��չ��ܣ�ʹ���жϷ�ʽ��X-ProtocolЭ��ͨ��
  * 4.��ͨ��AX_UART_DB_GetRxData()�����ж��Ƿ������ݽ���
  *
  * X-ProtocolЭ����ܣ���֡����
  * ֡���壺AA 55 | 0B  | 01  | 03 E8 FC 18 00 0A | 14
  *        ֡ͷ   ֡��   ֡��  ����                У���
  * ֡  ͷ��˫֡ͷ��������ǿ
  * ֡  �����������ݳ����趨
  * ֡  �룺�û����ݹ����趨����ʶ֡��Ψһ��
  * ��  �ݣ���λ��ǰ�����ȿɱ䣬�����������8λ��16λ��32λ����
  * У��ͣ�ǰ�������ۼӺ͵ĵ�8λ
  * ֡ʾ����( AA 55 0B 01 03 E8 FC 18 00 0A 14 ) ���ݣ�1000��-1000,10,
  * 
  ******************************************************************************
  */

#include "ax_uart2.h"
#include <stdio.h>

#include "ax_robot.h"

static uint8_t uart2_rx_con=0;       //���ռ�����
static uint8_t uart2_rx_checksum;    //֡ͷ����У���
static uint8_t uart2_rx_buf[40];     //���ջ��壬��������С�ڵ���32Byte
static uint8_t uart2_tx_buf[40];     //���ͻ���

/**
  * @��  ��  UART   ���ڳ�ʼ��
  * @��  ��  baud�� ����������
  * @����ֵ	 ��
  */
void AX_UART2_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;


	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); 

	//USART �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	//USART��������
	USART_InitStructure.USART_BaudRate = baud;    //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	//USARTʹ��
	USART_Cmd(USART2, ENABLE); 
	
	//�������ڽ����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
	
    //USART2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	
}

/**
  * @��  ��  UART �����жϷ�����
  * @��  ��  �� 
  * @����ֵ  ��
  */
void USART2_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Res =USART_ReceiveData(USART2);	
		
		if(uart2_rx_con < 3)    //==����֡ͷ + ����
		{
			if(uart2_rx_con == 0)  //����֡ͷ1 0xAA
			{
				if(Res == 0xAA)
				{
					uart2_rx_buf[0] = Res;
					uart2_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart2_rx_con == 1) //����֡ͷ2 0x55
			{
				if(Res == 0x55)
				{
					uart2_rx_buf[1] = Res;
					uart2_rx_con = 2;
				}
				else
				{
					uart2_rx_con = 0;						
				}				
			}
			else  //�������ݳ���
			{
				uart2_rx_buf[2] = Res;
				uart2_rx_con = 3;
				uart2_rx_checksum = (0xAA+0x55) + Res;	//����У���
			}
		}
		else    //==��������
		{
			if(uart2_rx_con < (uart2_rx_buf[2]-1) )
			{
				uart2_rx_buf[uart2_rx_con] = Res;
				uart2_rx_con++;
				uart2_rx_checksum = uart2_rx_checksum + Res;					
			}
			else    //�ж����1λ
			{
				//������ɣ��ָ���ʼ״̬
				uart2_rx_con = 0;	
				
				//����У��
				if( Res == uart2_rx_checksum )  //У����ȷ
				{	
					
					//����ΪROS����ģʽ
					ax_control_mode = CTL_ROS;
					
					//�ٶȿ���֡
					if(uart2_rx_buf[3] == ID_URX_VEL)
					{
						R_Vel.TG_IX = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
						R_Vel.TG_IY = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
						R_Vel.TG_IW = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
						
						
						//����ǰ����������ˣ�����ʸ���ٶȼ���ǰ��ת���
						#if (ROBOT_TYPE == ROBOT_AKM)
							ax_akm_angle = AX_AKM_WToAngle(R_Vel.TG_IX, R_Vel.TG_IW);
						#endif
					}
					else
					{
						//IMU������У׼
						if(uart2_rx_buf[3] == ID_URX_IMU)
						{
							ax_imu_calibrate_flag = uart2_rx_buf[4];
						}	

						//RGB��Ч����֡
						if(uart2_rx_buf[3] == ID_URX_LG)
						{
							R_Light.M  = uart2_rx_buf[4];
							R_Light.S  = uart2_rx_buf[5];
							R_Light.T  = uart2_rx_buf[6];
							R_Light.R  = uart2_rx_buf[7];
							R_Light.G  = uart2_rx_buf[8];
							R_Light.B  = uart2_rx_buf[9];
						}	

						//��ЧEEPROM����
						if(uart2_rx_buf[3] == ID_URX_LS)
						{
							//ִ�еƹ�Ч�����涯��
							ax_light_save_flag = 1;
						}	
						
						//���������п���
						if(uart2_rx_buf[3] == ID_URX_BP)
						{
							//����Ϊ1����������
							if(uart2_rx_buf[4] != 0)
							{
								AX_BEEP_On();
							}
							else
							{
								AX_BEEP_Off();
							}
						}						
						
						//�����˵����ͺ���Ϣ�������ж��Ƿ�һ��
						if(uart2_rx_buf[3] == ID_URX_RTY)
						{
							//�ж��뵱ǰ�����Ƿ�һ�£�
							if(uart2_rx_buf[4] == ROBOT_TYPE  )
							{
								//һ�£����������������ʾ
								ax_beep_ring = BEEP_SHORT;
							}
							else
							{
								//��һ�£�������������б���
								ax_beep_ring = BEEP_LONG;								
							}
						}						
					}		
				}
			}
		}
		
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	} 
}


/**
  * @��  ��  UART �������ݣ�X-ProtocolЭ�飩
  * @��  ��  *pbuf����������ָ��
  *          len���������ݳ��ȸ�������27 (32-5)
  *          num��֡�ţ�֡����
  * @����ֵ	 ��
  */
void AX_UART2_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//����У���
	
	if(len <= 32)
	{
		/******��ȡ����******/
		uart2_tx_buf[0] = 0xAA;    //֡ͷ
		uart2_tx_buf[1] = 0x55;    //
		uart2_tx_buf[2] = len+5;  //����������ȼ���֡����
		uart2_tx_buf[3] = num;    //֡����
		
		for(i=0; i<len; i++)
		{
			uart2_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******����У���******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart2_tx_buf[i];
		}
		uart2_tx_buf[i] = tx_checksum;
		
		
		/******��������******/	
		cnt = 5+len;
		
		//��ѯ���䷽ʽ
		for(i=0; i<cnt; i++)
		{
			USART_SendData(USART2, uart2_tx_buf[i]);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
		}
	}
}



/******************* (C) ��Ȩ 2023 XTARK **************************************/
