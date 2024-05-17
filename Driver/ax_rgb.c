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
  * @��  ��  WS2812 RGB�ƴ����� 
  *
  ******************************************************************************
  * @˵  ��
  *
  * 
  ******************************************************************************
  */

#include "ax_rgb.h"


// Ӳ��spiģ��ws2812ʱ����spi��3λ����ģ��ws2812��һλ���ݣ�
// Ҫ��SPI��ͨ��Ƶ��Ϊ2.25M������һλ���ݵ�ʱ��ԼΪ444���루ns��
//  __
// |  |_|   0b11111000  ����1
//  _   
// | |__|   0b11000000  ����0

#define TIM_ZERO          0xC0  
#define TIM_ONE           0xF8

//RGB�ʵ���ʾ����
uint8_t RGB_BYTE_Buffer[PIXEL_NUM*24+2] = {0};  //ͷ����β������0��������ʾ����


/**
  * @��  ��  RGB�ʵƳ�ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_RGB_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	//ʱ������ 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);   
	
	//���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);

	//GPIO�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	//���� SPI �ӿ�
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;    //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                  //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		              //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                      //ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                      //���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                      //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;    //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256  //40M/8=5M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                  //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                          //CRCֵ����Ķ���ʽ
	SPI_Init(SPI3, &SPI_InitStructure);

	//ʹ��SPI����
	SPI_Cmd(SPI3, ENABLE);//
	SPI_CalculateCRC(SPI3, DISABLE);
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);

	//DMA����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream5);
	
	//�ȴ�DMA������ 
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}


	//���� DMA Stream
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&SPI3->DR);//DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RGB_BYTE_Buffer;//DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize = 0;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//�е����ȼ�
	
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);  //��ʼ��DMA Stream	

}

/**
  * @��  ��  RGB�ʵƸ�������
  * @��  ��  ��
  * @����ֵ  ��
  */
void RGB_Update(void)
{
	
	//������ʾ
	DMA_SetCurrDataCounter(DMA1_Stream5, PIXEL_NUM*24+2 );     // load number of bytes to be transferred
	
	DMA_Cmd(DMA1_Stream5, ENABLE);                             // enable DMA channel 3

	while (!DMA_GetFlagStatus( DMA1_Stream5,DMA_FLAG_TCIF5));         // �ȴ��������

	DMA_Cmd(DMA1_Stream5, DISABLE); // �ر�DMAͨ��

	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);    // ���DMAͨ��״̬

	
}

/**
  * @��  ��  WS2812B����ÿ���Ƶ���ɫ
  * @��  ��  uint32_t rgb���Ƶ���ɫ
  * @����ֵ  ��
  */
void AX_RGB_SetFullColor( uint8_t r, uint8_t g, uint8_t b)
{
	
	uint16_t i,j;

	//����β����0��ֹ��ʼʱ�������ݴ�����ʱ�����
	RGB_BYTE_Buffer[0]=0;                               
	RGB_BYTE_Buffer[PIXEL_NUM*24 + 1]=0;	

	for(j=0;j<8;j++)
	{
		RGB_BYTE_Buffer[ j + 1] = ((g<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;
		RGB_BYTE_Buffer[ j + 1 + 8] = ((r<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;
		RGB_BYTE_Buffer[ j + 1 + 16] = ((b<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;			
	}

	for(i=1; i<PIXEL_NUM; i++ )
	{
		for(j=1;j<25;j++)
		{
			RGB_BYTE_Buffer[(24*i)+j] = RGB_BYTE_Buffer[j];  
		}		
	} 

	//RGB�ʵƸ�������
	RGB_Update();
}


/**
  * @��  ��  WS2812B����ÿ���Ƶ���ɫ
  * @��  ��  uint8_t pixel[PIXEL_NUM][3] rgb���� ��̬����
  * @����ֵ  ��
  */
void AX_RGB_SetPixelColor(uint8_t pixel[PIXEL_NUM][3])
{
	
	uint8_t i,j;
	
	//����β����0��ֹ��ʼʱ�������ݴ�����ʱ�����
    RGB_BYTE_Buffer[0]=0;                               
    RGB_BYTE_Buffer[PIXEL_NUM*24 + 1]=0;	
	
	for(i=0; i<PIXEL_NUM; i++ )
	{	
		for(j=0;j<8;j++)
		{
			RGB_BYTE_Buffer[(i*24) + j + 1]      = (( pixel[i][1]<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;
			RGB_BYTE_Buffer[(i*24) + j + 1 + 8]  = ((pixel[i][0]<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;
			RGB_BYTE_Buffer[(i*24) + j + 1 + 16] = (( pixel[i][2]<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;	
		}	
	}
	
	//RGB�ʵƸ�������
    RGB_Update();
}

/**
  * @��  ��  WS2812B����ÿ���Ƶ���ɫ
  * @��  ��  uint8_t pixel[PIXEL_NUM][3] rgb���� ��̬����
  * @����ֵ  ��
  */
void AX_RGB_SetPixelColor1(const uint8_t pixel[PIXEL_NUM][3])
{
	
	uint8_t i,j;
	
	//����β����0��ֹ��ʼʱ�������ݴ�����ʱ�����
    RGB_BYTE_Buffer[0]=0;                               
    RGB_BYTE_Buffer[PIXEL_NUM*24 + 1]=0;	
	
	for(i=0; i<PIXEL_NUM; i++ )
	{	
		for(j=0;j<8;j++)
		{
			RGB_BYTE_Buffer[(i*24) + j + 1]      = (( pixel[i][1]<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;
			RGB_BYTE_Buffer[(i*24) + j + 1 + 8]  = ((pixel[i][0]<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;
			RGB_BYTE_Buffer[(i*24) + j + 1 + 16] = (( pixel[i][2]<<j) & 0x0080) ? TIM_ONE:TIM_ZERO;	
		}	
	}

	//RGB�ʵƸ�������
    RGB_Update();
	
}




/******************* (C) ��Ȩ 2023 XTARK **************************************/
