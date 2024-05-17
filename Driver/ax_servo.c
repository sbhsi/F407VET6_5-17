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
  * @��  ��  PWM�ӿڶ������
  *
  ******************************************************************************
  * @˵  ��
  *
  ******************************************************************************
  */

#include "ax_servo.h" 

/**
  * @��  ��  ����ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S1_Init(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure; 
	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;   

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);/*ʹ��GPIOFʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);/*ʹ�ܶ�ʱ��11ʱ��*/

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM10);/*����*/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; /*����*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*�������*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8; /*PF7*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /*����*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; /**/
	GPIO_Init(GPIOB,&GPIO_InitStructure); /*��ʼ��IO*/
	
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	
	//PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //ռ�ձȳ�ʼ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	
	
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);
	

	TIM_ARRPreloadConfig(TIM10,ENABLE);/*�Զ�����Ԥװ��ʹ��*/

	TIM_Cmd(TIM10,ENABLE);/*����ʹ��*/	
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S1_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare1(TIM10,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �ͷŶ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S1_Release(void)
{
	TIM_SetCompare1(TIM10, 0);
}

/**
  * @��  ��  ����ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S2_Init(void)
{ 
	
    GPIO_InitTypeDef GPIO_InitStructure; 
	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;   

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);/*ʹ��GPIOFʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);/*ʹ�ܶ�ʱ��11ʱ��*/

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11);/*����*/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; /*����*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*�������*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9; /*PF7*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /*����*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; /**/
	GPIO_Init(GPIOB,&GPIO_InitStructure); /*��ʼ��IO*/
	
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);
	
	//PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //ռ�ձȳ�ʼ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	
	
    TIM_OC1Init(TIM11, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);
	

	TIM_ARRPreloadConfig(TIM11,ENABLE);/*�Զ�����Ԥװ��ʹ��*/

	TIM_Cmd(TIM11,ENABLE);/*����ʹ��*/	
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S2_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare1(TIM11,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �ͷŶ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S2_Release(void)
{
	TIM_SetCompare1(TIM11, 0);
}


/**
  * @��  ��  ����ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S3456_Init(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure; 
	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;   
	
	//ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	//���ù�������
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); 
	
	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
    //��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
    //-----------------------------------------------------------------------
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //ռ�ձȳ�ʼ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration: Channel3
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel4
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
    TIM_ARRPreloadConfig(TIM8, ENABLE);

    //ʹ�ܶ�ʱ��
    TIM_Cmd(TIM8, ENABLE);  
	
    //ʹ��MOEλ
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S3_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare1(TIM8,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �ͷŶ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S3_Release(void)
{
	TIM_SetCompare1(TIM8, 0);
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S4_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare2(TIM8,(1.111f*angle + 1500));
}


/**
  * @��  ��  �ͷŶ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S4_Release(void)
{
	TIM_SetCompare2(TIM8, 0);
}


/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S5_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare3(TIM8,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �ͷŶ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S5_Release(void)
{
	TIM_SetCompare3(TIM8, 0);
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S6_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare4(TIM8,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �ͷŶ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S6_Release(void)
{
	TIM_SetCompare4(TIM8, 0);
}


/******************* (C) ��Ȩ 2023 XTARK **************************************/
