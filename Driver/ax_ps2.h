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
  * @��  ��  PS2�����ֱ������ļ�
  *
  ******************************************************************************
  * @˵  ��
  *
  *   PS2���ݶ���
  *   BYTE   DATA   ����
  *   01     idle
  *   02     0x73   �ֱ�����ģʽ
  *   03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
  *   04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
  *   05     data   L2     R2     L1    R1   Y     B     A      X
  *   06     data   �ұ�ҡ��  0x00 = ��    0xff = ��
  *   07     data   �ұ�ҡ��  0x00 = ��    0xff = ��
  *   08     data   ���ҡ��  0x00 = ��    0xff = ��
  *   09     data   ���ҡ��  0x00 = ��    0xff = ��
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PS2_H
#define __AX_PS2_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//PS2��������
#define  PS2_BT1_SELECT     0x01    //ѡ�񰴼�
#define  PS2_BT1_JOY_L      0x02    //��ҡ�˰���
#define  PS2_BT1_JOY_R      0x04    //��ҡ�˰��� 
#define  PS2_BT1_START      0x08    //��������
#define  PS2_BT1_UP         0x10    //���� �ϰ���
#define  PS2_BT1_RIGHT      0x20    //���� �Ұ���
#define  PS2_BT1_DOWN       0x40    //���� �°��� 
#define  PS2_BT1_LEFT       0x80    //���� �󰴼�

#define  PS2_BT2_L2         0x01    //ѡ�񰴼�
#define  PS2_BT2_R2         0x02    //��ҡ�˰���
#define  PS2_BT2_L1         0x04    //��ҡ�˰��� 
#define  PS2_BT2_R1         0x08    //��������
#define  PS2_BT2_Y          0x10    //���� Y����(������) 
#define  PS2_BT2_B          0x20    //���� B����(Բ��)
#define  PS2_BT2_A          0x40    //���� A����(���)
#define  PS2_BT2_X          0x80    //���� X����(����)


//PS2�ֱ���ֵ���ݽṹ��	 
typedef struct			 				
{
  uint8_t mode;		    /* �ֱ��Ĺ���ģʽ */

  uint8_t btn1;         /* B0:SLCT B1:JR  B0:JL B3:STRT B4:UP B5:R B6:DOWN  B7:L   */

  uint8_t btn2;         /* B0:L2   B1:R2  B2:L1 B3:R1   B4:Y  B5:B B6:A     B7:X */

  uint8_t RJoy_LR;      /* �ұ�ҡ��  0x00 = ��    0xff = ��   */

  uint8_t RJoy_UD;      /* �ұ�ҡ��  0x00 = ��    0xff = ��   */

  uint8_t LJoy_LR;      /* ���ҡ��  0x00 = ��    0xff = ��   */

  uint8_t LJoy_UD;      /* ���ҡ��  0x00 = ��    0xff = ��   */
	
}JOYSTICK_TypeDef;


/*** PS2�����ֱ��������� **********/
void AX_PS2_Init(void);  //PS2��ʼ��
void AX_PS2_ScanKey(JOYSTICK_TypeDef* JoystickStruct);//PS2��ȡ������ҡ����ֵ

#endif 

/******************* (C) ��Ȩ 2023 XTARK **************************************/
