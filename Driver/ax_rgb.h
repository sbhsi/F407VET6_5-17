/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @内  容  WS2812 RGB灯带控制 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_RGB_H
#define __AX_RGB_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//灯个数
#define  PIXEL_NUM  8

//X-Soft 接口函数
void AX_RGB_Init(void);
void AX_RGB_SetFullColor(uint8_t b, uint8_t r, uint8_t g);
void AX_RGB_SetPixelColor(uint8_t pixel[PIXEL_NUM][3]);
void AX_RGB_SetPixelColor1( const uint8_t pixel[PIXEL_NUM][3]);


#endif

/******************* (C) 版权 2023 XTARK **************************************/
