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
  * @内  容  阿克曼机器人处理函数文件
  ******************************************************************************
  * @说  明
  *
  * 
  ******************************************************************************
  */

#include "ax_akm.h"

#include "ax_robot.h"

/**
  * @简  述  根据阿克曼前进和转向速度转换为前轮转向角度
  * @参  数  vx  前进速度，单位m/s 
  *          vw  转向速度，单位rad/s
  * @返回值  右前轮转向角度，单位rad
  */
int16_t AX_AKM_WToAngle(int16_t vx, int16_t vw) 
{
    float akm_angle; //前轮转向角度
	float radius;  //转弯半径

	if(vw!=0 && vx!=0)
	{
		//计算转弯半径
		radius =  (float)vx/(float)vw;
		
		//转弯半径小于最小转弯
		if(radius>0 && radius<AKM_TURN_R_MINI)
		{
			radius = AKM_TURN_R_MINI; 
		}
			
		if(radius<0 && radius>(-AKM_TURN_R_MINI))
		{
			radius = -AKM_TURN_R_MINI;
		}		

		//计算机器人前轮转向角度,单位弧度
		akm_angle = atan(AKM_ACLE_BASE/(radius));
	}
	else
	{
		akm_angle = 0;
	}
	
	return (akm_angle*1000);
}

/******************* (C) 版权 2023 XTARK **************************************/
