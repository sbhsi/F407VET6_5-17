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
  * @内  容  机器人轮子PID速度控制
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_speed.h"
#include "ax_robot.h"


/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 ,范围（±250）
  *          spd_current: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}	

//姿态环,pd
int Gesture(float Angle, float Gyro)
{
	float AngleBias;  //姿态角偏差
	int GPWM;  //姿态环PWM输出
	
	AngleBias=Angle-BalanAngle;  //计算姿态角偏差
	GPWM=Kp_Gesture*AngleBias+Kd_Gesture*(Gyro-0);  //计算姿态环PWM输出
	if(GPWM > 4200) GPWM = 4200;
	else if (GPWM<-4200) GPWM = -4200;
	
	return GPWM;
}
//速度环：PI控制器
int Speed(int EcA, int EcB)
{
	static float EncoderBias=0,	EB_Integral=0, EncoderBiaslast = 0;  //速度偏差与偏差积分
	int SPWM;  //速度环PWM输出
	EncoderBias=EncoderBiaslast*0.7+(EcA+EcB-0)*0.3;  //速度偏差，低通滤波
	EncoderBiaslast = EncoderBias;
	EB_Integral+=EncoderBias;  //偏差积分
	if (EB_Integral>10000) EB_Integral=10000;  //积分限幅
	if (EB_Integral<-10000) EB_Integral=-10000;  //积分限幅
	SPWM=Kp_Speed*EncoderBias+Ki_Speed*EB_Integral;  //计算速度环PWM输出
	return SPWM;
}
/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}

/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlC(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}

/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlD(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}


/******************* (C) 版权 2023 XTARK **************************************/

